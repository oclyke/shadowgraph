// shadowgraph demo: trace a morphing Lissajous "ballywhoop" on the galvos while
// slowly cycling the laser color through the HSV hue wheel at 25% intensity.
// Exercises the full laser_engine path end to end: the point ring, the
// fixed-rate timer ISR consumer, and galvo + laser DAC writes.
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#include "dac8871_idf6.h"
#include "dacx0004_idf6.h"
#include "isr_spi.h"
#include "laser_engine.h"
#include "wifi_ap.h"

static const char *TAG = "shadowgraph";

#define ENABLE_WIFI 0

// ---------------------------------------------------------------------------
// Galvo SPI bus  (SPI2 / HSPI) — two DAC8871s share the bus
// ---------------------------------------------------------------------------
#define GALVO_SPI_HOST   SPI2_HOST
#define GALVO_PIN_MOSI   13
#define GALVO_PIN_CLK    14
#define GALVO_CLK_HZ     40000000

// Galvo X  (DAC8871, U$2)
#define GX_PIN_CS        21
#define GX_PIN_LDAC      19
#define GX_PIN_RST       22

// Galvo Y  (DAC8871, U$1)
#define GY_PIN_CS        16
#define GY_PIN_LDAC      15
#define GY_PIN_RST       17

// ---------------------------------------------------------------------------
// Laser SPI bus  (SPI3 / VSPI) — one DAC80004
// ---------------------------------------------------------------------------
#define LASER_SPI_HOST   SPI3_HOST
#define LASER_PIN_MOSI   23
#define LASER_PIN_CLK    18
#define LASER_CLK_HZ     40000000

// Laser color  (DAC80004, U$3)
#define LASER_PIN_SYNC   26
#define LASER_PIN_LDAC   27
#define LASER_PIN_CLR    25

#define LASER_CH_RED     DACX0004_ADD_A
#define LASER_CH_GREEN   DACX0004_ADD_B
#define LASER_CH_BLUE    DACX0004_ADD_C

// ---------------------------------------------------------------------------
// Demo parameters
// ---------------------------------------------------------------------------
#define GALVO_CENTER      0x8000    // raw DAC8871 mid-scale (zero deflection),
                                    // used only to prime the devices at startup

// Points are ILDA-signed (center 0, +/-32767 full scale); the engine maps them
// to galvo DAC codes. Stay within +/-20% of full peak-to-peak travel to keep the
// galvos in their linear region: 0.20 * 65535 ~= 13107 (same LSB as the DAC code,
// only the origin shifts, so the physical deflection matches the old code space).
#define GALVO_AMPLITUDE   13107

// "Ballywhoop": a 3:2 Lissajous figure whose y-axis phase precesses slowly, so
// the figure continuously morphs.
#define LISSAJOUS_FX      3.0f
#define LISSAJOUS_FY      2.0f
#define POINTS_PER_LOOP   256
#define POINT_RATE_HZ     30000     // ILDA "30K" sample rate; 256 pts -> ~117 Hz
                                    // redraw: well above flicker fusion
#define POINT_PERIOD_S    (1.0f / POINT_RATE_HZ)

// Background animation rates in wall-clock units, so morph/color speed stay
// constant regardless of the point rate.
#define MORPH_RATE_RAD_S  1.0f      // y-phase precession (~6 s per full morph)
#define HUE_RATE_DEG_S    50.0f     // color cycling (~7.2 s per full wheel)

#define LASER_INTENSITY   0.25f     // 25% of full scale
#define COLOR_EVERY       16        // refresh laser color every N points

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------------------------------------------------------------------------
// Device handles + interface args
// ---------------------------------------------------------------------------
static dac8871_dev_t  galvo_x = {0};
static dac8871_dev_t  galvo_y = {0};
static dacx0004_dev_t laser   = {0};

// ISR-safe SPI writers used by the laser engine on the hot path. Initialized
// after the devices are primed (which routes each CS to its peripheral line).
static isr_spi_dev_t  isr_gx    = {0};
static isr_spi_dev_t  isr_gy    = {0};
static isr_spi_dev_t  isr_laser = {0};

static dac8871_idf6_arg_t galvo_x_arg = {
    .host       = GALVO_SPI_HOST,
    .clk_freq   = GALVO_CLK_HZ,
    .spi_q_size = 1,
    .cs_pin     = GX_PIN_CS,
    .ldac_pin   = GX_PIN_LDAC,
    .rst_pin    = GX_PIN_RST,
};

static dac8871_idf6_arg_t galvo_y_arg = {
    .host       = GALVO_SPI_HOST,
    .clk_freq   = GALVO_CLK_HZ,
    .spi_q_size = 1,
    .cs_pin     = GY_PIN_CS,
    .ldac_pin   = GY_PIN_LDAC,
    .rst_pin    = GY_PIN_RST,
};

static dacx0004_idf6_arg_t laser_arg = {
    .host       = LASER_SPI_HOST,
    .clk_freq   = LASER_CLK_HZ,
    .spi_q_size = 1,
    .sync_pin   = LASER_PIN_SYNC,
    .ldac_pin   = LASER_PIN_LDAC,
    .clr_pin    = LASER_PIN_CLR,
    .miso_pin   = -1,
};

// ---------------------------------------------------------------------------
// HSV -> 8-bit RGB (ILDA color depth), with saturation and value fixed at full
// (S = V = 1). h is in degrees [0, 360).
// ---------------------------------------------------------------------------
static void hsv_to_rgb8(float h, uint8_t *r, uint8_t *g, uint8_t *b)
{
    const float c  = LASER_INTENSITY;   // chroma = V * S  (V = LASER_INTENSITY, S = 1)
    const float hp = h / 60.0f;
    const float x  = c * (1.0f - fabsf(fmodf(hp, 2.0f) - 1.0f));
    float rf = 0.0f, gf = 0.0f, bf = 0.0f;

    if      (hp < 1.0f) { rf = c; gf = x; }
    else if (hp < 2.0f) { rf = x; gf = c; }
    else if (hp < 3.0f) { gf = c; bf = x; }
    else if (hp < 4.0f) { gf = x; bf = c; }
    else if (hp < 5.0f) { rf = x; bf = c; }
    else                { rf = c; bf = x; }

    *r = (uint8_t)(rf * 0xFF);
    *g = (uint8_t)(gf * 0xFF);
    *b = (uint8_t)(bf * 0xFF);
}

// ---------------------------------------------------------------------------
// Renderer: trace a morphing Lissajous ("ballywhoop") while slowly cycling the
// hue. Each point carries its position and color; the engine consumes one point
// per tick at POINT_RATE_HZ. Producer-side push retries on a full ring.
// ---------------------------------------------------------------------------
static void render_task(void *arg)
{
    (void)arg;

    const float two_pi = (float)(2.0 * M_PI);
    const float t_step = two_pi / POINTS_PER_LOOP;
    float t = 0.0f, phase = 0.0f, hue = 0.0f;
    int color_div = 0;
    uint8_t r = 0, g = 0, b = 0;

    for (;;) {
        // Recompute the slowly-drifting color only every COLOR_EVERY points to
        // keep the float work down at the high point rate; the value still
        // travels with every point.
        if (color_div == 0) hsv_to_rgb8(hue, &r, &g, &b);
        if (++color_div >= COLOR_EVERY) color_div = 0;

        laser_point_t p = {
            .x      = (int16_t)(GALVO_AMPLITUDE * sinf(LISSAJOUS_FX * t)),
            .y      = (int16_t)(GALVO_AMPLITUDE * sinf(LISSAJOUS_FY * t + phase)),
            .status = 0,
            .r      = r,
            .g      = g,
            .b      = b,
        };
        while (!laser_engine_point(&p)) { vTaskDelay(1); }

        t += t_step;
        if (t >= two_pi) t -= two_pi;
        phase += MORPH_RATE_RAD_S * POINT_PERIOD_S;
        if (phase >= two_pi) phase -= two_pi;
        hue += HUE_RATE_DEG_S * POINT_PERIOD_S;
        if (hue >= 360.0f) hue -= 360.0f;
    }
}

void app_main(void)
{
    #if ENABLE_WIFI
    // -- Networking: bring up the SoftAP for streaming. Owns NVS / netif /
    //    event loop, so start it first. --------------------------------------
    if (!wifi_ap_start()) {
        ESP_LOGE(TAG, "wifi_ap_start failed");  // non-fatal: keep tracing
    }
    #endif

    // -- SPI buses (one per DAC group) --------------------------------------
    spi_bus_config_t galvo_bus = {
        .mosi_io_num     = GALVO_PIN_MOSI,
        .miso_io_num     = -1,
        .sclk_io_num     = GALVO_PIN_CLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 0,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(GALVO_SPI_HOST, &galvo_bus, SPI_DMA_CH_AUTO));

    spi_bus_config_t laser_bus = {
        .mosi_io_num     = LASER_PIN_MOSI,
        .miso_io_num     = -1,
        .sclk_io_num     = LASER_PIN_CLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 0,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LASER_SPI_HOST, &laser_bus, SPI_DMA_CH_AUTO));

    // -- DAC devices --------------------------------------------------------
    if (dac8871_init_dev(&galvo_x, &dac8871_if_idf6, &galvo_x_arg) != DAC8871_STAT_OK) {
        ESP_LOGE(TAG, "galvo_x init failed"); return;
    }
    if (dac8871_init_dev(&galvo_y, &dac8871_if_idf6, &galvo_y_arg) != DAC8871_STAT_OK) {
        ESP_LOGE(TAG, "galvo_y init failed"); return;
    }
    if (dacx0004_init_dev(&laser, DAC80004, &dacx0004_if_idf6, &laser_arg) != DACX0004_STAT_OK) {
        ESP_LOGE(TAG, "laser init failed"); return;
    }

    // -- Prime each device once through the IDF driver. The first transmit adds
    //    the device to its bus (spi_bus_add_device), which routes the CS pin to
    //    a peripheral CS line, assigning them in add order: galvo_x -> CS0,
    //    galvo_y -> CS1 on the galvo bus; laser -> CS0 on its own bus. After
    //    this the engine drives the buses through isr_spi only. -----------------
    dac8871_set_code(&galvo_x, GALVO_CENTER);
    dac8871_set_code(&galvo_y, GALVO_CENTER);
    dacx0004_write_update_channel(&laser, LASER_CH_RED, 0);

    // -- Build the ISR-safe writers now that CS routing + clock are set up. The
    //    cs_id / mode / clock here must match the priming config above:
    //    galvos = mode 0, laser = mode 2 (see the idf6 wrappers). ---------------
    ESP_ERROR_CHECK(isr_spi_dev_init(&isr_gx,    GALVO_SPI_HOST, 0, 0, GALVO_CLK_HZ));
    ESP_ERROR_CHECK(isr_spi_dev_init(&isr_gy,    GALVO_SPI_HOST, 1, 0, GALVO_CLK_HZ));
    ESP_ERROR_CHECK(isr_spi_dev_init(&isr_laser, LASER_SPI_HOST, 0, 2, LASER_CLK_HZ));

    // -- Laser engine: the consumer is the gptimer ISR (no output task). It
    //    centers the galvo and blanks the laser at start. ----------------------
    laser_engine_cfg_t cfg = {
        .galvo_x       = &isr_gx,
        .galvo_y       = &isr_gy,
        .laser         = &isr_laser,
        .ch_r          = LASER_CH_RED,
        .ch_g          = LASER_CH_GREEN,
        .ch_b          = LASER_CH_BLUE,
        .point_rate_hz = POINT_RATE_HZ,
    };
    if (!laser_engine_init(&cfg)) {
        ESP_LOGE(TAG, "laser_engine_init failed"); return;
    }
    laser_engine_start();
    ESP_LOGI(TAG, "laser engine started; tracing ballywhoop at 25%% intensity");

    // Renderer on core 1; the gptimer consumer ISR runs on core 0 (where it was
    // installed), so the two don't contend for the same CPU.
    xTaskCreatePinnedToCore(render_task, "render", 4096, NULL, 5, NULL, 1);
}
