// shadowgraph demo: trace a morphing Lissajous "ballywhoop" on the galvos while
// slowly cycling the laser color through the HSV hue wheel at 25% intensity.
// Exercises the full laser_engine path end to end: queue, command codec,
// timer-paced consumer, and galvo + laser DAC writes.
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#include "dac8871_idf6.h"
#include "dacx0004_idf6.h"
#include "laser_engine.h"

static const char *TAG = "shadowgraph";

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
#define GALVO_CENTER      0x8000    // mid-scale = zero deflection

// Stay within +/-20% of full-scale travel to keep the galvos in their linear
// region: 0.20 * 0xFFFF ~= 13107.
#define GALVO_AMPLITUDE   13107

// "Ballywhoop": a 3:2 Lissajous figure whose y-axis phase precesses slowly, so
// the figure continuously morphs.
#define LISSAJOUS_FX      3.0f
#define LISSAJOUS_FY      2.0f
#define POINTS_PER_LOOP   256
#define POINT_DWELL_US    2000      // 2 ms/point -> ~0.5 s per base loop
#define PHASE_STEP        0.002f    // y-phase precession per point (radians)

#define HUE_STEP          0.1f      // degrees/point -> ~7.2 s per color cycle
#define LASER_INTENSITY   0.25f     // 25% of full scale

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------------------------------------------------------------------------
// Device handles + interface args
// ---------------------------------------------------------------------------
static dac8871_dev_t  galvo_x = {0};
static dac8871_dev_t  galvo_y = {0};
static dacx0004_dev_t laser   = {0};

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
// HSV -> 16-bit RGB, with saturation and value fixed at full (S = V = 1).
// h is in degrees [0, 360).
// ---------------------------------------------------------------------------
static void hsv_to_rgb16(float h, uint16_t *r, uint16_t *g, uint16_t *b)
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

    *r = (uint16_t)(rf * 0xFFFF);
    *g = (uint16_t)(gf * 0xFFFF);
    *b = (uint16_t)(bf * 0xFFFF);
}

// ---------------------------------------------------------------------------
// Renderer: trace a morphing Lissajous ("ballywhoop") while slowly cycling the
// hue. Each point is goto + color + dwell; producer-side calls retry on a full
// queue (the consumer drains at the dwell rate).
// ---------------------------------------------------------------------------
static void render_task(void *arg)
{
    (void)arg;

    const float two_pi = (float)(2.0 * M_PI);
    const float t_step = two_pi / POINTS_PER_LOOP;
    float t = 0.0f, phase = 0.0f, hue = 0.0f;

    for (;;) {
        int32_t x = GALVO_CENTER + (int32_t)(GALVO_AMPLITUDE * sinf(LISSAJOUS_FX * t));
        int32_t y = GALVO_CENTER + (int32_t)(GALVO_AMPLITUDE * sinf(LISSAJOUS_FY * t + phase));

        uint16_t r, g, b;
        hsv_to_rgb16(hue, &r, &g, &b);

        while (!laser_engine_goto((uint16_t)x, (uint16_t)y)) { vTaskDelay(1); }
        while (!laser_engine_laser(r, g, b))                 { vTaskDelay(1); }
        while (!laser_engine_dwell(POINT_DWELL_US))          { vTaskDelay(1); }

        t += t_step;
        if (t >= two_pi) t -= two_pi;
        phase += PHASE_STEP;
        if (phase >= two_pi) phase -= two_pi;
        hue += HUE_STEP;
        if (hue >= 360.0f) hue -= 360.0f;
    }
}

void app_main(void)
{
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

    // -- Laser engine: consumer pinned to core 1; it primes the DACs (forcing
    //    both galvos onto the shared bus) and blanks the laser at start. ------
    laser_engine_cfg_t cfg = {
        .galvo_x   = &galvo_x,
        .galvo_y   = &galvo_y,
        .laser     = &laser,
        .ch_r      = LASER_CH_RED,
        .ch_g      = LASER_CH_GREEN,
        .ch_b      = LASER_CH_BLUE,
        .retry_us  = 1000,
        .task_core = 1,
        .task_prio = configMAX_PRIORITIES - 1,
    };
    if (!laser_engine_init(&cfg)) {
        ESP_LOGE(TAG, "laser_engine_init failed"); return;
    }
    laser_engine_start();
    ESP_LOGI(TAG, "laser engine started; tracing ballywhoop at 25%% intensity");

    // Renderer on core 0 (the consumer owns core 1).
    xTaskCreatePinnedToCore(render_task, "render", 4096, NULL, 5, NULL, 0);
}
