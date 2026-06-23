// shadowgraph: drive the galvos + laser from an ILDA-style point stream. By
// default the device brings up its SoftAP and a TCP scene receiver, then loops
// whatever scene the host last streamed (see tools/svg2scene --stream). A
// compile-time fallback (RENDER_DEMO) traces a built-in Lissajous instead.
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#include "dac8871_idf6.h"
#include "dacx0004_idf6.h"
#include "isr_spi.h"
#include "laser_engine.h"
#include "point_stream.h"
#include "wifi_ap.h"

static const char *TAG = "shadowgraph";

// Scene source: 0 = stream scenes over TCP (needs WiFi); 1 = built-in Lissajous
// demo (offline fallback, no networking).
#define RENDER_DEMO     0
#define SCENE_TCP_PORT  7777
#define ENABLE_WIFI     (!RENDER_DEMO)
// WiFi role: 1 = join the "ioio" network as a station (device + host share that
// LAN; stream to the DHCP IP logged at boot); 0 = host our own SoftAP.
#define WIFI_STA_MODE   1

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
// Engine parameters
// ---------------------------------------------------------------------------
#define GALVO_CENTER      0x8000    // raw DAC8871 mid-scale (zero deflection),
                                    // used only to prime the devices at startup
#define POINT_RATE_HZ     30000     // ILDA "30K" sample rate (engine cadence)

#if RENDER_DEMO
// Lissajous "ballywhoop" demo parameters (offline fallback). Points are ILDA-
// signed (center 0); stay within ~±20% of full travel to keep the galvos linear
// (0.20 * 65535 ~= 13107).
#define GALVO_AMPLITUDE   13107
#define LISSAJOUS_FX      3.0f
#define LISSAJOUS_FY      2.0f
#define POINTS_PER_LOOP   256
#define POINT_PERIOD_S    (1.0f / POINT_RATE_HZ)
#define MORPH_RATE_RAD_S  1.0f      // y-phase precession (~6 s per full morph)
#define HUE_RATE_DEG_S    50.0f     // color cycling (~7.2 s per full wheel)
#define LASER_INTENSITY   0.25f     // 25% of full scale
#define FRAMES_PER_REBUILD 4        // rebuild the morph every N loops (~30 Hz)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#endif // RENDER_DEMO

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

#if RENDER_DEMO
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

// One precomputed loop of the figure, in internal DRAM. The hot path bulk-copies
// this into the ring with no trig and no flash access; all sinf/HSV work happens
// off the critical path in build_frame at the (much lower) rebuild cadence.
static laser_point_t s_frame[POINTS_PER_LOOP];

// Evaluate the morphing Lissajous for the current phase/hue into s_frame. Called
// once per FRAMES_PER_REBUILD displayed loops, so its sinf cost is ~30 Hz, not
// per output point. Color is constant within a frame (the hue drift per rebuild
// is small), which matches the old per-16-point color quantization.
static void build_frame(float phase, float hue)
{
    const float two_pi = (float)(2.0 * M_PI);
    const float t_step = two_pi / POINTS_PER_LOOP;

    uint8_t r, g, b;
    hsv_to_rgb8(hue, &r, &g, &b);

    for (int i = 0; i < POINTS_PER_LOOP; i++) {
        const float t = i * t_step;
        s_frame[i] = (laser_point_t){
            .x      = (int16_t)(GALVO_AMPLITUDE * sinf(LISSAJOUS_FX * t)),
            .y      = (int16_t)(GALVO_AMPLITUDE * sinf(LISSAJOUS_FY * t + phase)),
            .status = 0,
            .r      = r,
            .g      = g,
            .b      = b,
        };
    }
}

// ---------------------------------------------------------------------------
// Renderer: trace a morphing Lissajous ("ballywhoop") while slowly cycling the
// hue. The figure is precomputed into s_frame and bulk-pushed repeatedly; the
// morph/hue are advanced and the frame rebuilt only every FRAMES_PER_REBUILD
// displayed loops, decoupling the cheap display path from the trig work.
// ---------------------------------------------------------------------------
static void render_task(void *arg)
{
    (void)arg;

    const float two_pi = (float)(2.0 * M_PI);
    float phase = 0.0f, hue = 0.0f;

    build_frame(phase, hue);

    for (;;) {
        // Display the current frame FRAMES_PER_REBUILD times. Each push is a bulk
        // ring write of a DRAM buffer; retry only the tail that didn't fit.
        for (int f = 0; f < FRAMES_PER_REBUILD; f++) {
            uint32_t pushed = 0;
            while (pushed < POINTS_PER_LOOP) {
                pushed += laser_engine_points(s_frame + pushed, POINTS_PER_LOOP - pushed);
                if (pushed < POINTS_PER_LOOP) vTaskDelay(1);   // ring full: drain
            }
        }

        // Advance the animation by the wall-clock time those loops took, then
        // re-evaluate the figure for the new phase/hue.
        const float dt = FRAMES_PER_REBUILD * POINTS_PER_LOOP * POINT_PERIOD_S;
        phase += MORPH_RATE_RAD_S * dt;
        if (phase >= two_pi) phase -= two_pi;
        hue += HUE_RATE_DEG_S * dt;
        if (hue >= 360.0f) hue -= 360.0f;
        build_frame(phase, hue);
    }
}

#else // !RENDER_DEMO

// ---------------------------------------------------------------------------
// Renderer: loop the active streamed scene. point_stream_get hands back the
// latest published scene (held in internal DRAM); we bulk-push it into the ring
// on repeat. Pushing only from the DRAM scene buffer keeps flash off the hot
// path (cf. the sin-LUT stall). Until the first scene arrives the ring drains
// and the engine blanks — safe: galvo centered, beam dark. A new scene swaps in
// seamlessly between whole loops.
// ---------------------------------------------------------------------------
static void render_task(void *arg)
{
    (void)arg;
    for (;;) {
        const laser_point_t *pts = NULL;
        uint32_t n = 0;
        if (!point_stream_get(&pts, &n) || n == 0) {
            vTaskDelay(pdMS_TO_TICKS(50));   // idle: no scene yet
            continue;
        }
        // Display one full loop of the scene; retry only the tail that didn't fit.
        uint32_t pushed = 0;
        while (pushed < n) {
            pushed += laser_engine_points(pts + pushed, n - pushed);
            if (pushed < n) vTaskDelay(1);   // ring full: let it drain
        }
    }
}

#endif // RENDER_DEMO

// Low-priority watchdog on producer health: log the engine's underrun count
// once a second. A rising delta means render_task can't keep the ring full at
// POINT_RATE_HZ and the beam is being blanked on empty ticks (visible flicker).
static void underrun_log_task(void *arg)
{
    (void)arg;
    uint32_t last = laser_engine_underruns();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uint32_t now = laser_engine_underruns();
        ESP_LOGI(TAG, "underruns: %lu (+%lu/s)",
                 (unsigned long)now, (unsigned long)(now - last));
        last = now;
    }
}

void app_main(void)
{
    #if ENABLE_WIFI
    // -- Networking: bring up WiFi for streaming. Owns NVS / netif / event loop,
    //    so start it first. STA joins "ioio" (IP logged on join); AP hosts our
    //    own network. ----------------------------------------------------------
    #if WIFI_STA_MODE
    if (!wifi_sta_start()) {
        ESP_LOGE(TAG, "wifi_sta_start failed");  // non-fatal: keep tracing
    }
    #else
    if (!wifi_ap_start()) {
        ESP_LOGE(TAG, "wifi_ap_start failed");  // non-fatal: keep tracing
    }
    #endif
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
    ESP_LOGI(TAG, "laser engine started");

#if !RENDER_DEMO
    // -- Scene streaming: triple-buffered DRAM scene store + TCP receiver. The
    //    renderer loops the latest published scene. ----------------------------
    point_stream_init();
    if (!point_stream_start(SCENE_TCP_PORT)) {
        ESP_LOGE(TAG, "point_stream_start failed");  // non-fatal: renderer idles
    }
#endif

    // Renderer on core 1; the gptimer consumer ISR runs on core 0 (where it was
    // installed), so the two don't contend for the same CPU.
    xTaskCreatePinnedToCore(render_task, "render", 4096, NULL, 5, NULL, 1);

    // Diagnostics: report underruns once a second at low priority.
    xTaskCreate(underrun_log_task, "underrun", 2048, NULL, 1, NULL);
}
