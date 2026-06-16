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
#include "isr_spi.h"
#include "laser_engine.h"
#include "curve_interp.h"   // CURVE_DEFAULT_V_MAX_CPS (shared limit contract)

// Networking build config. Pick exactly one WiFi mode when ENABLE_WIFI is set:
// STA joins an existing AP (phone hotspot), AP stands up our own SoftAP. These
// gate the includes below, so they must be defined first.
#define ENABLE_WIFI 1
#define ENABLE_STA  1
#define ENABLE_AP   0

#if ENABLE_WIFI
#if ENABLE_STA == ENABLE_AP
#error "Select exactly one WiFi mode: set one of ENABLE_STA / ENABLE_AP to 1"
#endif
#if ENABLE_STA
#include "wifi_sta.h"
// STA: the network we join for UDP tests (a phone hotspot here).
#define WIFI_STA_SSID  "ioio"
#define WIFI_STA_PASS  "spicygreen"
#endif
#if ENABLE_AP
#include "wifi_ap.h"
// AP: the network we host. WPA2-PSK needs a >= 8 char password.
#define WIFI_AP_SSID   "shadowgraph"
#define WIFI_AP_PASS   "letslaser"
#endif
#endif // ENABLE_WIFI

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
#define POINT_DWELL_US    50        // 256 pts * 50 us -> ~78 Hz redraw: well
                                    // above flicker fusion, looks continuous
#define POINT_PERIOD_S    (POINT_DWELL_US * 1e-6f)

// Background animation rates in wall-clock units, so morph/color speed stay
// constant regardless of the point rate.
#define MORPH_RATE_RAD_S  1.0f      // y-phase precession (~6 s per full morph)
#define HUE_RATE_DEG_S    50.0f     // color cycling (~7.2 s per full wheel)

#define LASER_INTENSITY   0.25f     // 25% of full scale
#define COLOR_EVERY       16        // refresh laser color every N points

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Demo selector. The CURVE demos exercise the curve engine; see docs/CURVE_MOTION.md.
#define DEMO_LISSAJOUS     0    // legacy point-stream Lissajous (GOTO + DWELL)
#define DEMO_FIGURE_EIGHT  1    // cubic-native figure-eight (CURVE)
#define DEMO_SQUARE        2    // large square, ~90% of full range (CURVE)
#define DEMO_MODE          DEMO_SQUARE

// Figure-eight as a Gerono lemniscate: x = W*sin t, y = (H/2)*sin 2t, t in [0,2pi).
// Drawn as a chained cubic spline (one CURVE per segment), so the whole figure
// is ~EIGHT_SEGMENTS*21 bytes on the wire instead of hundreds of GOTOs.
#define EIGHT_SEGMENTS    64
#define EIGHT_W           12000.0f   // half-width  (x swing, ~18% of full field)
#define EIGHT_H           16000.0f   // height param (y swing = H/2)

// Large square (DEMO_SQUARE). Each side spans ~90% of the full DAC range
// (0..0xFFFF): half-side = 0.45 * 0xFFFF, so corners sit at center +/- this and the
// figure runs 3278..62258. This deliberately drives the galvos well past the
// conservative +/-20% linear region the Lissajous demo uses — it is a full-range
// exercise of the engine and the corner accel/decel limiting.
#define SQUARE_HALF       29490

// Networking: WiFi station and access point configuration
// ---------------------------------------------------------------------------
#if ENABLE_WIFI
#if ENABLE_STA
#pragma message("WiFi STA mode enabled")
#elif ENABLE_AP
#pragma message("WiFi AP mode enabled")
#else
#error "No WiFi mode enabled"
#endif
#endif // ENABLE_WIFI

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

#if DEMO_MODE == DEMO_LISSAJOUS
// ---------------------------------------------------------------------------
// Renderer: trace a morphing Lissajous ("ballywhoop") while slowly cycling the
// hue. Each point is goto + color + dwell; producer-side calls retry on a full
// queue (the consumer drains at the dwell rate).
// ---------------------------------------------------------------------------
static void render_lissajous_task(void *arg)
{
    (void)arg;

    const float two_pi = (float)(2.0 * M_PI);
    const float t_step = two_pi / POINTS_PER_LOOP;
    float t = 0.0f, phase = 0.0f, hue = 0.0f;
    int color_div = 0;
    uint16_t r = 0, g = 0, b = 0;

    for (;;) {
        int32_t x = GALVO_CENTER + (int32_t)(GALVO_AMPLITUDE * sinf(LISSAJOUS_FX * t));
        int32_t y = GALVO_CENTER + (int32_t)(GALVO_AMPLITUDE * sinf(LISSAJOUS_FY * t + phase));

        while (!laser_engine_goto((uint16_t)x, (uint16_t)y)) { vTaskDelay(1); }

        // Color drifts slowly, so refresh it only every COLOR_EVERY points to
        // keep per-point SPI traffic low at the high point rate.
        if (color_div == 0) {
            hsv_to_rgb16(hue, &r, &g, &b);
            while (!laser_engine_laser(r, g, b)) { vTaskDelay(1); }
        }
        if (++color_div >= COLOR_EVERY) color_div = 0;

        while (!laser_engine_dwell(POINT_DWELL_US)) { vTaskDelay(1); }

        t += t_step;
        if (t >= two_pi) t -= two_pi;
        phase += MORPH_RATE_RAD_S * POINT_PERIOD_S;
        if (phase >= two_pi) phase -= two_pi;
        hue += HUE_RATE_DEG_S * POINT_PERIOD_S;
        if (hue >= 360.0f) hue -= 360.0f;
    }
}
#endif // DEMO_MODE == DEMO_LISSAJOUS

#if DEMO_MODE == DEMO_FIGURE_EIGHT
// ---------------------------------------------------------------------------
// Figure-eight position and analytic derivative (Gerono lemniscate).
// ---------------------------------------------------------------------------
static void eight_point(float t, float *x, float *y)
{
    *x = GALVO_CENTER + EIGHT_W * sinf(t);
    *y = GALVO_CENTER + 0.5f * EIGHT_H * sinf(2.0f * t);
}
static void eight_deriv(float t, float *dx, float *dy)
{
    *dx = EIGHT_W * cosf(t);
    *dy = EIGHT_H * cosf(2.0f * t);   // d/dt[(H/2) sin 2t] = H cos 2t
}

// ---------------------------------------------------------------------------
// Renderer: trace a cubic-native figure-eight via LASER_CMD_CURVE, cycling hue.
// Each segment is a Hermite->Bezier cubic built from the lemniscate's analytic
// tangents (control point = anchor +/- tangent*dt/3). We hand the firmware
// v_in=v_out=v_max and let its friction-circle interpolator pick the actual
// speed (it slows wherever a lobe is tight). A host-side planner (Phase 2) will
// compute true junction velocities; here the firmware does all the physics.
// ---------------------------------------------------------------------------
static void render_eight_task(void *arg)
{
    (void)arg;
    const float two_pi = (float)(2.0 * M_PI);
    const float dt_par = two_pi / EIGHT_SEGMENTS;     // parameter step per segment
    // laser_engine_curve wants WIRE units (counts/tick * 256), not counts/s.
    const uint32_t V   = (uint32_t)curve_cps_to_wire(CURVE_DEFAULT_V_MAX_CPS,
                                                     CURVE_DEFAULT_DT_TICK_US);
    float hue = 0.0f;
    uint16_t r = 0, g = 0, b = 0;

    // Jump (blanked) to the start of the figure, then enable the beam. P0 of the
    // first CURVE is implicit: this position.
    float sx, sy;
    eight_point(0.0f, &sx, &sy);
    while (!laser_engine_laser(0, 0, 0))                          { vTaskDelay(1); }
    while (!laser_engine_goto((uint16_t)sx, (uint16_t)sy))        { vTaskDelay(1); }

    for (;;) {
        hsv_to_rgb16(hue, &r, &g, &b);
        while (!laser_engine_laser(r, g, b))                     { vTaskDelay(1); }

        for (int i = 0; i < EIGHT_SEGMENTS; i++) {
            float t0 = i * dt_par, t1 = (i + 1) * dt_par;
            float p0x, p0y, p3x, p3y, d0x, d0y, d1x, d1y;
            eight_point(t0, &p0x, &p0y);
            eight_point(t1, &p3x, &p3y);
            eight_deriv(t0, &d0x, &d0y);
            eight_deriv(t1, &d1x, &d1y);
            // Hermite -> Bezier: tangents scaled by dt_par, control arm = T/3.
            uint16_t c1x = (uint16_t)(p0x + d0x * dt_par / 3.0f);
            uint16_t c1y = (uint16_t)(p0y + d0y * dt_par / 3.0f);
            uint16_t c2x = (uint16_t)(p3x - d1x * dt_par / 3.0f);
            uint16_t c2y = (uint16_t)(p3y - d1y * dt_par / 3.0f);
            while (!laser_engine_curve(c1x, c1y, c2x, c2y,
                                       (uint16_t)p3x, (uint16_t)p3y, V, V)) {
                vTaskDelay(1);
            }
        }

        hue += 2.0f;
        if (hue >= 360.0f) hue -= 360.0f;
    }
}
#endif // DEMO_MODE == DEMO_FIGURE_EIGHT

#if DEMO_MODE == DEMO_SQUARE
// ---------------------------------------------------------------------------
// Renderer: trace a large axis-aligned square via LASER_CMD_CURVE, cycling hue.
// Each side is a straight (degenerate cubic) CURVE with v_in = v_out = 0, so the
// interpolator accelerates off each corner and brakes back to rest into the next
// one — the physically correct way to take a galvo around a 90 deg corner (the
// direction reverses, so the corner must be a near-stop). Spans ~90% of full scale.
// ---------------------------------------------------------------------------
static void render_square_task(void *arg)
{
    (void)arg;
    const int C = GALVO_CENTER, h = SQUARE_HALF;
    const int corner[4][2] = {
        { C - h, C - h }, { C + h, C - h }, { C + h, C + h }, { C - h, C + h },
    };
    float hue = 0.0f;
    uint16_t r = 0, g = 0, b = 0;

    // Jump (blanked) to the first corner; P0 of the first CURVE is implicit.
    while (!laser_engine_laser(0, 0, 0)) { vTaskDelay(1); }
    while (!laser_engine_goto((uint16_t)corner[0][0], (uint16_t)corner[0][1])) {
        vTaskDelay(1);
    }

    for (;;) {
        hsv_to_rgb16(hue, &r, &g, &b);
        while (!laser_engine_laser(r, g, b)) { vTaskDelay(1); }

        for (int i = 0; i < 4; i++) {
            int x0 = corner[i][0],           y0 = corner[i][1];
            int x3 = corner[(i + 1) % 4][0], y3 = corner[(i + 1) % 4][1];
            // Straight edge as a cubic with evenly-spaced control points (constant
            // |B'|). v_in = v_out = 0 -> a clean accel/cruise/decel down each side.
            uint16_t c1x = (uint16_t)(x0 + (x3 - x0) / 3);
            uint16_t c1y = (uint16_t)(y0 + (y3 - y0) / 3);
            uint16_t c2x = (uint16_t)(x0 + 2 * (x3 - x0) / 3);
            uint16_t c2y = (uint16_t)(y0 + 2 * (y3 - y0) / 3);
            while (!laser_engine_curve(c1x, c1y, c2x, c2y,
                                       (uint16_t)x3, (uint16_t)y3, 0, 0)) {
                vTaskDelay(1);
            }
        }

        // hue += 2.0f;
        // if (hue >= 360.0f) hue -= 360.0f;
    }
}
#endif // DEMO_MODE == DEMO_SQUARE

void app_main(void)
{
    #if ENABLE_WIFI
    // -- Networking: bring up WiFi for streaming. Owns NVS / netif / event
    //    loop, so start it first. ---------------------------------------------
    #if ENABLE_STA
    if (!wifi_sta_start(WIFI_STA_SSID, WIFI_STA_PASS)) {
        ESP_LOGE(TAG, "wifi_sta_start failed");  // non-fatal: keep tracing
    }
    #elif ENABLE_AP
    if (!wifi_ap_start(WIFI_AP_SSID, WIFI_AP_PASS)) {
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
        .galvo_x  = &isr_gx,
        .galvo_y  = &isr_gy,
        .laser    = &isr_laser,
        .ch_r     = LASER_CH_RED,
        .ch_g     = LASER_CH_GREEN,
        .ch_b     = LASER_CH_BLUE,
        .retry_us = 1000,
    };
    if (!laser_engine_init(&cfg)) {
        ESP_LOGE(TAG, "laser_engine_init failed"); return;
    }
    laser_engine_start();

    // Renderer on core 1; the gptimer consumer ISR runs on core 0 (where it was
    // installed), so the two don't contend for the same CPU.
#if   DEMO_MODE == DEMO_FIGURE_EIGHT
    ESP_LOGI(TAG, "laser engine started; tracing cubic figure-eight (CURVE)");
    xTaskCreatePinnedToCore(render_eight_task, "render", 4096, NULL, 5, NULL, 1);
#elif DEMO_MODE == DEMO_SQUARE
    ESP_LOGI(TAG, "laser engine started; tracing large square (CURVE)");
    xTaskCreatePinnedToCore(render_square_task, "render", 4096, NULL, 5, NULL, 1);
#else
    ESP_LOGI(TAG, "laser engine started; tracing Lissajous figure at 25%% intensity");
    xTaskCreatePinnedToCore(render_lissajous_task, "render", 4096, NULL, 5, NULL, 1);
#endif
}
