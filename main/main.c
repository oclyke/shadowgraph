// shadowgraph: drive the galvos + laser from an ILDA-style point stream. The
// device brings up WiFi, a TCP scene receiver (tools/svg2scene + ildaplay), and
// an Art-Net control receiver (tools/artnetctl). The renderer chooses its source
// at runtime from the Art-Net control state: trace a built-in Lissajous
// ("pattern"), or loop the last scene streamed in ("stream"). It starts on the
// pattern with offline defaults until a console says otherwise; ENABLE_NET 0
// drops all networking and just traces the default pattern.
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "mdns.h"

#include "dac8871_idf6.h"
#include "dacx0004_idf6.h"
#include "isr_spi.h"
#include "artnet_control.h"
#include "laser_engine.h"
#include "point_stream.h"
#include "wifi_ap.h"

static const char *TAG = "shadowgraph";

// Networking: 1 = bring up WiFi plus the TCP scene receiver and the Art-Net
// control receiver; 0 = fully offline (no radio), tracing the built-in Lissajous
// from its defaults. With networking up the live mode is chosen at runtime over
// Art-Net (control channel 1): pattern (Lissajous) vs stream.
#define ENABLE_NET      1
#define SCENE_TCP_PORT  7777
// WiFi role: 1 = join the "ioio" network as a station (device + host share that
// LAN; stream to the DHCP IP logged at boot); 0 = host our own SoftAP.
#define WIFI_STA_MODE   0

// Art-Net control fixture: the DMX universe we listen on and the 1-based slot of
// its first channel (see components/artnet_control for the channel map).
#define ARTNET_UNIVERSE      1
#define ARTNET_BASE_CHANNEL  1

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

// Built-in Lissajous renderer (pattern mode). The figure's shape, size, color,
// and morph all come from the live Art-Net control state; only the sampling
// cadence is fixed here. Points are ILDA-signed (center 0). POINTS_PER_LOOP
// samples close the figure for integer frequency ratios; the phase morph and the
// frame rebuild happen only every FRAMES_PER_REBUILD displayed loops, keeping the
// per-point trig off the hot path (cf. the sin-LUT flash-stall lesson).
#define POINTS_PER_LOOP    256
#define POINT_PERIOD_S     (1.0f / POINT_RATE_HZ)
#define FRAMES_PER_REBUILD 4
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
// HSV -> 8-bit RGB (ILDA color depth) at full saturation. `h` is in degrees
// [0, 360); `value` is the beam brightness [0, 1] (the Art-Net intensity).
// ---------------------------------------------------------------------------
static void hsv_to_rgb8(float h, float value, uint8_t *r, uint8_t *g, uint8_t *b)
{
    h = fmodf(h, 360.0f);               // wrap: callers add spans/cycling to hue
    if (h < 0.0f) h += 360.0f;
    const float c  = value;             // chroma = V * S  (S = 1)
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

// Time-varying phases the renderer advances between rebuilds. Each wraps within
// its own period so it never loses float precision (cf. a free-running clock).
typedef struct {
    float y_phase;      // y-axis morph precession (rad)
    float blank_pos;    // leading edge of the sliding blank gap, loop fraction [0,1)
    float color_phase;  // color rotation over time (deg)
    float fx_phase;     // fx morph oscillator phase (rad)
    float fy_phase;     // fy morph oscillator phase (rad)
} render_dyn_t;

// Evaluate the Lissajous for the current control state + dynamic phases into
// s_frame. Called once per FRAMES_PER_REBUILD displayed loops, so the per-point
// trig stays off the hot path. On top of the base figure it applies three live
// effects from the Art-Net state:
//   - a sliding blank gap (blank_width/_pos): an "aliasing" window in the
//     parametric domain where the beam blanks;
//   - color as a function of t (color_t_span) that also rotates over time
//     (color_phase);
//   - fx/fy morph: the frequencies oscillate around their base, so the figure
//     evolves. A morphed figure no longer closes, so the loop seam is blanked to
//     hide the flyback.
static void build_frame(const artnet_control_state_t *st, const render_dyn_t *d)
{
    const float two_pi = (float)(2.0 * M_PI);

    // Effective frequencies, optionally morphing around the integer base.
    float fx = (float)st->fx;
    float fy = (float)st->fy;
    if (st->freq_morph_depth > 0.0f) {
        if (st->fx_morph_rate > 0.0f) fx += st->freq_morph_depth * sinf(d->fx_phase);
        if (st->fy_morph_rate > 0.0f) fy += st->freq_morph_depth * sinf(d->fy_phase);
    }
    const bool blank_seam = st->freq_morph_depth > 0.0f &&
                            (st->fx_morph_rate > 0.0f || st->fy_morph_rate > 0.0f);

    const float amp  = (float)st->amplitude;
    const float yoff = d->y_phase + st->phase_off;
    const float bw   = st->blank_width;            // 0 = no blanking
    const bool  color_per_point = st->color_t_span > 0.0f;

    // Constant color (no t-dependence) is computed once; the time rotation still
    // applies via color_phase.
    uint8_t r = 0, g = 0, b = 0;
    if (!color_per_point)
        hsv_to_rgb8(st->hue_deg + d->color_phase, st->intensity, &r, &g, &b);

    for (int i = 0; i < POINTS_PER_LOOP; i++) {
        const float u = (float)i / POINTS_PER_LOOP;   // parametric position [0,1)
        const float t = u * two_pi;

        uint8_t status = 0;
        if (bw > 0.0f) {
            float dpos = u - d->blank_pos;
            dpos -= floorf(dpos);                     // distance ahead of the gap edge, [0,1)
            if (dpos < bw) status = POINT_BLANK;
        }
        if (color_per_point) {
            hsv_to_rgb8(st->hue_deg + d->color_phase + u * st->color_t_span,
                        st->intensity, &r, &g, &b);
        }
        s_frame[i] = (laser_point_t){
            .x      = (int16_t)(amp * sinf(fx * t)),
            .y      = (int16_t)(amp * sinf(fy * t + yoff)),
            .status = status,
            .r      = r,
            .g      = g,
            .b      = b,
        };
    }
    if (blank_seam) s_frame[0].status |= POINT_BLANK;
}

// ---------------------------------------------------------------------------
// Renderer: pick pattern vs stream from the live Art-Net control state each
// loop. In pattern mode it traces the Lissajous from the current settings
// (morphing the y-phase); in stream mode it loops the latest scene received over
// TCP. Both push only from internal DRAM, keeping flash off the hot path (cf.
// the sin-LUT stall). When no scene has arrived yet the ring drains and the
// engine blanks — safe: galvo centered, beam dark.
// ---------------------------------------------------------------------------
static void render_task(void *arg)
{
    (void)arg;

    const float two_pi = (float)(2.0 * M_PI);
    render_dyn_t dyn = {0};
    int   f = 0;   // displayed-loop counter within the current rebuild cycle

    for (;;) {
        artnet_control_state_t st;
        artnet_control_get(&st);

        if (st.mode == ARTNET_MODE_STREAM) {
            f = 0;   // restart pattern bookkeeping so we rebuild on return
            const laser_point_t *pts = NULL;
            uint32_t n = 0;
            if (!point_stream_get(&pts, &n) || n == 0) {
                vTaskDelay(pdMS_TO_TICKS(50));   // idle: no scene yet
                continue;
            }
            uint32_t pushed = 0;
            while (pushed < n) {
                pushed += laser_engine_points(pts + pushed, n - pushed);
                if (pushed < n) vTaskDelay(1);   // ring full: let it drain
            }
            continue;
        }

        // Pattern mode: (re)build the figure from the live settings at the start
        // of each rebuild cycle, then bulk-push it. Rebuilding every
        // FRAMES_PER_REBUILD loops keeps the per-point trig off the hot path and
        // still picks up fader moves within a few tens of ms.
        if (f == 0) build_frame(&st, &dyn);

        uint32_t pushed = 0;
        while (pushed < POINTS_PER_LOOP) {
            pushed += laser_engine_points(s_frame + pushed, POINTS_PER_LOOP - pushed);
            if (pushed < POINTS_PER_LOOP) vTaskDelay(1);   // ring full: drain
        }

        if (++f >= FRAMES_PER_REBUILD) {
            f = 0;
            // Advance each animated phase by the wall-clock time those loops took,
            // then wrap it within its own period (no precision drift over time).
            const float dt = FRAMES_PER_REBUILD * POINTS_PER_LOOP * POINT_PERIOD_S;
            dyn.y_phase     = fmodf(dyn.y_phase + st.morph_rate * dt, two_pi);
            dyn.color_phase = fmodf(dyn.color_phase + st.color_cycle_rate * dt, 360.0f);
            dyn.fx_phase    = fmodf(dyn.fx_phase + two_pi * st.fx_morph_rate * dt, two_pi);
            dyn.fy_phase    = fmodf(dyn.fy_phase + two_pi * st.fy_morph_rate * dt, two_pi);
            dyn.blank_pos  += st.blank_slide_rate * dt;
            dyn.blank_pos  -= floorf(dyn.blank_pos);   // keep in [0,1)
        }
    }
}

#if ENABLE_NET
// Advertise the device over mDNS so hosts can use a stable name instead of the
// DHCP IP: ildaplay/artnetctl --host shadowgraph.local. Also publishes the
// Art-Net and scene-stream services for discovery-aware tools. Non-fatal: on
// failure the device still works, you just address it by IP.
#define MDNS_HOSTNAME "shadowgraph"
static void start_mdns(void)
{
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mdns_init failed: %s (address the device by IP)",
                 esp_err_to_name(err));
        return;
    }
    mdns_hostname_set(MDNS_HOSTNAME);
    mdns_instance_name_set("shadowgraph laser projector");
    mdns_service_add(NULL, "_artnet", "_udp", 6454, NULL, 0);
    mdns_service_add(NULL, "_shadowgraph", "_tcp", SCENE_TCP_PORT, NULL, 0);
    ESP_LOGI(TAG, "mDNS up: %s.local", MDNS_HOSTNAME);
}
#endif

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
    #if ENABLE_NET
    // -- Networking: bring up WiFi for streaming + Art-Net control. Owns NVS /
    //    netif / event loop, so start it first. STA joins "ioio" (IP logged on
    //    join); AP hosts our own network. ----------------------------------------
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

    // -- Control + scene stores. Seed both unconditionally so the renderer always
    //    has valid state to read; the network receivers below depend on WiFi. The
    //    renderer chooses pattern vs stream at runtime from the Art-Net control
    //    state, which holds the offline Lissajous defaults until a console (or
    //    tools/artnetctl) says otherwise. --------------------------------------
    artnet_control_state_t defaults;
    artnet_control_defaults(&defaults);
    artnet_control_init(&defaults, ARTNET_UNIVERSE, ARTNET_BASE_CHANNEL);
    point_stream_init();

#if ENABLE_NET
    // Scene streaming: triple-buffered DRAM scene store + TCP receiver.
    if (!point_stream_start(SCENE_TCP_PORT)) {
        ESP_LOGE(TAG, "point_stream_start failed");  // non-fatal: renderer idles
    }
    // Art-Net control receiver: maps DMX -> mode + Lissajous settings.
    if (!artnet_control_start()) {
        ESP_LOGE(TAG, "artnet_control_start failed");  // non-fatal: defaults hold
    }
    // mDNS: reach the device as shadowgraph.local instead of by DHCP IP.
    start_mdns();
#endif

    // Renderer on core 1; the gptimer consumer ISR runs on core 0 (where it was
    // installed), so the two don't contend for the same CPU.
    xTaskCreatePinnedToCore(render_task, "render", 4096, NULL, 5, NULL, 1);

    // Diagnostics: report underruns once a second at low priority.
    xTaskCreate(underrun_log_task, "underrun", 2048, NULL, 1, NULL);
}
