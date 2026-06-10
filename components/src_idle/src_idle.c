// Idle source: the morphing Lissajous + hue cycle that used to live in main.c,
// repackaged as a renderer_source_t. pump() emits exactly one point (goto +
// occasional color + dwell) per call, carrying the animation state between
// calls, so the mux can switch away cleanly at point boundaries.
#include "src_idle.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "laser_engine.h"

// --- figure / animation parameters (unchanged from the original demo) -------
#define GALVO_CENTER      0x8000
#define GALVO_AMPLITUDE   13107       // ~20% of full scale: linear galvo region
#define LISSAJOUS_FX      3.0f
#define LISSAJOUS_FY      2.0f
#define POINTS_PER_LOOP   256
#define POINT_DWELL_US    50
#define POINT_PERIOD_S    (POINT_DWELL_US * 1e-6f)
#define MORPH_RATE_RAD_S  1.0f
#define HUE_RATE_DEG_S    50.0f
#define LASER_INTENSITY   0.25f
#define COLOR_EVERY       16

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    float    t, phase, hue;
    int      color_div;
    uint16_t r, g, b;
} idle_state_t;

static idle_state_t s_state;

// HSV -> 16-bit RGB with S=1, V=LASER_INTENSITY. h in degrees [0,360).
static void hsv_to_rgb16(float h, uint16_t *r, uint16_t *g, uint16_t *b)
{
    const float c  = LASER_INTENSITY;
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

static void idle_start(void *ctx)
{
    (void)ctx;
    s_state = (idle_state_t){0};
}

static void idle_pump(void *ctx)
{
    (void)ctx;
    idle_state_t *st = &s_state;
    const float two_pi = (float)(2.0 * M_PI);
    const float t_step = two_pi / POINTS_PER_LOOP;

    int32_t x = GALVO_CENTER + (int32_t)(GALVO_AMPLITUDE * sinf(LISSAJOUS_FX * st->t));
    int32_t y = GALVO_CENTER + (int32_t)(GALVO_AMPLITUDE * sinf(LISSAJOUS_FY * st->t + st->phase));

    // Blocking retries on a full queue yield the task, satisfying the pump
    // contract (the consumer drains at the dwell rate).
    while (!laser_engine_goto((uint16_t)x, (uint16_t)y)) { vTaskDelay(1); }

    if (st->color_div == 0) {
        hsv_to_rgb16(st->hue, &st->r, &st->g, &st->b);
        while (!laser_engine_laser(st->r, st->g, st->b)) { vTaskDelay(1); }
    }
    if (++st->color_div >= COLOR_EVERY) st->color_div = 0;

    while (!laser_engine_dwell(POINT_DWELL_US)) { vTaskDelay(1); }

    st->t += t_step;
    if (st->t >= two_pi) st->t -= two_pi;
    st->phase += MORPH_RATE_RAD_S * POINT_PERIOD_S;
    if (st->phase >= two_pi) st->phase -= two_pi;
    st->hue += HUE_RATE_DEG_S * POINT_PERIOD_S;
    if (st->hue >= 360.0f) st->hue -= 360.0f;
}

static const renderer_source_t s_src = {
    .name  = "idle",
    .start = idle_start,
    .stop  = NULL,
    .pump  = idle_pump,
    .ctx   = NULL,
};

const renderer_source_t *src_idle_get(void)
{
    return &s_src;
}
