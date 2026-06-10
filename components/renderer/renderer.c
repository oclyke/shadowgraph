#include "renderer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdatomic.h>

static const renderer_source_t *s_sources[RENDERER_MAX_SOURCES];
static _Atomic int               s_pending = -1;   // requested source id
static int                       s_active  = -1;   // currently running id
static TaskHandle_t              s_task;

// The mux task is the single producer. It applies a pending source switch only
// between pumps, so a source never has start()/stop() race with its own pump().
static void mux_task(void *arg)
{
    (void)arg;
    for (;;) {
        int pending = atomic_load(&s_pending);
        if (pending != s_active && pending >= 0 &&
            pending < RENDERER_MAX_SOURCES && s_sources[pending] != NULL) {
            if (s_active >= 0 && s_sources[s_active]->stop) {
                s_sources[s_active]->stop(s_sources[s_active]->ctx);
            }
            s_active = pending;
            if (s_sources[s_active]->start) {
                s_sources[s_active]->start(s_sources[s_active]->ctx);
            }
        }

        if (s_active >= 0) {
            // pump() is contracted to block/yield when idle; no delay here so an
            // active source can run flat out (e.g. drain a full engine queue).
            s_sources[s_active]->pump(s_sources[s_active]->ctx);
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));   // nothing selected yet
        }
    }
}

bool renderer_init(const renderer_cfg_t *cfg)
{
    uint32_t prio = (cfg && cfg->task_prio) ? cfg->task_prio : 5;
    int      core = cfg ? cfg->task_core : -1;

    BaseType_t ok;
    if (core < 0) {
        ok = xTaskCreate(mux_task, "renderer", 4096, NULL, prio, &s_task);
    } else {
        ok = xTaskCreatePinnedToCore(mux_task, "renderer", 4096, NULL,
                                     prio, &s_task, core);
    }
    return ok == pdPASS;
}

bool renderer_register(int id, const renderer_source_t *src)
{
    if (id < 0 || id >= RENDERER_MAX_SOURCES || src == NULL) {
        return false;
    }
    s_sources[id] = src;
    return true;
}

void renderer_select(int id)
{
    atomic_store(&s_pending, id);
}

int renderer_active(void)
{
    return s_active;
}
