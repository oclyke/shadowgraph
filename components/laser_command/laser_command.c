#include "laser_command.h"

// laser_command_pop (and its helper laser_command_size) run in the laser_engine
// timer ISR, which is IRAM-safe; keep them out of flash. push_* are producer-
// side (renderer task) and stay in flash. No-op on the host test build.
#if defined(ESP_PLATFORM)
#include "esp_attr.h"
#else
#define IRAM_ATTR
#endif

#define REC_GOTO   (1 + 2 + 2)       // type + x + y
#define REC_LASER  (1 + 2 + 2 + 2)   // type + r + g + b
#define REC_DWELL  (1 + 4)           // type + dt
#define REC_MAX    REC_LASER

uint32_t IRAM_ATTR laser_command_size(laser_command_type_t type) {
    switch (type) {
        case LASER_CMD_GOTO:  return REC_GOTO;
        case LASER_CMD_LASER: return REC_LASER;
        case LASER_CMD_DWELL: return REC_DWELL;
        default:              return 0;
    }
}

static inline void put_u16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}
static inline void put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}
static inline uint16_t get_u16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline uint32_t get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

bool laser_command_push_goto(byte_queue_t *q, uint16_t x, uint16_t y) {
    uint8_t rec[REC_GOTO];
    rec[0] = LASER_CMD_GOTO;
    put_u16(rec + 1, x);
    put_u16(rec + 3, y);
    return byte_queue_write(q, rec, sizeof rec);
}

bool laser_command_push_laser(byte_queue_t *q, uint16_t r, uint16_t g, uint16_t b) {
    uint8_t rec[REC_LASER];
    rec[0] = LASER_CMD_LASER;
    put_u16(rec + 1, r);
    put_u16(rec + 3, g);
    put_u16(rec + 5, b);
    return byte_queue_write(q, rec, sizeof rec);
}

bool laser_command_push_dwell(byte_queue_t *q, uint32_t dt) {
    uint8_t rec[REC_DWELL];
    rec[0] = LASER_CMD_DWELL;
    put_u32(rec + 1, dt);
    return byte_queue_write(q, rec, sizeof rec);
}

bool IRAM_ATTR laser_command_pop(byte_queue_t *q, laser_command_t *out) {
    uint8_t type;
    if (byte_queue_peek(q, &type, 1) < 1) {
        return false;   // empty
    }
    uint32_t size = laser_command_size((laser_command_type_t)type);
    if (size == 0) {
        return false;   // unknown type — corruption guard (record left in place)
    }
    if (byte_queue_avail(q) < size) {
        return false;   // partial record (cannot happen with atomic push)
    }

    uint8_t rec[REC_MAX];
    byte_queue_read(q, rec, size);
    out->type = (laser_command_type_t)type;
    switch (out->type) {
        case LASER_CMD_GOTO:
            out->pos.x = get_u16(rec + 1);
            out->pos.y = get_u16(rec + 3);
            return true;
        case LASER_CMD_LASER:
            out->col.r = get_u16(rec + 1);
            out->col.g = get_u16(rec + 3);
            out->col.b = get_u16(rec + 5);
            return true;
        case LASER_CMD_DWELL:
            out->dwell.dt = get_u32(rec + 1);
            return true;
        default:
            return false;
    }
}
