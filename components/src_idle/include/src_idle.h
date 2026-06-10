#pragma once

#include "renderer.h"

// Idle/demo renderer source: traces a slowly morphing Lissajous ("ballywhoop")
// while cycling the laser hue. This is the safe default source (engine just
// keeps a bounded figure on screen when nothing is being streamed).
//
// Returns a static source vtable to hand to renderer_register().
const renderer_source_t *src_idle_get(void);
