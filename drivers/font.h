//
//  PicoCalc LCD display font
//  Copyright 2025-2026 Blair Leduc. See LICENSE for details.
//

#pragma once

#include <pico/stdlib.h>

#define GLYPH_HEIGHT 10 // Height of each glyph in pixels

typedef struct {
    uint8_t width;
    uint8_t glyphs[];
} font_t;

extern const font_t font_8x10; // 8x10 pixel font
extern const font_t font_5x10; // 5x10 pixel font
