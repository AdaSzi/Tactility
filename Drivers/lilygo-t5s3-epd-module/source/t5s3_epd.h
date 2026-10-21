// SPDX-License-Identifier: LGPL-3.0-or-later
//
// Waveform updates of the ED047TC1 panel of the LILYGO T5 4.7 Inch E-Paper S3.
// The update logic follows the epdiy project (https://github.com/vroland/epdiy).
// The calls must come from one task, there is no locking.

#pragma once

#include "t5s3_epd_rows.h"

#include <cstdint>

// Columns [x0, x1) of the rows [y0, y1)
struct T5s3EpdRect {
    int32_t x0, y0, x1, y1;
};

// Allocates the frame buffers and sets up the pins and peripherals. The panel stays unpowered.
bool t5s3_epd_init(uint32_t pixel_clock_hz);

void t5s3_epd_deinit();

// Frame buffer to draw in, 4 bits per pixel. The panel shows it after an update.
uint8_t* t5s3_epd_framebuffer();

// Black and white version of the frame buffer with levels 0 and 15 only, shown by fast updates
uint8_t* t5s3_epd_dither_buffer();

// Shows the pixels of the rows [y_start, y_end) whose level changed with their black and white dither, in a few frames.
// changed gets the area of the changed pixels, which still needs a refresh for its gray levels. Needs the panel to be powered.
bool t5s3_epd_show_changes(int32_t y_start, int32_t y_end, T5s3EpdRect* changed);

// Draws the pixels of the areas that do not show their level yet with the 16 level waveform, after driving them to white.
// Needs the panel to be powered.
bool t5s3_epd_refresh(const T5s3EpdRect* rects, int count);

void t5s3_epd_fill_white();

// Flashes the whole panel to white. Needs the panel to be powered.
bool t5s3_epd_clear();
