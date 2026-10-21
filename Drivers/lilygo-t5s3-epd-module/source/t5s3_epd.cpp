// SPDX-License-Identifier: LGPL-3.0-or-later
//
// Waveform updates of the ED047TC1 panel of the LILYGO T5 4.7 Inch E-Paper S3.
// The update logic follows the epdiy project (https://github.com/vroland/epdiy).
//
// Each update diffs the frame buffer (front) against what the panel shows (back), sends only the rows that changed
// and only drives the columns that changed.

#include "t5s3_epd.h"

#include <tactility/log.h>
#include <tactility/memory.h>

#include <esp_attr.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstring>

#define TAG "T5s3Epd"

// A pixel keeps its drive until its row is addressed again, so the drive of a phase depends on the length of the frame.
// Frames are padded to a fixed period, which makes an update of a few rows drive as long as one of the whole panel.
// Black needs about 32 frames to get as dark as the waveform makes it and white about 8 to leave black. The fast update
// stops at a dark gray for black, the refresh completes it.
static constexpr uint32_t FAST_PHASES = 8;
static constexpr int64_t FAST_FRAME_US = 16667;
// Gate pulse of a row of the fast update in units of 0.1 us
static constexpr uint32_t FAST_HOLD_TICKS = 120;
// Duration of a frame of the whole panel, the gray levels of the waveform were measured with it
static constexpr int64_t QUALITY_FRAME_US = 40000;
// Frames that settle the pixels of a refresh before the waveform. Black pixels get the rest of their drive and white
// pixels go further toward white. Gray pixels are shaken: REFRESH_SHAKE_FRAMES frames toward black and the rest toward
// white, which erases what they showed before, also a black that was shown for a long time. The waveform then draws
// their gray levels like from a clean white.
static constexpr int REFRESH_SETTLE_FRAMES = 24;
static constexpr int REFRESH_SHAKE_FRAMES = 8;

// 1 bit per pixel
static constexpr int TOUCHED_ROW_BYTES = T5S3_EPD_WIDTH / 8;
static constexpr int TOUCHED_BYTES = TOUCHED_ROW_BYTES * T5S3_EPD_HEIGHT;
// Hold time of the frames that flash the panel to white
static constexpr uint32_t CLEAR_HOLD_TICKS = 120;
static constexpr int CLEAR_CYCLES = 3;
static constexpr int CLEAR_DARK_FRAMES = 10;
static constexpr int CLEAR_LIGHT_FRAMES = 10;
static constexpr int CLEAR_REST_FRAMES = 2;

struct Engine {
    uint8_t* front;
    uint8_t* back;
    // Black and white targets of the fast update
    uint8_t* dither;
    // Pixels that a fast update drove since their last refresh, 1 bit per pixel
    uint8_t* touched;
    // Targets of the fast update being sent, dithered where front differs from back
    uint8_t* fast;
    // Targets of the update being sent
    const uint8_t* to;
    // Actions of the phase being sent
    uint8_t table[256];
    // Areas of the refresh being sent
    const T5s3EpdRect* rects;
    int rect_count;
    int settle_frame;
    bool row_dirty[T5S3_EPD_HEIGHT];
    alignas(4) uint8_t changes[T5S3_EPD_FB_ROW_BYTES];
    alignas(4) uint8_t line_mask[T5S3_EPD_BUS_ROW_BYTES];
};

static Engine* engine = nullptr;

// Sleeps for most of the rest of the period, which also lets the idle task feed the watchdog
static void pad_frame(int64_t frame_start_us, int64_t period_us) {
    const int64_t remaining_us = period_us - (esp_timer_get_time() - frame_start_us);
    if (remaining_us > 2000) {
        vTaskDelay(pdMS_TO_TICKS((remaining_us - 1000) / 1000));
    }
    while (esp_timer_get_time() - frame_start_us < period_us) {}
}

static void IRAM_ATTR prepare_row(int y, uint8_t* out, void*) {
    t5s3_epd_prepare_row(
        out,
        engine->to + y * T5S3_EPD_FB_ROW_BYTES,
        engine->back + y * T5S3_EPD_FB_ROW_BYTES,
        engine->table,
        engine->line_mask
    );
}

bool t5s3_epd_init(uint32_t pixel_clock_hz) {
    if (engine != nullptr) {
        return true;
    }
    static const MemoryPolicy external_policy = { .required = 0, .desired = MEMORY_CAPABILITY_EXTERNAL, .alignment = 16 };
    engine = static_cast<Engine*>(memory_calloc_with_policy(1, sizeof(Engine), &external_policy));
    if (engine == nullptr) {
        LOG_E(TAG, "Out of memory");
        return false;
    }
    engine->front = static_cast<uint8_t*>(memory_alloc_with_policy(T5S3_EPD_FB_BYTES, &external_policy));
    engine->back = static_cast<uint8_t*>(memory_alloc_with_policy(T5S3_EPD_FB_BYTES, &external_policy));
    engine->dither = static_cast<uint8_t*>(memory_alloc_with_policy(T5S3_EPD_FB_BYTES, &external_policy));
    engine->fast = static_cast<uint8_t*>(memory_alloc_with_policy(T5S3_EPD_FB_BYTES, &external_policy));
    engine->touched = static_cast<uint8_t*>(memory_calloc_with_policy(1, TOUCHED_BYTES, &external_policy));
    if (engine->front == nullptr || engine->back == nullptr || engine->dither == nullptr || engine->fast == nullptr || engine->touched == nullptr) {
        LOG_E(TAG, "Out of memory");
        t5s3_epd_deinit();
        return false;
    }
    memset(engine->front, 0xFF, T5S3_EPD_FB_BYTES);
    memset(engine->back, 0xFF, T5S3_EPD_FB_BYTES);
    memset(engine->dither, 0xFF, T5S3_EPD_FB_BYTES);

    if (!t5s3_epd_board_init(pixel_clock_hz)) {
        LOG_E(TAG, "Failed to set up the pins and peripherals");
        t5s3_epd_deinit();
        return false;
    }
    return true;
}

void t5s3_epd_deinit() {
    if (engine == nullptr) {
        return;
    }
    t5s3_epd_board_deinit();
    memory_free(engine->front);
    memory_free(engine->back);
    memory_free(engine->dither);
    memory_free(engine->fast);
    memory_free(engine->touched);
    memory_free(engine);
    engine = nullptr;
}

uint8_t* t5s3_epd_framebuffer() {
    return engine->front;
}

uint8_t* t5s3_epd_dither_buffer() {
    return engine->dither;
}

void t5s3_epd_fill_white() {
    memset(engine->front, 0xFF, T5S3_EPD_FB_BYTES);
    memset(engine->dither, 0xFF, T5S3_EPD_FB_BYTES);
}

// Pixels whose level changed get their black or white target, the others keep what the panel shows.
// first and last get the first and last group of 8 pixels with a change, or -1 for a row without changes.
static void build_fast_row(int y, int* first, int* last) {
    const size_t offset = y * T5S3_EPD_FB_ROW_BYTES;
    const uint8_t* front = engine->front + offset;
    const uint8_t* back = engine->back + offset;
    const uint8_t* dither = engine->dither + offset;
    uint8_t* fast = engine->fast + offset;
    uint8_t* touched = engine->touched + y * TOUCHED_ROW_BYTES;
    *first = -1;
    *last = -1;
    for (int i = 0; i < T5S3_EPD_FB_ROW_BYTES; i++) {
        const uint8_t difference = front[i] ^ back[i];
        uint8_t changed = 0;
        changed |= (difference & 0x0F) != 0 ? 0x0F : 0x00;
        changed |= (difference & 0xF0) != 0 ? 0xF0 : 0x00;
        fast[i] = static_cast<uint8_t>((dither[i] & changed) | (back[i] & ~changed));
        // Byte i holds the pixels 2i and 2i + 1
        const int bit = (i & 3) * 2;
        touched[i / 4] |= static_cast<uint8_t>((((changed & 0x0F) != 0 ? 1 : 0) | ((changed & 0xF0) != 0 ? 2 : 0)) << bit);
        if (changed != 0) {
            *first = *first < 0 ? i / 4 : *first;
            *last = i / 4;
        }
    }
}

bool t5s3_epd_show_changes(int32_t y_start, int32_t y_end, T5s3EpdRect* changed) {
    *changed = { 0, 0, 0, 0 };
    if (y_start < 0) {
        y_start = 0;
    }
    if (y_end > T5S3_EPD_HEIGHT) {
        y_end = T5S3_EPD_HEIGHT;
    }
    const int64_t started = esp_timer_get_time();

    // Find the rows and columns that changed
    memset(engine->row_dirty, 0, sizeof(engine->row_dirty));
    memset(engine->changes, 0, sizeof(engine->changes));
    auto* changes = reinterpret_cast<uint32_t*>(engine->changes);
    int changed_rows = 0;
    int driven_rows = 0;
    int first_group = T5S3_EPD_WIDTH;
    int last_group = -1;
    for (int y = y_start; y < y_end; y++) {
        int first;
        int last;
        build_fast_row(y, &first, &last);
        if (first < 0) {
            continue;
        }
        if (changed_rows == 0) {
            changed->y0 = y;
        }
        changed->y1 = y + 1;
        first_group = first < first_group ? first : first_group;
        last_group = last > last_group ? last : last_group;
        changed_rows++;

        const auto* to_row = reinterpret_cast<const uint32_t*>(engine->fast + y * T5S3_EPD_FB_ROW_BYTES);
        const auto* from_row = reinterpret_cast<const uint32_t*>(engine->back + y * T5S3_EPD_FB_ROW_BYTES);
        uint32_t row_changes = 0;
        for (int i = 0; i < T5S3_EPD_FB_ROW_BYTES / 4; i++) {
            const uint32_t difference = to_row[i] ^ from_row[i];
            changes[i] |= difference;
            row_changes |= difference;
        }
        engine->row_dirty[y] = row_changes != 0;
        driven_rows += row_changes != 0 ? 1 : 0;
    }
    if (changed_rows == 0) {
        return true;
    }
    changed->x0 = first_group * 8;
    changed->x1 = (last_group + 1) * 8;

    // Pixels whose dither is what the panel shows already are only drawn by the refresh
    if (driven_rows > 0) {
        t5s3_epd_build_line_mask(engine->line_mask, engine->changes);
        t5s3_epd_build_fast_table(engine->table);
        engine->to = engine->fast;
        bool success = true;
        for (uint32_t phase = 0; phase < FAST_PHASES && success; phase++) {
            const int64_t frame_start_us = esp_timer_get_time();
            success = t5s3_epd_board_frame(changed->y0, changed->y1, engine->row_dirty, FAST_HOLD_TICKS, prepare_row, nullptr);
            pad_frame(frame_start_us, FAST_FRAME_US);
        }
        // A failed update leaves the back buffer alone, so the next update draws the same rows again
        if (!success) {
            LOG_E(TAG, "Update failed, a peripheral did not respond");
            return false;
        }
        for (int y = changed->y0; y < changed->y1; y++) {
            if (engine->row_dirty[y]) {
                memcpy(engine->back + y * T5S3_EPD_FB_ROW_BYTES, engine->fast + y * T5S3_EPD_FB_ROW_BYTES, T5S3_EPD_FB_ROW_BYTES);
            }
        }
    }

    LOG_D(TAG, "Fast update of %dx%d at %d,%d took %d ms", static_cast<int>(changed->x1 - changed->x0), static_cast<int>(changed->y1 - changed->y0),
        static_cast<int>(changed->x0), static_cast<int>(changed->y0), static_cast<int>((esp_timer_get_time() - started) / 1000));
    return true;
}

// Drives only the columns of the areas that cover the row
static void IRAM_ATTR prepare_refresh_row(int y, uint8_t* out, void*) {
    memset(engine->line_mask, 0, sizeof(engine->line_mask));
    for (int i = 0; i < engine->rect_count; i++) {
        const T5s3EpdRect& rect = engine->rects[i];
        if (y >= rect.y0 && y < rect.y1) {
            // A mask byte covers 4 pixels
            memset(engine->line_mask + rect.x0 / 4, 0xFF, (rect.x1 + 3) / 4 - rect.x0 / 4);
        }
    }
    prepare_row(y, out, nullptr);
}

static inline bool needs_refresh(int y, int x) {
    const uint8_t front = engine->front[y * T5S3_EPD_FB_ROW_BYTES + x / 2];
    const uint8_t back = engine->back[y * T5S3_EPD_FB_ROW_BYTES + x / 2];
    const int shift = (x & 1) * 4;
    const bool differs = ((front ^ back) >> shift & 0x0F) != 0;
    return differs || (engine->touched[y * TOUCHED_ROW_BYTES + x / 8] >> (x & 7) & 1) != 0;
}

// Bus rows of the settle frames during and after the shake, kept in the buffer of the fast update, which a refresh
// does not use. A row of that buffer holds both.
static inline uint8_t* settle_row(int y, bool shaking) {
    return engine->fast + y * T5S3_EPD_FB_ROW_BYTES + (shaking ? 0 : T5S3_EPD_BUS_ROW_BYTES);
}

static void build_settle_rows(int y) {
    uint8_t* shake = settle_row(y, true);
    uint8_t* after = settle_row(y, false);
    memset(shake, 0, T5S3_EPD_BUS_ROW_BYTES);
    memset(after, 0, T5S3_EPD_BUS_ROW_BYTES);
    for (int i = 0; i < engine->rect_count; i++) {
        const T5s3EpdRect& rect = engine->rects[i];
        if (y < rect.y0 || y >= rect.y1) {
            continue;
        }
        for (int x = rect.x0 / 4 * 4; x < (rect.x1 + 3) / 4 * 4; x++) {
            if (!needs_refresh(y, x)) {
                continue;
            }
            const uint8_t level = engine->front[y * T5S3_EPD_FB_ROW_BYTES + x / 2] >> ((x & 1) * 4) & 0x0F;
            const int shift = (x & 3) * 2;
            const uint8_t shake_action = level == 15 ? T5S3_EPD_ACTION_LIGHT : T5S3_EPD_ACTION_DARK;
            const uint8_t after_action = level == 0 ? T5S3_EPD_ACTION_DARK : T5S3_EPD_ACTION_LIGHT;
            shake[x / 4] |= static_cast<uint8_t>(shake_action << shift);
            after[x / 4] |= static_cast<uint8_t>(after_action << shift);
        }
    }
}

static void IRAM_ATTR prepare_settle_row(int y, uint8_t* out, void*) {
    memcpy(out, settle_row(y, engine->settle_frame < REFRESH_SHAKE_FRAMES), T5S3_EPD_BUS_ROW_BYTES);
}

bool t5s3_epd_refresh(const T5s3EpdRect* rects, int count) {
    const int64_t started = esp_timer_get_time();
    // Only rows with pixels that need a refresh are sent
    int y_start = T5S3_EPD_HEIGHT;
    int y_end = 0;
    for (int i = 0; i < count; i++) {
        const T5s3EpdRect& rect = rects[i];
        for (int y = rect.y0; y < rect.y1; y++) {
            bool needed = false;
            for (int x = rect.x0; x < rect.x1 && !needed; x++) {
                needed = needs_refresh(y, x);
            }
            if (needed) {
                y_start = y < y_start ? y : y_start;
                y_end = y + 1 > y_end ? y + 1 : y_end;
            }
        }
    }
    if (y_start >= y_end) {
        return true;
    }

    engine->rects = rects;
    engine->rect_count = count;
    for (int y = y_start; y < y_end; y++) {
        build_settle_rows(y);
    }
    bool success = true;
    for (int frame = 0; frame < REFRESH_SETTLE_FRAMES && success; frame++) {
        engine->settle_frame = frame;
        const int64_t frame_start_us = esp_timer_get_time();
        success = t5s3_epd_board_frame(y_start, y_end, nullptr, FAST_HOLD_TICKS, prepare_settle_row, nullptr);
        pad_frame(frame_start_us, FAST_FRAME_US);
    }

    // Gray pixels are white now, black and white pixels show their level. The waveform starts from the white that its
    // gray levels were measured from.
    for (int i = 0; i < count && success; i++) {
        const T5s3EpdRect& rect = rects[i];
        for (int y = rect.y0; y < rect.y1; y++) {
            for (int x = rect.x0 / 4 * 4; x < (rect.x1 + 3) / 4 * 4; x++) {
                if (!needs_refresh(y, x)) {
                    continue;
                }
                uint8_t* byte = &engine->back[y * T5S3_EPD_FB_ROW_BYTES + x / 2];
                const int shift = (x & 1) * 4;
                const uint8_t level = engine->front[y * T5S3_EPD_FB_ROW_BYTES + x / 2] >> shift & 0x0F;
                const uint8_t shown = level == 0 ? 0 : 15;
                *byte = static_cast<uint8_t>((*byte & ~(0x0F << shift)) | (shown << shift));
                engine->touched[y * TOUCHED_ROW_BYTES + x / 8] &= static_cast<uint8_t>(~(1 << (x & 7)));
            }
        }
    }

    engine->to = engine->front;
    for (int phase = 0; phase < T5S3_EPD_QUALITY_PHASES && success; phase++) {
        t5s3_epd_build_quality_table(engine->table, phase);
        const int64_t frame_start_us = esp_timer_get_time();
        success = t5s3_epd_board_frame(y_start, y_end, nullptr, T5S3_EPD_QUALITY_HOLD[phase], prepare_refresh_row, nullptr);
        pad_frame(frame_start_us, QUALITY_FRAME_US);
    }
    if (!success) {
        LOG_E(TAG, "Refresh failed, a peripheral did not respond");
        return false;
    }

    // The mask covers groups of 4 pixels, so whole groups were driven
    for (int i = 0; i < count; i++) {
        const T5s3EpdRect& rect = rects[i];
        const int x0 = rect.x0 / 4 * 2;
        const int x1 = (rect.x1 + 3) / 4 * 2;
        for (int y = rect.y0; y < rect.y1; y++) {
            const size_t offset = y * T5S3_EPD_FB_ROW_BYTES + x0;
            memcpy(engine->back + offset, engine->front + offset, x1 - x0);
        }
    }

    LOG_D(TAG, "Refresh of %d areas in rows %d to %d took %d ms", count, y_start, y_end, static_cast<int>((esp_timer_get_time() - started) / 1000));
    return true;
}

static bool draw_clear_frames(uint8_t pattern, int count) {
    t5s3_epd_board_fill_rows(pattern);
    bool success = true;
    for (int i = 0; i < count && success; i++) {
        success = t5s3_epd_board_frame(0, T5S3_EPD_HEIGHT, nullptr, CLEAR_HOLD_TICKS, nullptr, nullptr);
        vTaskDelay(1);
    }
    return success;
}

bool t5s3_epd_clear() {
    const int64_t started = esp_timer_get_time();
    t5s3_epd_fill_white();
    const T5s3EpdRect panel = { 0, 0, T5S3_EPD_WIDTH, T5S3_EPD_HEIGHT };
    if (!t5s3_epd_refresh(&panel, 1)) {
        return false;
    }
    bool success = true;
    for (int cycle = 0; cycle < CLEAR_CYCLES && success; cycle++) {
        success = draw_clear_frames(T5S3_EPD_ALL_DARK, CLEAR_DARK_FRAMES) &&
            draw_clear_frames(T5S3_EPD_ALL_LIGHT, CLEAR_LIGHT_FRAMES) &&
            draw_clear_frames(0, CLEAR_REST_FRAMES);
    }
    if (!success) {
        LOG_E(TAG, "Clear failed, a peripheral did not respond");
        return false;
    }
    LOG_I(TAG, "Cleared in %d ms", static_cast<int>((esp_timer_get_time() - started) / 1000));
    return true;
}
