// SPDX-License-Identifier: Apache-2.0
#include <drivers/t5s3_display.h>

#include <tactility/device.h>
#include <tactility/driver.h>
#include <tactility/drivers/display.h>
#include <tactility/error.h>
#include <tactility/log.h>
#include <tactility/memory.h>
#include <tactility/module.h>

#include "t5s3_epd.h"

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <algorithm>
#include <cstdint>
#include <cstring>

#define TAG "T5s3Display"
#define GET_CONFIG(device) (static_cast<const T5s3DisplayConfig*>((device)->config))

// draw_bitmap() writes 16 gray levels into a staging buffer and returns. A task shows each change with a fast update
// that dithers the changed pixels to black and white. Once an area had no change for REFRESH_DELAY_MS, a refresh
// draws its gray levels and clears the ghosting.

// Areas that wait for a refresh, more are merged
static constexpr int MAX_AREAS = 8;

// A refresh takes more than a second and delays the changes that come during it, so it waits for a pause.
// Until then the changed areas show their dither.
static constexpr uint32_t REFRESH_DELAY_MS = 3000;

// Brightness of the levels that look different, measured with a camera. Levels 10 to 14 look like white.
static constexpr int TONE_STEPS = 11;
static constexpr uint8_t TONE_LEVELS[TONE_STEPS] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 15 };
static constexpr uint8_t TONE_BRIGHTNESS[TONE_STEPS] = { 14, 17, 26, 52, 81, 111, 129, 143, 151, 155, 158 };

// Darker luminance is drawn black. The edges of text then keep the weight that the dither of the fast update gives them.
static constexpr int BLACK_LUMINANCE = 64;

static constexpr uint32_t IDLE_POWER_OFF_MS = 2000;

static constexpr uint32_t CMD_UPDATE = 1U << 0;
static constexpr uint32_t CMD_CLEAR = 1U << 1;
static constexpr uint32_t CMD_REFRESH = 1U << 2;
static constexpr uint32_t CMD_POWER_OFF = 1U << 3;
static constexpr uint32_t CMD_EXIT = 1U << 4;

// 4x4 ordered (Bayer) dither thresholds
static constexpr uint8_t BAYER_4X4[4][4] = {
    { 0, 8, 2, 10 },
    { 12, 4, 14, 6 },
    { 3, 11, 1, 9 },
    { 15, 7, 13, 5 },
};

// A luminance lies between two levels of the panel and is dithered between them
struct Tone {
    uint8_t darker;
    uint8_t lighter;
    // Share of the lighter level, 0 to 16
    uint8_t share;
};

static void build_tones(Tone* tones) {
    for (int luminance = 0; luminance < 256; luminance++) {
        if (luminance < BLACK_LUMINANCE) {
            tones[luminance] = { 0, 0, 0 };
            continue;
        }
        const float brightness = TONE_BRIGHTNESS[0] + (TONE_BRIGHTNESS[TONE_STEPS - 1] - TONE_BRIGHTNESS[0]) * luminance / 255.0f;
        int step = 0;
        while (step < TONE_STEPS - 2 && brightness > TONE_BRIGHTNESS[step + 1]) {
            step++;
        }
        const float share = (brightness - TONE_BRIGHTNESS[step]) / (TONE_BRIGHTNESS[step + 1] - TONE_BRIGHTNESS[step]);
        tones[luminance].darker = TONE_LEVELS[step];
        tones[luminance].lighter = TONE_LEVELS[step + 1];
        tones[luminance].share = static_cast<uint8_t>(std::clamp(static_cast<int>(share * 16.0f + 0.5f), 0, 16));
    }
}

// Share of white pixels, 0 to 16, that the black and white dither of a level gets
static constexpr uint8_t white_share(int level) {
    for (int step = 0; step < TONE_STEPS; step++) {
        if (TONE_LEVELS[step] == level) {
            return static_cast<uint8_t>((TONE_BRIGHTNESS[step] - TONE_BRIGHTNESS[0]) * 16 / (TONE_BRIGHTNESS[TONE_STEPS - 1] - TONE_BRIGHTNESS[0]));
        }
    }
    // Unused levels look like white
    return 16;
}

static constexpr uint8_t WHITE_SHARES[16] = {
    white_share(0), white_share(1), white_share(2), white_share(3), white_share(4), white_share(5), white_share(6), white_share(7),
    white_share(8), white_share(9), white_share(10), white_share(11), white_share(12), white_share(13), white_share(14), white_share(15),
};

static inline uint8_t dither_to_level(const Tone& tone, int32_t x, int32_t y) {
    return tone.share > BAYER_4X4[y & 3][x & 3] ? tone.lighter : tone.darker;
}

extern "C" {

// Rows [y0, y1)
struct Rows {
    int y0, y1;

    bool empty() const { return y1 <= y0; }

    void merge(const Rows& other) {
        if (other.empty()) return;
        if (empty()) {
            *this = other;
            return;
        }
        y0 = std::min(y0, other.y0);
        y1 = std::max(y1, other.y1);
    }
};

static constexpr Rows NO_ROWS = { 0, 0 };

struct T5s3DisplayInternal {
    TaskHandle_t task;
    SemaphoreHandle_t request_mutex;
    SemaphoreHandle_t request_done;
    // Guards staging
    SemaphoreHandle_t staging_mutex;
    portMUX_TYPE lock;
    volatile bool exited;
    bool powered;
    // Result of the last clear, set by the task
    bool clear_succeeded;
    // Rows changed in staging and not yet picked up by the task, guarded by lock
    Rows pending;
    // Time of the oldest change not yet shown and the longest draw_bitmap() call since, guarded by lock
    int64_t pending_since_us;
    int64_t draw_max_us;
    // 4 bit levels written by draw_bitmap(), copied into the frame buffer of the panel by the task
    uint8_t* staging;
    // Panel levels for each luminance
    Tone* tones;
};

static inline void set_level(uint8_t* buffer, int32_t x, int32_t y, uint8_t level) {
    uint8_t* byte = &buffer[y * T5S3_EPD_FB_ROW_BYTES + x / 2];
    if (x % 2) {
        *byte = (*byte & 0x0F) | static_cast<uint8_t>(level << 4);
    } else {
        *byte = (*byte & 0xF0) | (level & 0x0F);
    }
}

// region Task

static void power_on(T5s3DisplayInternal* internal) {
    if (internal->powered) return;
    t5s3_epd_board_power_on();
    internal->powered = true;
}

static void power_off(T5s3DisplayInternal* internal) {
    if (!internal->powered) return;
    t5s3_epd_board_power_off();
    internal->powered = false;
}

// Copies the rows into the frame buffer and their black and white dither into the dither buffer
static void copy_staging(T5s3DisplayInternal* internal, const Rows& rows) {
    const size_t offset = rows.y0 * T5S3_EPD_FB_ROW_BYTES;
    uint8_t* framebuffer = t5s3_epd_framebuffer();
    xSemaphoreTake(internal->staging_mutex, portMAX_DELAY);
    memcpy(framebuffer + offset, internal->staging + offset, (rows.y1 - rows.y0) * T5S3_EPD_FB_ROW_BYTES);
    xSemaphoreGive(internal->staging_mutex);

    uint8_t* dither = t5s3_epd_dither_buffer();
    for (int y = rows.y0; y < rows.y1; y++) {
        const uint8_t* levels = framebuffer + y * T5S3_EPD_FB_ROW_BYTES;
        uint8_t* out = dither + y * T5S3_EPD_FB_ROW_BYTES;
        const uint8_t* thresholds = BAYER_4X4[y & 3];
        for (int i = 0; i < T5S3_EPD_FB_ROW_BYTES; i++) {
            const int x = i * 2;
            const uint8_t first = WHITE_SHARES[levels[i] & 0x0F] > thresholds[x & 3] ? 0x0F : 0x00;
            const uint8_t second = WHITE_SHARES[levels[i] >> 4] > thresholds[(x + 1) & 3] ? 0xF0 : 0x00;
            out[i] = first | second;
        }
    }
}

static bool run_clear(T5s3DisplayInternal* internal) {
    power_on(internal);
    xSemaphoreTake(internal->staging_mutex, portMAX_DELAY);
    memset(internal->staging, 0xFF, T5S3_EPD_FB_BYTES);
    xSemaphoreGive(internal->staging_mutex);
    t5s3_epd_fill_white();
    const T5s3EpdRect panel = { 0, 0, T5S3_EPD_WIDTH, T5S3_EPD_HEIGHT };
    return t5s3_epd_refresh(&panel, 1);
}

// Areas that got fast updates and still need a refresh for their gray levels
struct Areas {
    T5s3EpdRect rects[MAX_AREAS];
    // Time of the last change of each area
    int64_t changed_us[MAX_AREAS];
    int count;
};

static int64_t rect_size(const T5s3EpdRect& rect) {
    return static_cast<int64_t>(rect.x1 - rect.x0) * (rect.y1 - rect.y0);
}

static T5s3EpdRect rect_union(const T5s3EpdRect& a, const T5s3EpdRect& b) {
    return { std::min(a.x0, b.x0), std::min(a.y0, b.y0), std::max(a.x1, b.x1), std::max(a.y1, b.y1) };
}

// Merges the area into the one whose union grows the least, unless it is far from all of them and there is room
static void add_area(Areas* areas, const T5s3EpdRect& rect, int64_t now_us) {
    int best = -1;
    int64_t best_growth = 0;
    for (int i = 0; i < areas->count; i++) {
        const T5s3EpdRect& other = areas->rects[i];
        const int64_t growth = rect_size(rect_union(other, rect)) - rect_size(other) - rect_size(rect);
        if (best < 0 || growth < best_growth) {
            best = i;
            best_growth = growth;
        }
    }
    if (best >= 0 && (best_growth <= 0 || areas->count == MAX_AREAS)) {
        areas->rects[best] = rect_union(areas->rects[best], rect);
        areas->changed_us[best] = now_us;
    } else {
        areas->rects[areas->count] = rect;
        areas->changed_us[areas->count] = now_us;
        areas->count++;
    }
}

// Each area waits for its own pause, so changes elsewhere do not hold it back
static int64_t next_refresh_us(const Areas& areas) {
    int64_t next_us = INT64_MAX;
    for (int i = 0; i < areas.count; i++) {
        next_us = std::min(next_us, areas.changed_us[i] + REFRESH_DELAY_MS * 1000LL);
    }
    return next_us;
}

// Refreshes the areas that are due, or all of them
static void refresh_areas(Areas* areas, bool all) {
    Areas due = {};
    Areas waiting = {};
    const int64_t now_us = esp_timer_get_time();
    for (int i = 0; i < areas->count; i++) {
        Areas* target = all || areas->changed_us[i] + REFRESH_DELAY_MS * 1000LL <= now_us ? &due : &waiting;
        target->rects[target->count] = areas->rects[i];
        target->changed_us[target->count] = areas->changed_us[i];
        target->count++;
    }
    if (due.count == 0) {
        return;
    }
    // A failed refresh keeps its areas, so they are tried again
    if (!t5s3_epd_refresh(due.rects, due.count)) {
        return;
    }
    *areas = waiting;
}

static void display_task(void* arg) {
    auto* internal = static_cast<T5s3DisplayInternal*>(arg);
    Areas areas = {};

    while (true) {
        uint32_t commands = 0;
        TickType_t timeout = portMAX_DELAY;
        if (areas.count > 0) {
            const int64_t wait_us = std::max<int64_t>(0, next_refresh_us(areas) - esp_timer_get_time());
            timeout = pdMS_TO_TICKS(wait_us / 1000 + 1);
        } else if (internal->powered) {
            timeout = pdMS_TO_TICKS(IDLE_POWER_OFF_MS);
        }
        const bool notified = xTaskNotifyWait(0, UINT32_MAX, &commands, timeout) == pdTRUE;

        if (commands & CMD_EXIT) {
            break;
        }

        if (commands & CMD_POWER_OFF) {
            power_off(internal);
        }

        if (commands & CMD_CLEAR) {
            // The clear draws all rows, so earlier changes are included
            portENTER_CRITICAL(&internal->lock);
            internal->pending = NO_ROWS;
            internal->pending_since_us = 0;
            internal->draw_max_us = 0;
            portEXIT_CRITICAL(&internal->lock);
            internal->clear_succeeded = run_clear(internal);
            areas.count = 0;
            xSemaphoreGive(internal->request_done);
        }

        // A refresh of the whole panel only drives the pixels that do not show their level
        const bool refresh_all = (commands & CMD_REFRESH) != 0;
        if (refresh_all) {
            areas.rects[0] = { 0, 0, T5S3_EPD_WIDTH, T5S3_EPD_HEIGHT };
            areas.changed_us[0] = esp_timer_get_time();
            areas.count = 1;
        }

        Rows pending;
        int64_t pending_since_us;
        int64_t draw_max_us;
        portENTER_CRITICAL(&internal->lock);
        pending = internal->pending;
        pending_since_us = internal->pending_since_us;
        draw_max_us = internal->draw_max_us;
        internal->pending = NO_ROWS;
        internal->pending_since_us = 0;
        internal->draw_max_us = 0;
        portEXIT_CRITICAL(&internal->lock);

        if (!pending.empty()) {
            copy_staging(internal, pending);
            power_on(internal);
            T5s3EpdRect changed;
            if (t5s3_epd_show_changes(pending.y0, pending.y1, &changed) && changed.x1 > changed.x0) {
                add_area(&areas, changed, esp_timer_get_time());
            }
            LOG_D(TAG, "Rows %d to %d shown after %d ms (draw_bitmap up to %d us)", pending.y0, pending.y1,
                static_cast<int>((esp_timer_get_time() - pending_since_us) / 1000), static_cast<int>(draw_max_us));
        } else if (areas.count > 0 && (refresh_all || next_refresh_us(areas) <= esp_timer_get_time())) {
            power_on(internal);
            refresh_areas(&areas, refresh_all);
        } else if (!notified && areas.count == 0) {
            power_off(internal);
        }
    }

    power_off(internal);
    internal->exited = true;
    vTaskDelete(nullptr);
}

static error_t clear_blocking(T5s3DisplayInternal* internal) {
    xSemaphoreTake(internal->request_mutex, portMAX_DELAY);
    xTaskNotify(internal->task, CMD_CLEAR, eSetBits);
    xSemaphoreTake(internal->request_done, portMAX_DELAY);
    const bool success = internal->clear_succeeded;
    xSemaphoreGive(internal->request_mutex);
    return success ? ERROR_NONE : ERROR_RESOURCE;
}

// endregion

// region DisplayApi

static error_t t5s3_display_reset(Device* device) {
    auto* internal = static_cast<T5s3DisplayInternal*>(device_get_driver_data(device));
    // The panel has no reset line, it powers up again with the next update
    xTaskNotify(internal->task, CMD_POWER_OFF, eSetBits);
    return ERROR_NONE;
}

/** Initialization is done in start() */
static error_t t5s3_display_init(Device*) {
    return ERROR_NONE;
}

/** Fills the panel white, without the flashing cycles of the initial clear */
static error_t t5s3_display_clear(Device* device) {
    return clear_blocking(static_cast<T5s3DisplayInternal*>(device_get_driver_data(device)));
}

/** Draws the gray levels of all pixels that show a dither now, without waiting for it */
static error_t t5s3_display_refresh(Device* device) {
    auto* internal = static_cast<T5s3DisplayInternal*>(device_get_driver_data(device));
    xTaskNotify(internal->task, CMD_REFRESH, eSetBits);
    return ERROR_NONE;
}

// Reports GRAYSCALE8 so that draw_bitmap() is called per changed tile
static error_t t5s3_display_draw_bitmap(Device* device, int32_t x_start, int32_t y_start, int32_t x_end, int32_t y_end, const void* color_data) {
    auto* internal = static_cast<T5s3DisplayInternal*>(device_get_driver_data(device));
    const int64_t start_us = esp_timer_get_time();

    // color_data is 8-bit luminance (0x00 black to 0xFF white) and the panel wants 4-bit levels (0x0 black to 0xF white)
    // Level 15 is white, levels 10 to 14 are not used
    const auto* src = static_cast<const uint8_t*>(color_data);
    const int32_t width = x_end - x_start;
    xSemaphoreTake(internal->staging_mutex, portMAX_DELAY);
    for (int32_t y = y_start; y < y_end; y++) {
        const uint8_t* src_row = src + (y - y_start) * width;
        for (int32_t x = x_start; x < x_end; x++) {
            set_level(internal->staging, x, y, dither_to_level(internal->tones[src_row[x - x_start]], x, y));
        }
    }
    xSemaphoreGive(internal->staging_mutex);

    const int64_t draw_us = esp_timer_get_time() - start_us;
    portENTER_CRITICAL(&internal->lock);
    internal->pending.merge({ y_start, y_end });
    if (internal->pending_since_us == 0) {
        internal->pending_since_us = start_us;
    }
    internal->draw_max_us = std::max(internal->draw_max_us, draw_us);
    portEXIT_CRITICAL(&internal->lock);
    xTaskNotify(internal->task, CMD_UPDATE, eSetBits);
    return ERROR_NONE;
}

static error_t t5s3_display_disp_on_off(Device* device, bool on_off) {
    auto* internal = static_cast<T5s3DisplayInternal*>(device_get_driver_data(device));
    if (!on_off) {
        xTaskNotify(internal->task, CMD_POWER_OFF, eSetBits);
    }
    return ERROR_NONE;
}

static DisplayColorFormat t5s3_display_get_color_format(Device*) {
    return DISPLAY_COLOR_FORMAT_GRAYSCALE8;
}

static uint16_t t5s3_display_get_resolution_x(Device*) {
    return T5S3_EPD_WIDTH;
}

static uint16_t t5s3_display_get_resolution_y(Device*) {
    return T5S3_EPD_HEIGHT;
}

// endregion

static const DisplayApi t5s3_display_api = {
    // The driver converts into its own buffer, so the LVGL draw buffers may live in external RAM
    .capabilities = DISPLAY_CAPABILITY_ON_OFF | DISPLAY_CAPABILITY_SLOW_REFRESH | DISPLAY_CAPABILITY_PREFER_EXTERNAL_RAM,
    .reset = t5s3_display_reset,
    .init = t5s3_display_init,
    .draw_bitmap = t5s3_display_draw_bitmap,
    .clear = t5s3_display_clear,
    .refresh = t5s3_display_refresh,
    .mirror = nullptr,
    .swap_xy = nullptr,
    .get_swap_xy = nullptr,
    .get_mirror_x = nullptr,
    .get_mirror_y = nullptr,
    .set_gap = nullptr,
    .get_gap_x = nullptr,
    .get_gap_y = nullptr,
    .invert_color = nullptr,
    .disp_on_off = t5s3_display_disp_on_off,
    .disp_sleep = nullptr,
    .get_color_format = t5s3_display_get_color_format,
    .get_resolution_x = t5s3_display_get_resolution_x,
    .get_resolution_y = t5s3_display_get_resolution_y,
    // The framebuffer is 4bpp and not in the reported GRAYSCALE8 format
    .get_frame_buffer = nullptr,
    .get_frame_buffer_count = nullptr,
    .get_backlight = nullptr,
    .has_capability = nullptr,
    .get_shape = nullptr,
};

// region Driver lifecycle

static void destroy(T5s3DisplayInternal* internal) {
    if (internal->request_mutex) vSemaphoreDelete(internal->request_mutex);
    if (internal->request_done) vSemaphoreDelete(internal->request_done);
    if (internal->staging_mutex) vSemaphoreDelete(internal->staging_mutex);
    memory_free(internal->staging);
    memory_free(internal->tones);
    memory_free(internal);
}

static error_t start(Device* device) {
    const auto* config = GET_CONFIG(device);

    static const MemoryPolicy internal_policy = { .required = MEMORY_CAPABILITY_INTERNAL, .desired = 0, .alignment = 0 };
    static const MemoryPolicy external_policy = { .required = 0, .desired = MEMORY_CAPABILITY_EXTERNAL, .alignment = 16 };
    auto* internal = static_cast<T5s3DisplayInternal*>(memory_calloc_with_policy(1, sizeof(T5s3DisplayInternal), &internal_policy));
    if (internal == nullptr) {
        return ERROR_OUT_OF_MEMORY;
    }
    internal->lock = portMUX_INITIALIZER_UNLOCKED;
    internal->request_mutex = xSemaphoreCreateMutex();
    internal->request_done = xSemaphoreCreateBinary();
    internal->staging_mutex = xSemaphoreCreateMutex();
    internal->staging = static_cast<uint8_t*>(memory_alloc_with_policy(T5S3_EPD_FB_BYTES, &external_policy));
    internal->tones = static_cast<Tone*>(memory_alloc_with_policy(256 * sizeof(Tone), &external_policy));
    if (!internal->request_mutex || !internal->request_done || !internal->staging_mutex || !internal->staging || !internal->tones) {
        destroy(internal);
        return ERROR_OUT_OF_MEMORY;
    }
    memset(internal->staging, 0xFF, T5S3_EPD_FB_BYTES);
    build_tones(internal->tones);

    if (!t5s3_epd_init(config->pixel_clock_hz)) {
        destroy(internal);
        return ERROR_RESOURCE;
    }

    // The boot splash leaves ghosting, so clear the panel with the full flash cycle before LVGL draws
    power_on(internal);
    t5s3_epd_clear();

    if (xTaskCreatePinnedToCore(display_task, "t5s3_display", 4096, internal, 1, &internal->task, 1) != pdPASS) {
        LOG_E(TAG, "Failed to create display task");
        power_off(internal);
        t5s3_epd_deinit();
        destroy(internal);
        return ERROR_RESOURCE;
    }

    device_set_driver_data(device, internal);
    LOG_I(TAG, "Initialized (%dx%d)", T5S3_EPD_WIDTH, T5S3_EPD_HEIGHT);
    return ERROR_NONE;
}

static error_t stop(Device* device) {
    auto* internal = static_cast<T5s3DisplayInternal*>(device_get_driver_data(device));
    xTaskNotify(internal->task, CMD_EXIT, eSetBits);
    while (!internal->exited) {
        vTaskDelay(1);
    }
    device_set_driver_data(device, nullptr);
    t5s3_epd_deinit();
    destroy(internal);
    return ERROR_NONE;
}

// endregion

Driver t5s3_display_driver = {
    .name = "t5s3-display",
    .compatible = (const char*[]) { "lilygo,t5s3-display", nullptr },
    .start_device = start,
    .stop_device = stop,
    .probe = nullptr,
    .api = &t5s3_display_api,
    .device_type = &DISPLAY_TYPE,
    .internal = nullptr
};

}
