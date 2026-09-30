// SPDX-License-Identifier: Apache-2.0
#include "t5s3_display.h"

#include <tactility/device.h>
#include <tactility/driver.h>
#include <tactility/drivers/display.h>
#include <tactility/error.h>
#include <tactility/log.h>
#include <tactility/module.h>
#include <tactility/time.h>

#include <epd_board.h>

#include <epdiy.h>

#include <cstdlib>
#include <cstring>

#define TAG "T5s3Display"
#define GET_CONFIG(device) (static_cast<const T5s3DisplayConfig*>((device)->config))

// Fast partial updates use MODE_DU, config->quality_draw_mode is used for quality refreshes
static constexpr EpdDrawMode FAST_DRAW_MODE = MODE_DU;

// An update covering at least this fraction of the panel is a full-screen change and gets a quality refresh
static constexpr float FULL_AREA_QUALITY_THRESHOLD = 0.6f;

// Fast updates allowed before a quality refresh limits ghosting. A full-screen redraw takes about 10 updates, so this must be higher
static constexpr uint32_t QUALITY_REFRESH_PARTIAL_COUNT = 20;

// Updates within this time after a quality update also use quality mode, so the tiles of one redraw stay consistent.
// It must stay well below the duration of a GC16 update.
static constexpr uint32_t QUALITY_HOLD_MS = 50;

// Maximum time that consecutive quality updates keep extending the hold window
static constexpr uint32_t QUALITY_HOLD_SESSION_MAX_MS = 200;

// 4x4 ordered (Bayer) dither thresholds, spread evenly across a 0-15 nibble range.
static constexpr uint8_t BAYER_4X4[4][4] = {
    { 0, 8, 2, 10 },
    { 12, 4, 14, 6 },
    { 3, 11, 1, 9 },
    { 15, 7, 13, 5 },
};

// Dithers an 8-bit luminance (0x00 black to 0xFF white) to a 4-bit level (0x0 black to 0xF white)
static inline uint8_t dither_to_nibble(uint8_t luminance, int32_t x, int32_t y) {
    // Scaling by 17 spreads the thresholds over one full quantization step
    const uint32_t threshold = BAYER_4X4[y & 3][x & 3] * 17U;
    const uint32_t level = (static_cast<uint32_t>(luminance) * 15U + threshold) / 255U;
    return static_cast<uint8_t>(level > 15U ? 15U : level);
}

// Binary variant for MODE_DU, which only draws black and white
static inline uint8_t dither_to_bw_nibble(uint8_t luminance, int32_t x, int32_t y) {
    // Scaling by 16 keeps the highest threshold at 240, so pure white (0xFF) is never classified as black
    const uint32_t threshold = BAYER_4X4[y & 3][x & 3] * 16U;
    return luminance > threshold ? 0xF : 0x0;
}

extern "C" {

extern Module lilygo_t5_epd47_s3_module;

// epd_hl_init() cannot be undone, so the highlevel state is kept across stop() and start()
static bool s_hl_initialized = false;
static EpdiyHighlevelState s_hl_state = {};

struct T5s3DisplayInternal {
    EpdiyHighlevelState hl_state;
    uint8_t* framebuffer;
    bool powered;
    uint32_t panel_pixel_count;
    // Fast updates since the last quality refresh
    uint32_t partial_count_since_quality;
    // Until this tick every update uses quality mode
    TickType_t quality_hold_until_tick;
    // Tick at which the current hold window started
    TickType_t quality_hold_session_start_tick;
};

static void power_on(T5s3DisplayInternal* internal) {
    if (!internal->powered) {
        epd_poweron();
        internal->powered = true;
    }
}

// region DisplayApi

static error_t t5s3_display_reset(Device* device) {
    auto* internal = static_cast<T5s3DisplayInternal*>(device_get_driver_data(device));
    // The panel has no reset line, so a power cycle is the closest equivalent
    epd_poweroff();
    internal->powered = false;
    power_on(internal);
    return ERROR_NONE;
}

/** Initialization is done in start() */
static error_t t5s3_display_init(Device*) {
    return ERROR_NONE;
}

/** Fills the panel white with one GC16 update, without the flashing cycles of epd_fullclear() */
static error_t t5s3_display_clear(Device* device) {
    auto* internal = static_cast<T5s3DisplayInternal*>(device_get_driver_data(device));
    const auto* config = GET_CONFIG(device);
    power_on(internal);
    epd_hl_set_all_white(&internal->hl_state);
    auto result = epd_hl_update_screen(&internal->hl_state, MODE_GC16, config->temperature_celsius);
    if (result == EPD_DRAW_SUCCESS) {
        internal->partial_count_since_quality = 0;
        internal->quality_hold_until_tick = 0;
    }
    return result == EPD_DRAW_SUCCESS ? ERROR_NONE : ERROR_RESOURCE;
}

/**
 * Redraws the current content with a GC16 update to remove ghosting.
 * The update skips pixels that did not change, so back_fb is inverted first to make every pixel differ.
 */
static error_t t5s3_display_refresh(Device* device) {
    auto* internal = static_cast<T5s3DisplayInternal*>(device_get_driver_data(device));
    const auto* config = GET_CONFIG(device);
    power_on(internal);
    const size_t fb_size = static_cast<size_t>(epd_width()) / 2 * epd_height();
    uint8_t* back_fb = internal->hl_state.back_fb;
    for (size_t i = 0; i < fb_size; i++) {
        back_fb[i] = static_cast<uint8_t>(~back_fb[i]);
    }
    auto result = epd_hl_update_screen(&internal->hl_state, MODE_GC16, config->temperature_celsius);
    if (result == EPD_DRAW_SUCCESS) {
        internal->partial_count_since_quality = 0;
        internal->quality_hold_until_tick = 0;
    }
    return result == EPD_DRAW_SUCCESS ? ERROR_NONE : ERROR_RESOURCE;
}

// Decides between a quality refresh and a fast MODE_DU update, *out_within_hold tells if the hold window applied
static bool should_use_quality_mode(T5s3DisplayInternal* internal, int32_t width, int32_t height, bool* out_within_hold) {
    const TickType_t now = get_ticks();

    const uint32_t area = static_cast<uint32_t>(width) * static_cast<uint32_t>(height);
    const bool is_full_screen_change = area >= static_cast<uint32_t>(
        static_cast<float>(internal->panel_pixel_count) * FULL_AREA_QUALITY_THRESHOLD
    );
    const bool partial_count_exceeded = internal->partial_count_since_quality >= QUALITY_REFRESH_PARTIAL_COUNT;
    const bool within_hold = now < internal->quality_hold_until_tick;
    *out_within_hold = within_hold;

    return is_full_screen_change || partial_count_exceeded || within_hold;
}

// Updates the counters after an update. A failed quality update leaves them unchanged and a failed fast update still counts.
// A quality update starts or extends the hold window, up to QUALITY_HOLD_SESSION_MAX_MS.
static void commit_quality_mode_decision(T5s3DisplayInternal* internal, bool used_quality, bool draw_succeeded, bool was_within_hold) {
    if (used_quality) {
        if (draw_succeeded) {
            internal->partial_count_since_quality = 0;
            const TickType_t now = get_ticks();
            if (!was_within_hold) {
                internal->quality_hold_session_start_tick = now;
            }
            const TickType_t session_elapsed = now - internal->quality_hold_session_start_tick;
            if (session_elapsed < millis_to_ticks(QUALITY_HOLD_SESSION_MAX_MS)) {
                internal->quality_hold_until_tick = now + millis_to_ticks(QUALITY_HOLD_MS);
            }
        }
    } else {
        internal->partial_count_since_quality++;
    }
}

// Reports GRAYSCALE8 so that draw_bitmap() is called per changed tile
static error_t t5s3_display_draw_bitmap(Device* device, int32_t x_start, int32_t y_start, int32_t x_end, int32_t y_end, const void* color_data) {
    auto* internal = static_cast<T5s3DisplayInternal*>(device_get_driver_data(device));
    const auto* config = GET_CONFIG(device);

    const int32_t width = x_end - x_start;
    const int32_t height = y_end - y_start;
    bool within_hold = false;
    const bool use_quality = should_use_quality_mode(internal, width, height, &within_hold);

    // color_data is 8-bit luminance (0x00 black to 0xFF white) and epdiy wants 4-bit levels (0x0 black to 0xF white)
    // Dithering uses 16 levels for quality updates and 2 levels for MODE_DU
    // epd_draw_pixel() applies the configured rotation
    const auto* src = static_cast<const uint8_t*>(color_data);
    const size_t src_stride = static_cast<size_t>(width);

    for (int32_t row = 0; row < height; row++) {
        const uint8_t* src_row = src + static_cast<size_t>(row) * src_stride;
        const int32_t display_y = y_start + row;

        for (int32_t col = 0; col < width; col++) {
            const int32_t display_x = x_start + col;
            const uint8_t nibble = use_quality
                ? dither_to_nibble(src_row[col], display_x, display_y)
                : dither_to_bw_nibble(src_row[col], display_x, display_y);
            epd_draw_pixel(display_x, display_y, static_cast<uint8_t>(nibble << 4), internal->framebuffer);
        }
    }

    const EpdRect update_area = {
        .x = x_start,
        .y = y_start,
        .width = static_cast<uint16_t>(width),
        .height = static_cast<uint16_t>(height)
    };

    power_on(internal);
    const auto draw_mode = use_quality ? config->quality_draw_mode : FAST_DRAW_MODE;
    auto draw_result = epd_hl_update_area(
        &internal->hl_state,
        static_cast<EpdDrawMode>(draw_mode | MODE_PACKING_2PPB),
        config->temperature_celsius,
        update_area
    );

    commit_quality_mode_decision(internal, use_quality, draw_result == EPD_DRAW_SUCCESS, within_hold);
    return draw_result == EPD_DRAW_SUCCESS ? ERROR_NONE : ERROR_RESOURCE;
}

static error_t t5s3_display_disp_on_off(Device* device, bool on_off) {
    auto* internal = static_cast<T5s3DisplayInternal*>(device_get_driver_data(device));
    if (on_off) {
        power_on(internal);
    } else if (internal->powered) {
        epd_poweroff();
        internal->powered = false;
    }
    return ERROR_NONE;
}

static DisplayColorFormat t5s3_display_get_color_format(Device*) {
    return DISPLAY_COLOR_FORMAT_GRAYSCALE8;
}

// LVGL and draw_bitmap() use the rotated resolution, epd_draw_pixel() maps it to the native panel
static uint16_t t5s3_display_get_resolution_x(Device*) {
    return static_cast<uint16_t>(epd_rotated_display_width());
}

static uint16_t t5s3_display_get_resolution_y(Device*) {
    return static_cast<uint16_t>(epd_rotated_display_height());
}

// endregion

static const DisplayApi t5s3_display_api = {
    // The driver converts into its own framebuffer, so the LVGL draw buffers may live in external RAM
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
    // The epdiy framebuffer is 4bpp and not in the reported GRAYSCALE8 format
    .get_frame_buffer = nullptr,
    .get_frame_buffer_count = nullptr,
    .get_backlight = nullptr,
    .has_capability = nullptr,
};

// region Driver lifecycle

static error_t start(Device* device) {
    const auto* config = GET_CONFIG(device);

    auto* internal = static_cast<T5s3DisplayInternal*>(malloc(sizeof(T5s3DisplayInternal)));
    if (internal == nullptr) {
        return ERROR_OUT_OF_MEMORY;
    }
    internal->powered = false;

    // The row-by-row render engine requires the 64K LUT. A short feed queue saves internal RAM.
    epd_init(&epd_board_lilygo_t5_47_s3, &ED047TC1, static_cast<EpdInitOptions>(EPD_LUT_64K | EPD_FEED_QUEUE_8));
    epd_set_rotation(config->rotation);

    if (!s_hl_initialized) {
        s_hl_state = epd_hl_init(EPD_BUILTIN_WAVEFORM);
        if (s_hl_state.front_fb == nullptr) {
            LOG_E(TAG, "Failed to initialize EPDiy highlevel state");
            epd_deinit();
            free(internal);
            return ERROR_RESOURCE;
        }
        s_hl_initialized = true;
    } else {
        LOG_I(TAG, "Reusing existing EPDiy highlevel state");
    }

    internal->hl_state = s_hl_state;
    internal->framebuffer = epd_hl_get_framebuffer(&internal->hl_state);

    internal->panel_pixel_count = static_cast<uint32_t>(epd_rotated_display_width()) * static_cast<uint32_t>(epd_rotated_display_height());
    internal->partial_count_since_quality = 0;
    internal->quality_hold_until_tick = 0;
    internal->quality_hold_session_start_tick = 0;

    device_set_driver_data(device, internal);

    // The boot splash leaves ghosting, so clear the panel with the full flash cycle before LVGL draws
    power_on(internal);
    epd_fullclear(&internal->hl_state, config->temperature_celsius);

    LOG_I(TAG, "EPDiy initialized (%dx%d native, %dx%d rotated)", epd_width(), epd_height(), epd_rotated_display_width(), epd_rotated_display_height());
    return ERROR_NONE;
}

static error_t stop(Device* device) {
    auto* internal = static_cast<T5s3DisplayInternal*>(device_get_driver_data(device));

    if (internal->powered) {
        epd_poweroff();
        internal->powered = false;
    }

    epd_deinit();

    free(internal);
    device_set_driver_data(device, nullptr);
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
    .owner = &lilygo_t5_epd47_s3_module,
    .internal = nullptr
};

}
