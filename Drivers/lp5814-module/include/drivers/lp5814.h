// SPDX-License-Identifier: Apache-2.0
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

struct Lp5814Config {
    uint8_t address;
    // Bit mask of OUT0 to OUT3
    uint8_t channels;
    bool high_current;
    uint8_t dot_current;
    uint8_t brightness_default;
};

#ifdef __cplusplus
}
#endif
