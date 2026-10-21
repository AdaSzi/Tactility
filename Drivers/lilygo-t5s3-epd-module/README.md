# lilygo-t5s3-epd-module

Kernel drivers for the 4.7" 960x540 `ED047TC1` e-paper panel of the LILYGO T5 4.7 Inch E-Paper S3 (V2.3/V2.4).

The drivers only work on this board. The pins of the panel, the 74HCT4094 shift register that carries the panel supplies and the sequence of the panel are fixed in the code, so the devicetree node only has the pixel clock as property.

There are two drivers. A devicetree uses one of them, the device `lilygo-t5-epd47-s3` uses the black and white one:

- `lilygo,t5s3-display` (`t5s3_display.cpp`): 16 gray levels. `draw_bitmap()` returns without waiting for the panel. A task shows each change at once with a black and white dither and draws the gray levels of a changed area once it had no change for 3 seconds. Only the pixels that need it are driven, so there is no flash of the whole screen.
- `lilygo,t5s3-display-bw` (`t5s3_display_bw.cpp`): black and white with a pixel state machine instead of waveform tables, like the M5Stack PaperS3 driver. Gray levels are dithered. It is the fastest and does not flash outside of `clear()` and `refresh()`.

Shared and internal files:

- `t5s3_epd_board.cpp`: the signalling of the board. It controls the shift register, the CKV gate clock (RMT peripheral) and the row transfer (LCD peripheral in i80 mode), and sends one frame at a time.
- `t5s3_epd.cpp`: the updates of the grayscale driver: the fast black and white update and the refresh with the waveform.
- `t5s3_epd_rows.h`: the conversion of framebuffer rows into bus data. It has no hardware access.
- `t5s3_epd_waveform.h`: the 30 phase drive table for 16 gray levels.

Frames are padded to a fixed period, because a pixel keeps its drive until its row is addressed again. The frame counts and the gray levels in the code were measured with a camera on the panel.

## License

The board signalling, the update logic of the grayscale driver and the waveform are derived from [epdiy](https://github.com/vroland/epdiy) and are licensed under the
[LGPL v3.0 or later](LICENSE-LGPL-3.0.md) (the LGPL supplements the [GPL v3.0](../../Documentation/LICENSE-GPL-3.0.md)):

- `source/t5s3_epd.cpp`
- `source/t5s3_epd.h`
- `source/t5s3_epd_board.cpp`
- `source/t5s3_epd_board.h`
- `source/t5s3_epd_rows.h`
- `source/t5s3_epd_waveform.h`

These files follow the sequences and timings of epdiy and contain its ED047TC1 waveform data. Each of them has an `SPDX-License-Identifier: LGPL-3.0-or-later` header.

The rest of the module (`source/t5s3_display.cpp`, `source/t5s3_display_bw.cpp`, `source/module.cpp`, the bindings and the headers in `include/`) is licensed under [Apache License v2.0](LICENSE-Apache-2.0.md). The pixel state machine in `t5s3_display_bw.cpp` follows the M5Stack PaperS3 display driver of Tactility.

A project that contains this module has to follow the LGPL for the files above.
