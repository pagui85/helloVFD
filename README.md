# helloVFD — MN12832L on ESP32-S3

Experimental ESP-IDF 6.0.2 driver and bench diagnostics for the Noritake
MN12832L 128×32 VFD.

## Bench wiring

| ESP32-S3 GPIO | Adapter signal |
| --- | --- |
| 11 | SIN / SPI MOSI |
| 12 | CLK / SPI SCLK |
| 13 | SO1 (diagnostics only) |
| 14 | SO2 (diagnostics only) |
| 15 | BLK (HIGH = blanked) |
| 16 | LAT |
| 17 | HV enable |
| 18 | EF filament enable |

Adapter power: 5 V and common ground. The tested adapter has filament and HV
generation. Confirm your adapter's enable polarities before using this code.

## Current startup test: 128×32 graphics reel

The startup path initializes the display and executes a blanked SPI dry run,
then `mn12832l_spi_graphics_demo()`. This is a **first-pass hardware demo**,
not the final LVGL refresh engine. It uses a separate 1-bit framebuffer and
sequential single-grid SPI scanning. The old paired-grid encoder and earlier
hardware experiments remain in the source for comparison.

Each scene is **automatically timed**; no keys are required:

| Scene # | Duration | What it shows |
| --- | --- | --- |
| 1 | 6 s | Large `HELLO VFD!` and `123` |
| 2 | 6 s | Box, lines, solid rectangle, and diagonal X |
| 3 | 6 s | Checkerboard and horizontal bounds |
| 4 | 9 s | Bouncing rectangle |
| 5 | 9 s | Scrolling `NORITAKE 128X32` |
| 6 | 4 s | `END!` |

After ~40 seconds of HV-on time, the program **blanks the output and disables
HV and EF**. Press `q` in the serial monitor to stop early. The firmware
also has a 43-second hard HV-time limit. Filament preheat is 2 seconds.

### Critical mapping caveat

Physical columns **4–12** have been characterized using individual grid/anode
and independent-pixel tests. The first-24 mapping contains a mix of observed
and inferred choices. **Columns 25–128 are an extrapolation**, using repeating
ABC/DEF anode groupings and adjacent grid preference. Expect possible
misplaced, repeated, or missing columns, especially beyond the measured area.

The test uses **250 µs illumination per active grid slot** and **does not**
enable the earlier experimental extra dwell for dim column 5. We want to see
where the provisional full-width mapping fails before tuning brightness.

### Safety

This is attended-only experimental hardware. A crashed CPU or unexpected
control fault while HV is enabled is not covered by an independent hardware
interlock. Keep a physical 5 V disconnect accessible; do not leave the display
unattended. The elapsed-time limit and normal shutdown are software
safeguards, not a substitute for an independent blanking watchdog.

## Build and flash

Run from an ESP-IDF 6.0.2 environment:

```sh
idf.py build
idf.py flash monitor
```

Board under test: ESP32-S3 N8R8 (8 MB flash, 8 MB PSRAM).

The prototype graphics renderer is intentionally distinct from LVGL. The
original LVGL I1 framebuffer and flush callback remain available, but the
current startup reel does not consume them directly.
