# helloVFD — MN12832L on ESP32-S3

Experimental native ESP-IDF 6.0.2 driver for the Noritake MN12832L 128×32
chip-in-glass vacuum fluorescent display. LVGL 9 produces an inverted I1
framebuffer. The MN12832L component encodes 44 overlapping-grid scan frames.

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

Adapter power: 5 V and common ground. The tested board has an internal filament
and high-voltage converter. **Verify the power-enable polarity on your own
adapter** before attempting illumination; GPIO17/18 HIGH has been used in the
bench experiments.

## Current experiment

`app_main()` renders `HELLO VFD!`, initializes the MN12832L GPIOs, runs a
**blanked SPI dry run**, then a **time-bounded illumination test**. The latter
warms the filament for two seconds before enabling HV, sends at most 80 whole
scan cycles with 2 MHz SPI2, and finally sets BLK HIGH, HV LOW, and EF LOW.
The serial console reports measured elapsed time and complete-refresh rate.

This is **not a continuously safe display controller**. A CPU hang or loss of
control while HV is applied needs an independent hardware blanking/interlock
mechanism. Run the experiment attended with an accessible 5 V disconnect. Do
not leave the experimental firmware powered unattended.

The older bit-banged `mn12832l_shift_bits()` and readback tests are retained.
They must be run **before** `mn12832l_spi_dry_run()` takes control of SIN/CLK.
Neither transport changes the experimental grid/anode mapping, which still
produces duplicate/adjacent physical columns in some positions.

Build/flash on the configured ESP-IDF 6.0.2 ESP32-S3 environment:

```sh
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

Board under test: ESP32-S3 N8R8 (8 MB quad flash, 8 MB octal PSRAM).
