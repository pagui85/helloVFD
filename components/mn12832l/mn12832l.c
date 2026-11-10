/* MN12832L ESP32-S3 driver and diagnostics.
 * The bit-banged shift routine is retained for serial-chain readback.
 * The SPI path uses the same GPIO11/GPIO12 wiring and is the only transport
 * used by the short production-speed illumination experiment.
 */
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "mn12832l.h"

/* Change these to match YOUR dev board's free GPIOs. */
#define PIN_SIN GPIO_NUM_11
#define PIN_CLK GPIO_NUM_12
#define PIN_SO1 GPIO_NUM_13
#define PIN_SO2 GPIO_NUM_14
#define PIN_BLK GPIO_NUM_15
#define PIN_LAT GPIO_NUM_16
#define PIN_HV GPIO_NUM_17
#define PIN_EF GPIO_NUM_18

#define MN12832L_FRAME_BYTES 30
#define MN12832L_SCAN_COUNT 44
#define MN12832L_SPI_HZ 2000000

static spi_device_handle_t spi_device;

/* Must be called only after mn12832l_init() and after all bit-bang tests.
 * SPI takes ownership of SIN and CLK through the GPIO matrix. */
static esp_err_t mn12832l_spi_init(void) {
  if (spi_device != NULL) {
    return ESP_OK;
  }

  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);

  spi_bus_config_t bus = {
      .mosi_io_num = PIN_SIN,
      .miso_io_num = -1,
      .sclk_io_num = PIN_CLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = MN12832L_FRAME_BYTES,
  };
  esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED);
  if (err != ESP_OK) {
    return err;
  }

  spi_device_interface_config_t dev = {
      .clock_speed_hz = MN12832L_SPI_HZ,
      .mode = 0,
      .spics_io_num = -1, // The VFD has no chip-select pin.
      .queue_size = 1,
  };
  err = spi_bus_add_device(SPI2_HOST, &dev, &spi_device);
  if (err != ESP_OK) {
    spi_bus_free(SPI2_HOST);
  }
  return err;
}

static esp_err_t mn12832l_spi_shift(const uint8_t *data) {
  spi_transaction_t transfer = {
      .length = MN12832L_FRAME_BYTES * 8,
      .tx_buffer = data,
  };
  return spi_device_polling_transmit(spi_device, &transfer);
}

static void mn12832l_shift_bits(const uint8_t *data, size_t bit_count,
                                uint8_t *so1_capture, uint8_t *so2_capture);

static void mn12832l_set_bit(uint8_t *data, unsigned bit_number) {
  // The datasheet numbers bits starting at 1.
  if (bit_number < 1 || bit_number > 240) {
    return;
  }

  unsigned index = bit_number - 1;

  unsigned byte_index = index / 8;
  unsigned bit_index = index % 8;

  data[byte_index] |= (uint8_t)(0x80u >> bit_index);
}

static void mn12832l_select_grid(uint8_t *data, uint8_t grid) {
  // The MN12832L has 44 grid outputs.
  if (grid < 1 || grid > 44) {
    return;
  }

  // Grid 1 begins at register bit 193.
  mn12832l_set_bit(data, 192 + grid);
}

static void mn12832l_set_anode(uint8_t *data, uint8_t row, char anode) {
  // There are 32 rows, each containing six anodes.
  if (row < 1 || row > 32) {
    return;
  }

  // Shift-register order is A, F, B, E, C, D.
  const char order[] = {'A', 'F', 'B', 'E', 'C', 'D'};

  for (int i = 0; i < 6; i++) {
    if (order[i] == anode) {
      unsigned bit_number = ((row - 1) * 6) + i + 1;

      mn12832l_set_bit(data, bit_number);
      return;
    }
  }
}

static void mn12832l_encode_scan(const uint8_t *framebuffer, uint8_t scan,
                                 uint8_t *output) {
  // Start with all electrodes disabled.
  memset(output, 0, MN12832L_FRAME_BYTES);

  if (scan >= 44) {
    return;
  }

  // Select the overlapping grid pair.
  if (scan > 0) {
    mn12832l_select_grid(output, scan);
  }

  mn12832l_select_grid(output, scan + 1);

  // Three pixel columns per scan.
  unsigned start_x = scan * 3;

  // Alternate between anodes ABC and DEF.
  const char *anodes = (scan % 2 == 0) ? "ABC" : "DEF";

  for (unsigned y = 0; y < 32; y++) {
    for (unsigned column = 0; column < 3; column++) {

      unsigned x = start_x + column;

      if (x >= 128) {
        continue;
      }

      // Our framebuffer is 128 pixels wide, 1 bit/pixel.
      unsigned byte_index = y * 16 + x / 8;
      uint8_t mask = 0x80u >> (x % 8);

      // Our LVGL I1 buffer uses zero for illuminated pixels.
      bool illuminated = (framebuffer[byte_index] & mask) == 0;

      if (illuminated) {
        mn12832l_set_anode(output, y + 1, anodes[column]);
      }
    }
  }
}

static void mn12832l_encode_frame(
    const uint8_t *framebuffer,
    uint8_t output[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES]) {
  for (uint8_t scan = 0; scan < MN12832L_SCAN_COUNT; scan++) {
    mn12832l_encode_scan(framebuffer, scan, output[scan]);
  }
}

/* BLK is asserted throughout every SPI transfer and latch pulse. The output
 * is enabled only for the bounded dwell interval following a valid transfer. */
static esp_err_t mn12832l_spi_scan_dwell(
    const uint8_t frame[MN12832L_FRAME_BYTES], bool illuminate,
    uint32_t dwell_us) {
  gpio_set_level(PIN_BLK, 1);
  esp_err_t err = mn12832l_spi_shift(frame);
  if (err != ESP_OK) {
    return err;
  }
  gpio_set_level(PIN_LAT, 1);
  esp_rom_delay_us(1);
  gpio_set_level(PIN_LAT, 0);

  if (illuminate) {
    gpio_set_level(PIN_BLK, 0);
    esp_rom_delay_us(dwell_us);
    gpio_set_level(PIN_BLK, 1);
  }
  return ESP_OK;
}

/* Default 250 us grid dwell is unchanged for all existing tests. */
static esp_err_t mn12832l_spi_scan(
    const uint8_t frame[MN12832L_FRAME_BYTES], bool illuminate) {
  return mn12832l_spi_scan_dwell(frame, illuminate, 250);
}

esp_err_t mn12832l_spi_dry_run(const uint8_t *framebuffer) {
  if (framebuffer == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  uint8_t scans[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES];
  mn12832l_encode_frame(framebuffer, scans);
  esp_err_t err = mn12832l_spi_init();
  if (err != ESP_OK) {
    return err;
  }

  int64_t start = esp_timer_get_time();
  for (int scan = 0; scan < MN12832L_SCAN_COUNT; ++scan) {
    err = mn12832l_spi_scan(scans[scan], false);
    if (err != ESP_OK) {
      break;
    }
  }
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);
  printf("MN12832L SPI dry run: %d scans, %lld us, status=%s\n",
         MN12832L_SCAN_COUNT, (long long)(esp_timer_get_time() - start),
         esp_err_to_name(err));
  return err;
}

static esp_err_t mn12832l_spi_illumination_test_mode(
    const uint8_t *framebuffer, bool reference_phase) {
  if (framebuffer == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  uint8_t scans[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES];
  mn12832l_encode_frame(framebuffer, scans);
  if (reference_phase) {
    // The reference Arduino driver reserves scan slots 0 and 1 for no
    // pixel data. Keep our verified grid bits in place and shift only
    // the first 192 (anode) bits two slots forward. The last two logical
    // columns are not represented in this experimental mode.
    for (int scan = MN12832L_SCAN_COUNT - 1; scan >= 2; --scan) {
      memcpy(scans[scan], scans[scan - 2], 24);
    }
    memset(scans[0], 0, 24);
    memset(scans[1], 0, 24);
  }
  esp_err_t err = mn12832l_spi_init();
  if (err != ESP_OK) {
    return err;
  }

  // Bench-only, time-bounded test: do not run unattended. An independent
  // hardware blanking watchdog will be needed for continuous operation.
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 1);
  vTaskDelay(pdMS_TO_TICKS(2000)); // Verified filament warm-up time.
  gpio_set_level(PIN_HV, 1);

  const int max_cycles = 80;
  int completed = 0;
  int64_t start = esp_timer_get_time();
  for (int cycle = 0; cycle < max_cycles; ++cycle) {
    for (int scan = 0; scan < MN12832L_SCAN_COUNT; ++scan) {
      err = mn12832l_spi_scan(scans[scan], true);
      if (err != ESP_OK) {
        goto shutdown;
      }
    }
    ++completed;
  }

shutdown:
  // Power-off is the only normal exit once HV has been enabled.
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);
  int64_t elapsed = esp_timer_get_time() - start;
  printf("MN12832L SPI [%s]: %d/%d cycles in %lld us (~%.1f Hz), status=%s\n",
         reference_phase ? "reference +2 phase" : "original phase",
         completed, max_cycles, (long long)elapsed,
         elapsed > 0 ? (1000000.0 * completed / elapsed) : 0.0,
         esp_err_to_name(err));
  return err;
}

esp_err_t mn12832l_spi_illumination_test(const uint8_t *framebuffer) {
  return mn12832l_spi_illumination_test_mode(framebuffer, false);
}

esp_err_t mn12832l_spi_phase_shift_test(const uint8_t *framebuffer) {
  return mn12832l_spi_illumination_test_mode(framebuffer, true);
}

/* Walk a single 16-pixel-tall vertical segment through 18 logical columns.
 * The serial log identifies each position so a repeated/missing physical
 * column can be matched to the encoder. This is intentionally bench-only. */
esp_err_t mn12832l_spi_column_walk(void) {
  esp_err_t err = mn12832l_spi_init();
  if (err != ESP_OK) {
    return err;
  }

  enum { WALK_COLUMNS = 18, WALK_HOLD_US = 650000 };
  uint8_t framebuffer[128 * 32 / 8];
  uint8_t scans[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES];

  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 1);
  vTaskDelay(pdMS_TO_TICKS(2000));
  gpio_set_level(PIN_HV, 1);

  printf("MN12832L SPI column walk: X=0..17, 650 ms each; Y=8..23\n");
  for (int x = 0; x < WALK_COLUMNS; ++x) {
    // I1 pixels are active-low. Keep every other pixel off.
    memset(framebuffer, 0xFF, sizeof(framebuffer));
    for (int y = 8; y < 24; ++y) {
      framebuffer[y * 16 + x / 8] &= (uint8_t)~(0x80u >> (x % 8));
    }
    mn12832l_encode_frame(framebuffer, scans);

    printf("COLUMN X=%02d (step %d/%d)\n", x, x + 1, WALK_COLUMNS);
    fflush(stdout);
    int64_t deadline = esp_timer_get_time() + WALK_HOLD_US;
    do {
      for (int scan = 0; scan < MN12832L_SCAN_COUNT; ++scan) {
        err = mn12832l_spi_scan(scans[scan], true);
        if (err != ESP_OK) {
          goto shutdown_walk;
        }
      }
    } while (esp_timer_get_time() < deadline);
  }

shutdown_walk:
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);
  printf("MN12832L SPI column walk finished: %s\n", esp_err_to_name(err));
  return err;
}

/* Independent grid test: one grid bit and only anode A for rows 8..23.
 * Every other multiplex slot remains blank, preserving the low scan duty.
 * This bypasses both overlapping-grid and ABC/DEF encoder assumptions. */
esp_err_t mn12832l_spi_grid_walk(void) {
  esp_err_t err = mn12832l_spi_init();
  if (err != ESP_OK) {
    return err;
  }

  enum { WALK_GRIDS = 12, WALK_HOLD_US = 650000 };
  uint8_t scans[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES];

  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 1);
  vTaskDelay(pdMS_TO_TICKS(2000));
  gpio_set_level(PIN_HV, 1);

  printf("MN12832L grid isolation: grids 1..12, anode A, rows 8..23\n");
  for (int grid = 1; grid <= WALK_GRIDS; ++grid) {
    memset(scans, 0, sizeof(scans));
    uint8_t *active = scans[grid - 1];
    mn12832l_select_grid(active, grid);
    for (int row = 9; row <= 24; ++row) {
      mn12832l_set_anode(active, row, 'A');
    }

    printf("GRID %02d (step %d/%d)\n", grid, grid, WALK_GRIDS);
    fflush(stdout);
    int64_t deadline = esp_timer_get_time() + WALK_HOLD_US;
    do {
      for (int scan = 0; scan < MN12832L_SCAN_COUNT; ++scan) {
        err = mn12832l_spi_scan(scans[scan], scan == grid - 1);
        if (err != ESP_OK) {
          goto shutdown_grid_walk;
        }
      }
    } while (esp_timer_get_time() < deadline);
  }

shutdown_grid_walk:
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);
  printf("MN12832L grid isolation finished: %s\n", esp_err_to_name(err));
  return err;
}

/* A..F at a fixed adjacent grid pair. One anode per row, one illuminated
 * scan per multiplex cycle: keep this a short supervised diagnostic. */
esp_err_t mn12832l_spi_anode_walk(void) {
  esp_err_t err = mn12832l_spi_init();
  if (err != ESP_OK) {
    return err;
  }

  enum { TEST_SCAN = 4, WALK_HOLD_US = 800000 };
  const char anodes[] = {'A', 'B', 'C', 'D', 'E', 'F'};
  uint8_t scans[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES];

  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 1);
  vTaskDelay(pdMS_TO_TICKS(2000));
  gpio_set_level(PIN_HV, 1);

  printf("MN12832L anode isolation: fixed grids 4+5, rows 9..24\n");
  for (int lane = 0; lane < 6; ++lane) {
    // Visible gap between steps to distinguish successive anodes.
    gpio_set_level(PIN_BLK, 1);
    vTaskDelay(pdMS_TO_TICKS(150));

    memset(scans, 0, sizeof(scans));
    uint8_t *active = scans[TEST_SCAN];
    mn12832l_select_grid(active, TEST_SCAN);
    mn12832l_select_grid(active, TEST_SCAN + 1);
    for (int row = 9; row <= 24; ++row) {
      mn12832l_set_anode(active, row, anodes[lane]);
    }

    printf("ANODE %c (step %d/6)\n", anodes[lane], lane + 1);
    fflush(stdout);
    int64_t deadline = esp_timer_get_time() + WALK_HOLD_US;
    do {
      for (int scan = 0; scan < MN12832L_SCAN_COUNT; ++scan) {
        err = mn12832l_spi_scan(scans[scan], scan == TEST_SCAN);
        if (err != ESP_OK) {
          goto shutdown_anode_walk;
        }
      }
    } while (esp_timer_get_time() < deadline);
  }

shutdown_anode_walk:
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);
  printf("MN12832L anode isolation finished: %s\n", esp_err_to_name(err));
  return err;
}

/* Complete the neighboring-grid mapping for C..F. A/B were measured in
 * the previous experiment and can be retested by restoring that version. */
esp_err_t mn12832l_spi_grid_pair_test(void) {
  esp_err_t err = mn12832l_spi_init();
  if (err != ESP_OK) {
    return err;
  }

  enum { TEST_SCAN = 4, HOLD_US = 850000 };
  const char anodes[] = {'C', 'D', 'E', 'F'};
  const char *grid_names[] = {"G4 only", "G5 only", "G4+G5"};
  uint8_t scans[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES];

  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 1);
  vTaskDelay(pdMS_TO_TICKS(2000));
  gpio_set_level(PIN_HV, 1);

  printf("MN12832L grid pair isolation: anodes C/D/E/F, grids 4 and 5\n");
  for (int anode = 0; anode < 4; ++anode) {
    for (int selected = 0; selected < 3; ++selected) {
      gpio_set_level(PIN_BLK, 1);
      vTaskDelay(pdMS_TO_TICKS(150));

      memset(scans, 0, sizeof(scans));
      uint8_t *active = scans[TEST_SCAN];
      if (selected != 1) {
        mn12832l_select_grid(active, 4);
      }
      if (selected != 0) {
        mn12832l_select_grid(active, 5);
      }
      for (int row = 9; row <= 24; ++row) {
        mn12832l_set_anode(active, row, anodes[anode]);
      }

      printf("ANODE %c / %s (step %d/12)\n", anodes[anode],
             grid_names[selected], anode * 3 + selected + 1);
      fflush(stdout);
      int64_t deadline = esp_timer_get_time() + HOLD_US;
      do {
        for (int scan = 0; scan < MN12832L_SCAN_COUNT; ++scan) {
          err = mn12832l_spi_scan(scans[scan], scan == TEST_SCAN);
          if (err != ESP_OK) {
            goto shutdown_grid_pair;
          }
        }
      } while (esp_timer_get_time() < deadline);
    }
  }

shutdown_grid_pair:
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);
  printf("MN12832L grid pair isolation finished: %s\n",
         esp_err_to_name(err));
  return err;
}

/* Cumulative logical-column mapping diagnostic. The active-low I1
 * framebuffer grows one column at a time, then shrinks one column at a time.
 * Uses the existing (as-yet unverified) encoder; unusual physical patterns
 * are precisely what this test is intended to reveal.
 *
 * This is a SUPERVISED, bounded bench test, not continuous display firmware.
 * A wall-clock limit covers both passes even if SPI refresh slows down.
 * BLK remains asserted for every transfer/latch and all exit paths disable
 * HV and EF. No hardware-independent safety interlock is provided.
 */
esp_err_t mn12832l_spi_cumulative_fill_test(void) {
  enum {
    WIDTH = 128,
    FRAMEBUFFER_BYTES = WIDTH * 32 / 8,
    STRIP_Y_START = 8,
    STRIP_Y_END = 24,
    STEP_US = 180000,
    GAP_MS = 250,
    MAX_TEST_US = 55000000
  };
  esp_err_t err = mn12832l_spi_init();
  if (err != ESP_OK) {
    return err;
  }

  uint8_t framebuffer[FRAMEBUFFER_BYTES];
  uint8_t scans[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES];
  int64_t run_start = 0;

  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);
  gpio_set_level(PIN_EF, 1);
  vTaskDelay(pdMS_TO_TICKS(2000));
  gpio_set_level(PIN_HV, 1);
  run_start = esp_timer_get_time();

  printf("MN12832L cumulative fill: 0->128->0 columns, Y=8..23, "
         "180 ms/step; max 55 s HV-on\n");
  fflush(stdout);

  /* Pass 0 adds X=0..127. Pass 1 removes X=127..0.
   * The first step in the second pass starts with 127 columns on. */
  for (int pass = 0; pass < 2; ++pass) {
    if (pass == 1) {
      gpio_set_level(PIN_BLK, 1);
      vTaskDelay(pdMS_TO_TICKS(GAP_MS));
    }

    for (int step = 1; step <= WIDTH; ++step) {
      int lit_columns = (pass == 0) ? step : WIDTH - step;
      memset(framebuffer, 0xFF, sizeof(framebuffer));
      for (int y = STRIP_Y_START; y < STRIP_Y_END; ++y) {
        for (int x = 0; x < lit_columns; ++x) {
          framebuffer[y * 16 + x / 8] &=
              (uint8_t)~(0x80u >> (x % 8));
        }
      }
      mn12832l_encode_frame(framebuffer, scans);
      printf("%s: %3d/%3d logical columns\n",
             pass == 0 ? "FILL " : "ERASE", lit_columns, WIDTH);
      fflush(stdout);

      int64_t step_end = esp_timer_get_time() + STEP_US;
      do {
        if (esp_timer_get_time() - run_start >= MAX_TEST_US) {
          err = ESP_ERR_TIMEOUT;
          goto shutdown_cumulative;
        }
        for (int scan = 0; scan < MN12832L_SCAN_COUNT; ++scan) {
          err = mn12832l_spi_scan(scans[scan], true);
          if (err != ESP_OK) {
            goto shutdown_cumulative;
          }
        }
      } while (esp_timer_get_time() < step_end);
    }
  }

shutdown_cumulative:
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);
  printf("MN12832L cumulative fill complete: %lld ms, status=%s\n",
         (long long)((esp_timer_get_time() - run_start) / 1000),
         esp_err_to_name(err));
  return err;
}

/* For manual-fill debugging: raw 30-byte scan frames in SPI order.
 * Bytes 0..23 are anode bits, 24..29 are grid bits.
 */
static void mn12832l_manual_dump_scan(const uint8_t *frame) {
  for (int b = 0; b < MN12832L_FRAME_BYTES; ++b) {
    printf("%02X", frame[b]);
    if (b != MN12832L_FRAME_BYTES - 1) {
      putchar(b == 23 ? '|' : ' ');
    }
  }
  putchar('\n');
}

static void mn12832l_manual_dump_transition(
    const uint8_t *framebuffer,
    const uint8_t before[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES],
    const uint8_t after[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES],
    int lit_columns) {
  printf("FRAMEBUFFER y=8 x=0..15, 1=ON: ");
  for (int x = 0; x < 16; ++x) {
    unsigned pos = 8 * 16 + x / 8;
    putchar((framebuffer[pos] & (0x80u >> (x % 8))) ? '0' : '1');
  }
  printf("   I1 raw: %02X %02X (0=ON)\n",
         framebuffer[8 * 16], framebuffer[8 * 16 + 1]);

  int changed_scans = 0;
  for (int scan = 0; scan < MN12832L_SCAN_COUNT; ++scan) {
    if (memcmp(before[scan], after[scan], MN12832L_FRAME_BYTES) == 0) {
      continue;
    }
    ++changed_scans;
    printf("SCAN %02d changed at step %d:\n", scan, lit_columns);
    printf("  OLD: ");
    mn12832l_manual_dump_scan(before[scan]);
    printf("  NEW: ");
    mn12832l_manual_dump_scan(after[scan]);
    printf("  XOR: ");
    for (int b = 0; b < MN12832L_FRAME_BYTES; ++b) {
      printf("%02X", before[scan][b] ^ after[scan][b]);
      if (b != MN12832L_FRAME_BYTES - 1) {
        putchar(b == 23 ? '|' : ' ');
      }
    }
    putchar('\n');
  }
  printf("CHANGED SCANS: %d/%d\n", changed_scans, MN12832L_SCAN_COUNT);
  fflush(stdout);
}

/* Electrode map, bypassing the guessed framebuffer encoder entirely.
 * One grid and ONE anode letter in each of rows 9..24 are asserted during
 * a single 250us slot per 44-slot scan cycle. Other slots remain blank.
 * The 16-row stripe makes a physical COLUMN visible for manual mapping.
 *
 * Controls: 1..9 select grids 1..9; n/p next/previous grid (1..12),
 * a..f select the named anode lane; q quits. Every valid input resets
 * the 5-minute inactivity timer. HV shuts down by 15 minutes regardless.
 * No grid is continuously energized while a command is inspected.
 */
esp_err_t mn12832l_spi_electrode_map_test(void) {
  enum {
    FIRST_GRID = 1, LAST_GRID = 12,
    FIRST_ROW = 9, LAST_ROW = 24,
    DWELL_US = 250,
    MAX_ON_US = 900000000,
    MAX_IDLE_US = 300000000
  };
  static const uint8_t blank[MN12832L_FRAME_BYTES] = {0};
  static uint8_t active[MN12832L_FRAME_BYTES];
  const char anodes[] = "ABCDEF";
  esp_err_t err = mn12832l_spi_init();
  if (err != ESP_OK) return err;

  int grid = 1;
  int anode_index = 0;
  uint32_t measured_frames = 0;
  int64_t on_start, last_input, measurement_start;

  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);

  memset(active, 0, sizeof(active));
  mn12832l_select_grid(active, (uint8_t)grid);
  for (int row = FIRST_ROW; row <= LAST_ROW; ++row)
    mn12832l_set_anode(active, (uint8_t)row, anodes[anode_index]);

  printf("MN12832L physical electrode map: ONE grid, ONE anode/row\n");
  printf("1..9=grid; n/p=next/previous grid 1..12; a..f=anode; q=quit\n");
  printf("Map physical column number and brightness for each grid/anode.\n");
  printf("250us dwell once per 44 slots; 5min idle, 15min HV maximum.\n");
  fflush(stdout);

  gpio_set_level(PIN_EF, 1);
  vTaskDelay(pdMS_TO_TICKS(2000));
  gpio_set_level(PIN_HV, 1);
  on_start = esp_timer_get_time();
  last_input = measurement_start = on_start;
  printf("MAP GRID %02d ANODE %c (rows %d..%d)\n",
         grid, anodes[anode_index], FIRST_ROW, LAST_ROW);
  fflush(stdout);

  for (;;) {
    int64_t now = esp_timer_get_time();
    if (now - on_start >= MAX_ON_US || now - last_input >= MAX_IDLE_US) {
      err = ESP_ERR_TIMEOUT;
      printf("Electrode map timeout; shutting down.\n");
      break;
    }
    for (int slot = 0; slot < MN12832L_SCAN_COUNT; ++slot) {
      err = mn12832l_spi_scan_dwell(slot == grid - 1 ? active : blank,
                                    slot == grid - 1, DWELL_US);
      if (err != ESP_OK) goto electrode_map_shutdown;
    }
    /* IDLE task must run; watchdog stays enabled. */
    vTaskDelay(1);
    ++measured_frames;
    int64_t measured_now = esp_timer_get_time();
    if (measured_now - measurement_start >= 2000000) {
      printf("MAP GRID %02d ANODE %c REFRESH %.2f frames/s\n",
             grid, anodes[anode_index],
             (double)measured_frames * 1000000.0 /
                 (double)(measured_now - measurement_start));
      fflush(stdout);
      measured_frames = 0;
      measurement_start = measured_now;
    }

    char key;
    while (read(STDIN_FILENO, &key, 1) == 1) {
      if (key == 'q' || key == 'Q') {
        printf("Quit requested.\n");
        goto electrode_map_shutdown;
      }
      int next_grid = grid;
      int next_anode = anode_index;
      if (key >= '1' && key <= '9') {
        next_grid = key - '0';
      } else if (key == 'n' || key == 'N' || key == ' ') {
        next_grid = grid == LAST_GRID ? FIRST_GRID : grid + 1;
      } else if (key == 'p' || key == 'P') {
        next_grid = grid == FIRST_GRID ? LAST_GRID : grid - 1;
      } else if (key >= 'a' && key <= 'f') {
        next_anode = key - 'a';
      } else if (key >= 'A' && key <= 'F') {
        next_anode = key - 'A';
      } else {
        continue;
      }
      gpio_set_level(PIN_BLK, 1);
      grid = next_grid;
      anode_index = next_anode;
      memset(active, 0, sizeof(active));
      mn12832l_select_grid(active, (uint8_t)grid);
      for (int row = FIRST_ROW; row <= LAST_ROW; ++row)
        mn12832l_set_anode(active, (uint8_t)row, anodes[anode_index]);

      last_input = esp_timer_get_time();
      measurement_start = last_input;
      measured_frames = 0;
      printf("MAP GRID %02d ANODE %c (rows %d..%d)\n",
             grid, anodes[anode_index], FIRST_ROW, LAST_ROW);
      fflush(stdout);
      break;
    }
  }

electrode_map_shutdown:
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);
  printf("MN12832L electrode map ended: %s\n", esp_err_to_name(err));
  return err;
}

/* Controlled brightness test for two independently verified physical columns:
 * 10 = G4/D versus G5/D
 * 12 = G4/F versus G5/F
 *
 * Only one grid and one anode per row are enabled. The active slot always
 * receives exactly 250us of illumination; a selectable post-latch BLK guard
 * (0, 20, or 200us) is applied while outputs are blanked.
 * The other 43 SPI slots remain blank. This is NOT a continuous electrode hold.
 * Use equal camera exposure when comparing brightness; measured frame rate is
 * printed because increasing guard slightly affects overall refresh rate.
 */
esp_err_t mn12832l_spi_brightness_comparison_test(void) {
  enum {
    ROW_START = 9, ROW_END = 24,
    DWELL_US = 250,
    MAX_IDLE_US = 300000000,
    MAX_ON_US = 900000000
  };
  static const uint8_t blank[MN12832L_FRAME_BYTES] = {0};
  static uint8_t active[MN12832L_FRAME_BYTES];
  static const uint32_t guard_us_values[] = {0, 20, 200};
  esp_err_t err = mn12832l_spi_init();
  if (err != ESP_OK) return err;

  int physical_column = 11;
  int grid = 4;
  int guard_index = 0;
  uint32_t measured_frames = 0;
  int64_t on_start, last_input, measurement_start;

  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);

  memset(active, 0, sizeof(active));
  mn12832l_select_grid(active, (uint8_t)grid);
  for (int row = ROW_START; row <= ROW_END; ++row) {
    mn12832l_set_anode(active, (uint8_t)row, 'E');
  }

  printf("MN12832L brightness: physical 10 (D), 11 (E), 12 (F)\n");
  printf("Keys: 1=col10; 2=col12; 3=col11; l=G4; r=G5; g=guard; q=quit\n");
  printf("Guards: 0, 20, 200us. Active dwell fixed at 250us.\n");
  printf("Auto-off: 5min idle, 15min HV maximum.\n");
  fflush(stdout);

  gpio_set_level(PIN_EF, 1);
  vTaskDelay(pdMS_TO_TICKS(2000));
  gpio_set_level(PIN_HV, 1);
  on_start = esp_timer_get_time();
  last_input = measurement_start = on_start;
  printf("BRIGHTNESS COL %d G%d/%c, POST-LATCH GUARD %luus\n",
         physical_column, grid, 'E',
         (unsigned long)guard_us_values[guard_index]);
  fflush(stdout);

  for (;;) {
    int64_t now = esp_timer_get_time();
    if (now - on_start >= MAX_ON_US || now - last_input >= MAX_IDLE_US) {
      err = ESP_ERR_TIMEOUT;
      printf("Brightness comparison timed out; powering down.\n");
      break;
    }

    for (int slot = 0; slot < MN12832L_SCAN_COUNT; ++slot) {
      const bool selected = slot == grid - 1;
      if (!selected) {
        err = mn12832l_spi_scan_dwell(blank, false, 0);
      } else {
        /* Keep BLK asserted through SPI and latch, then optionally
         * wait before enabling the 250us illumination pulse. */
        gpio_set_level(PIN_BLK, 1);
        err = mn12832l_spi_shift(active);
        if (err != ESP_OK) goto brightness_shutdown;
        gpio_set_level(PIN_LAT, 1);
        esp_rom_delay_us(1);
        gpio_set_level(PIN_LAT, 0);
        if (guard_us_values[guard_index] > 0)
          esp_rom_delay_us(guard_us_values[guard_index]);
        gpio_set_level(PIN_BLK, 0);
        esp_rom_delay_us(DWELL_US);
        gpio_set_level(PIN_BLK, 1);
      }
      if (err != ESP_OK) goto brightness_shutdown;
    }
    vTaskDelay(1); /* Keep IDLE0 active: don't bypass the task watchdog. */

    ++measured_frames;
    int64_t measure_now = esp_timer_get_time();
    if (measure_now - measurement_start >= 2000000) {
      printf("COL %d G%d/%c GUARD %luus: %.2f frames/s\n",
             physical_column, grid, physical_column == 10 ? 'D' : (physical_column == 11 ? 'E' : 'F'),
             (unsigned long)guard_us_values[guard_index],
             (double)measured_frames * 1000000.0 /
                 (double)(measure_now - measurement_start));
      fflush(stdout);
      measurement_start = measure_now;
      measured_frames = 0;
    }

    char key;
    while (read(STDIN_FILENO, &key, 1) == 1) {
      if (key == 'q' || key == 'Q') {
        printf("Quit requested.\n");
        goto brightness_shutdown;
      }
      int new_column = physical_column;
      int new_grid = grid;
      int new_guard = guard_index;
      if (key == '1') new_column = 10;
      else if (key == '2') new_column = 12;
      else if (key == '3') new_column = 11;
      else if (key == 'l' || key == 'L') new_grid = 4;
      else if (key == 'r' || key == 'R') new_grid = 5;
      else if (key == 'g' || key == 'G') new_guard = (guard_index + 1) % 3;
      else continue;

      gpio_set_level(PIN_BLK, 1);
      physical_column = new_column;
      grid = new_grid;
      guard_index = new_guard;
      memset(active, 0, sizeof(active));
      mn12832l_select_grid(active, (uint8_t)grid);
      for (int row = ROW_START; row <= ROW_END; ++row)
        mn12832l_set_anode(active, (uint8_t)row,
                           physical_column == 10 ? 'D' : (physical_column == 11 ? 'E' : 'F'));

      last_input = esp_timer_get_time();
      measurement_start = last_input;
      measured_frames = 0;
      printf("BRIGHTNESS COL %d G%d/%c, POST-LATCH GUARD %luus\n",
             physical_column, grid, physical_column == 10 ? 'D' : (physical_column == 11 ? 'E' : 'F'),
             (unsigned long)guard_us_values[guard_index]);
      fflush(stdout);
      break;
    }
  }

brightness_shutdown:
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);
  printf("MN12832L brightness comparison ended: %s\n",
         esp_err_to_name(err));
  return err;
}

/* EXPERIMENTAL first-24-column single-grid renderer.
 *
 * The map is 1-based and deliberately explicit. Mapping was measured for
 * several entries and inferred by the repeating three-column pattern for
 * others. Selection favors the BRIGHTER measured grid when known; columns
 * without a measured brightness preference are provisional.
 *
 * Each logical pixel is assigned to only one grid/anode electrode. Frames
 * for other grids are fully blank. No pair of grids is ever active together.
 * The existing mn12832l_encode_frame() remains untouched for comparison.
 */
typedef struct {
  uint8_t grid;
  char anode;
} mn12832l_physical_column_t;

static const mn12832l_physical_column_t mn12832l_first24_map[24] = {
    {2, 'A'}, {2, 'B'}, {2, 'C'}, /* 1..3, G2 measured */
    {2, 'D'}, {3, 'E'}, {3, 'F'}, /* 4..6, G3 E/F stronger */
    {3, 'A'}, {4, 'B'}, {4, 'C'}, /* 7..9, G4 C unverified */
    {4, 'D'}, {5, 'E'}, {5, 'F'}, /* 10..12, preferred grids measured */
    {5, 'A'}, {5, 'B'}, {6, 'C'}, /* 13..15, G6 C unverified */
    {6, 'D'}, {6, 'E'}, {7, 'F'}, /* 16..18, G7/F verified bright */
    {7, 'A'}, {7, 'B'}, {8, 'C'}, /* 19..21, G8/C verified bright */
    {8, 'D'}, {8, 'E'}, {9, 'F'}  /* 22..24, G9/F verified bright */
};

/* Build a 16-row stripe for requested columns, using raw electrode bits,
 * not the old logical x/3 and ABC/DEF alternating encoder. */
static void mn12832l_encode_first24_pattern(
    uint32_t column_mask,
    uint8_t scans[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES],
    bool active_slots[MN12832L_SCAN_COUNT]) {
  memset(scans, 0,
         MN12832L_SCAN_COUNT * MN12832L_FRAME_BYTES);
  memset(active_slots, 0, MN12832L_SCAN_COUNT * sizeof(active_slots[0]));

  for (int x = 0; x < 24; ++x) {
    if ((column_mask & (1u << x)) == 0) continue;
    const mn12832l_physical_column_t *mapping = &mn12832l_first24_map[x];
    const int slot = mapping->grid - 1;
    uint8_t *scan = scans[slot];
    mn12832l_select_grid(scan, mapping->grid);
    active_slots[slot] = true;
    for (int row = 9; row <= 24; ++row)
      mn12832l_set_anode(scan, (uint8_t)row, mapping->anode);
  }
}

esp_err_t mn12832l_spi_first24_corrected_test(void) {
  enum {
    SLOT_US = 250,
    MAX_IDLE_US = 300000000,
    MAX_ON_US = 900000000
  };
  static uint8_t scans[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES];
  static bool active_slots[MN12832L_SCAN_COUNT];
  static const uint8_t blank[MN12832L_FRAME_BYTES] = {0};
  static const uint32_t masks[5] = {
      0x00ffffffu, /* 1: all columns 1..24 */
      0x00555555u, /* 2: odd columns */
      0x00aaaaaau, /* 3: even columns */
      (1u << 7) | (1u << 15) | (1u << 23), /* 4: 8,16,24 */
      0u /* 5: individually selected column */
  };
  static const char *pattern_names[5] = {
      "ALL 1..24", "ODD", "EVEN", "8,16,24", "SINGLE"
  };

  esp_err_t err = mn12832l_spi_init();
  if (err != ESP_OK) return err;
  int pattern = 4;
  int single_col = 1;
  uint32_t measured_frames = 0;
  int64_t on_start, last_input, measurement_start;

  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);
  mn12832l_encode_first24_pattern(1u, scans, active_slots);

  printf("MN12832L EXPERIMENTAL single-grid encoder, physical columns 1..24\n");
  printf("Keys: 6=col10, 7=col11, 8=col12, 9=all three; 1=all24, 2=odd, 3=even, 4=8+16+24, 5=single\n");
  printf("n/p=step one column; s=restart at col1; 0=off; q=quit\n");
  printf("Record brightness for 1..24 in groups of three (middle has black stripe).\n");
  printf("Provisional map in firmware; 250us per scan slot; 5min idle/15min HV maximum\n");
  fflush(stdout);

  gpio_set_level(PIN_EF, 1);
  vTaskDelay(pdMS_TO_TICKS(2000));
  gpio_set_level(PIN_HV, 1);
  on_start = esp_timer_get_time();
  last_input = measurement_start = on_start;
  printf("SURVEY COL 01 (group 1, phase 1/3): G2/A\n");
  fflush(stdout);

  for (;;) {
    int64_t now = esp_timer_get_time();
    if (now - on_start >= MAX_ON_US || now - last_input >= MAX_IDLE_US) {
      err = ESP_ERR_TIMEOUT;
      printf("First-24 experiment timed out; powering down.\n");
      break;
    }

    for (int slot = 0; slot < MN12832L_SCAN_COUNT; ++slot) {
      /* Constant slot duration regardless of pattern density. */
      if (active_slots[slot]) {
        err = mn12832l_spi_scan_dwell(scans[slot], true, SLOT_US);
      } else {
        err = mn12832l_spi_scan_dwell(blank, false, 0);
        if (err == ESP_OK) esp_rom_delay_us(SLOT_US);
      }
      if (err != ESP_OK) goto first24_shutdown;
    }
    vTaskDelay(1);

    ++measured_frames;
    int64_t measure_now = esp_timer_get_time();
    if (measure_now - measurement_start >= 2000000) {
      printf("FIRST24 %s COL %d: %.2f frames/s\n",
             pattern_names[pattern], single_col,
             (double)measured_frames * 1000000.0 /
                 (double)(measure_now - measurement_start));
      fflush(stdout);
      measurement_start = measure_now;
      measured_frames = 0;
    }

    char key;
    while (read(STDIN_FILENO, &key, 1) == 1) {
      if (key == 'q' || key == 'Q') {
        printf("Quit requested.\n");
        goto first24_shutdown;
      }
      bool accepted = true;
      bool force_off = false;
      bool group_10_12 = false;
      if (key >= '1' && key <= '5') {
        pattern = key - '1';
      } else if (key >= '6' && key <= '8') {
        pattern = 4;
        single_col = 10 + (key - '6');
      } else if (key == '9') {
        group_10_12 = true;
      } else if (key == 'n' || key == 'N' || key == ' ') {
        pattern = 4;
        single_col = single_col == 24 ? 1 : single_col + 1;
      } else if (key == 'p' || key == 'P') {
        pattern = 4;
        single_col = single_col == 1 ? 24 : single_col - 1;
      } else if (key == 's' || key == 'S') {
        pattern = 4;
        single_col = 1;
      } else if (key == '0') {
        force_off = true;
      } else {
        accepted = false;
      }
      if (!accepted) continue;
      gpio_set_level(PIN_BLK, 1);
      uint32_t column_mask = pattern == 4
          ? (1u << (single_col - 1)) : masks[pattern];
      if (group_10_12) column_mask = (1u << 9) | (1u << 10) | (1u << 11);
      if (force_off) column_mask = 0;
      mn12832l_encode_first24_pattern(column_mask, scans, active_slots);
      last_input = esp_timer_get_time();
      measurement_start = last_input;
      measured_frames = 0;
      if (force_off) {
        printf("FIRST24 ALL OFF\n");
      } else if (group_10_12) {
        printf("FIRST24 10+11+12: G4/D + G5/E,F\n");
      } else if (pattern == 4) {
        const mn12832l_physical_column_t *m =
            &mn12832l_first24_map[single_col - 1];
        printf("SURVEY COL %02d (group %d, phase %d/3): G%d/%c\n",
               single_col, (single_col - 1) / 3 + 1,
               (single_col - 1) % 3 + 1, m->grid, m->anode);
      } else {
        printf("FIRST24 PATTERN %d: %s\n",
               pattern + 1, pattern_names[pattern]);
      }
      fflush(stdout);
      break;
    }
  }

first24_shutdown:
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);
  printf("MN12832L first-24 experiment ended: %s\n",
         esp_err_to_name(err));
  return err;
}

/* Independence test for one measured three-column group.
 * All displayed columns use their preferred independently measured grid:
 * col4=G2/D, col5=G3/E, col6=G3/F. Anodes for columns 5 and 6
 * coexist in a single G3 scan; col4 occupies the G2 scan.
 * Hold timing and refresh slots constant across every pattern.
 */
esp_err_t mn12832l_spi_group_independence_test(void) {
  enum { SLOT_US = 250, MAX_IDLE_US = 300000000,
         MAX_ON_US = 900000000 };
  static const uint8_t blank[MN12832L_FRAME_BYTES] = {0};
  static uint8_t scans[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES];
  static bool active_slots[MN12832L_SCAN_COUNT];
  static const uint32_t patterns[] = {
      1u << 3,                          /* 1: col 4 */
      1u << 4,                          /* 2: col 5 */
      1u << 5,                          /* 3: col 6 */
      (1u << 3) | (1u << 4),            /* 4: 4+5 */
      (1u << 4) | (1u << 5),            /* 5: 5+6 */
      (1u << 3) | (1u << 5),            /* 6: 4+6 */
      (1u << 3) | (1u << 4) | (1u << 5),/* 7: 4+5+6 */
      0u                                /* 0: blank */
  };
  static const char *names[] = {
      "4 only", "5 only", "6 only", "4+5", "5+6", "4+6",
      "4+5+6", "blank"
  };
  esp_err_t err = mn12832l_spi_init();
  if (err != ESP_OK) return err;
  int selected = 0;
  int64_t on_start, last_input;
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);
  mn12832l_encode_first24_pattern(patterns[selected], scans, active_slots);
  printf("MN12832L column independence: preferred G2/D, G3/E, G3/F\n");
  printf("Numbered patterns: 1=4, 2=5, 3=6, 4=4+5, 5=5+6, 6=4+6, 7=4+5+6\n");
  printf("0=off, q=quit. Start at #1. Report positions and stray lines.\n");
  printf("Fixed 250us scan slot; auto-off 5min idle / 15min cumulative HV.\n");
  fflush(stdout);
  gpio_set_level(PIN_EF, 1);
  vTaskDelay(pdMS_TO_TICKS(2000));
  gpio_set_level(PIN_HV, 1);
  on_start = last_input = esp_timer_get_time();
  printf("PATTERN #1: %s\n", names[selected]);
  fflush(stdout);

  for (;;) {
    const int64_t now = esp_timer_get_time();
    if (now - on_start >= MAX_ON_US || now - last_input >= MAX_IDLE_US) {
      err = ESP_ERR_TIMEOUT;
      printf("Independence test timeout; powering down.\n");
      break;
    }
    for (int slot = 0; slot < MN12832L_SCAN_COUNT; ++slot) {
      if (active_slots[slot]) {
        err = mn12832l_spi_scan_dwell(scans[slot], true, SLOT_US);
      } else {
        err = mn12832l_spi_scan_dwell(blank, false, 0);
        if (err == ESP_OK) esp_rom_delay_us(SLOT_US);
      }
      if (err != ESP_OK) goto group_independence_shutdown;
    }
    vTaskDelay(1);

    char key;
    while (read(STDIN_FILENO, &key, 1) == 1) {
      if (key == 'q' || key == 'Q') {
        printf("Quit requested.\n");
        goto group_independence_shutdown;
      }
      int next;
      if (key >= '1' && key <= '7') next = key - '1';
      else if (key == '0') next = 7;
      else continue;
      gpio_set_level(PIN_BLK, 1);
      selected = next;
      mn12832l_encode_first24_pattern(patterns[selected], scans,
                                       active_slots);
      last_input = esp_timer_get_time();
      printf("PATTERN #%c: %s\n", key, names[selected]);
      fflush(stdout);
      break;
    }
  }
group_independence_shutdown:
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);
  printf("MN12832L independence test ended: %s\n", esp_err_to_name(err));
  return err;
}

/* Targeted absent-column check.
 *
 * For each requested physical column, use the same anode and compare the
 * two adjacent grids independently: 18=G6/F or G7/F,
 * 21=G7/C or G8/C, 24=G8/F or G9/F.
 *
 * These are candidate physical addresses from the measured/inferred weave;
 * no grid preference is assumed. Keep one 250us active slot per 44-scan
 * frame, regardless of grid, with other slots fully blank and time-matched.
 * No continuous-on electrode drive. No encoder mapping changed here.
 */
esp_err_t mn12832l_spi_missing_columns_test(void) {
  enum {
    SLOT_US = 250,
    MAX_IDLE_US = 300000000,
    MAX_ON_US = 900000000
  };
  typedef struct {
    uint8_t column;
    uint8_t left_grid;
    uint8_t right_grid;
    char anode;
  } candidate_t;
  static const candidate_t candidates[] = {
      {18, 6, 7, 'F'},
      {21, 7, 8, 'C'},
      {24, 8, 9, 'F'}
  };
  static const uint8_t blank[MN12832L_FRAME_BYTES] = {0};
  static uint8_t active[MN12832L_FRAME_BYTES];
  esp_err_t err = mn12832l_spi_init();
  if (err != ESP_OK) return err;

  int selected = 0; /* Column 18, the first missing position. */
  bool use_right_grid = false;
  uint32_t frame_count = 0;
  int64_t on_start, last_input, measurement_start;

  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);

  memset(active, 0, sizeof(active));
  mn12832l_select_grid(active, candidates[selected].left_grid);
  for (int row = 9; row <= 24; ++row)
    mn12832l_set_anode(active, (uint8_t)row, candidates[selected].anode);

  printf("MN12832L missing-column grid comparison (fixed 250us/slot)\n");
  printf("1=col18 F, 2=col21 C, 3=col24 F; L=lower grid, R=higher grid; Q=quit\n");
  printf("Only one grid is active per frame. Auto-off: 5min idle, 15min HV maximum.\n");
  fflush(stdout);

  gpio_set_level(PIN_EF, 1);
  vTaskDelay(pdMS_TO_TICKS(2000));
  gpio_set_level(PIN_HV, 1);
  on_start = esp_timer_get_time();
  last_input = measurement_start = on_start;
  printf("TEST COL 18: G6/F (L); R selects G7/F\n");
  fflush(stdout);

  for (;;) {
    const int64_t now = esp_timer_get_time();
    if (now - on_start >= MAX_ON_US || now - last_input >= MAX_IDLE_US) {
      err = ESP_ERR_TIMEOUT;
      printf("Missing-column test timeout; powering down.\n");
      break;
    }

    const candidate_t *candidate = &candidates[selected];
    const int grid = use_right_grid ? candidate->right_grid
                                    : candidate->left_grid;
    for (int slot = 0; slot < MN12832L_SCAN_COUNT; ++slot) {
      bool active_slot = slot == grid - 1;
      if (active_slot) {
        err = mn12832l_spi_scan_dwell(active, true, SLOT_US);
      } else {
        err = mn12832l_spi_scan_dwell(blank, false, 0);
        if (err == ESP_OK) esp_rom_delay_us(SLOT_US);
      }
      if (err != ESP_OK) goto missing_columns_shutdown;
    }
    vTaskDelay(1); /* Allow IDLE0; preserve task watchdog. */

    ++frame_count;
    const int64_t measured_now = esp_timer_get_time();
    if (measured_now - measurement_start >= 2000000) {
      printf("COL %u G%d/%c (%c): %.2f frames/s\n",
             (unsigned)candidate->column, grid, candidate->anode,
             use_right_grid ? 'R' : 'L',
             (double)frame_count * 1000000.0 /
                 (double)(measured_now - measurement_start));
      fflush(stdout);
      frame_count = 0;
      measurement_start = measured_now;
    }

    char key;
    while (read(STDIN_FILENO, &key, 1) == 1) {
      if (key == 'q' || key == 'Q') {
        printf("Quit requested.\n");
        goto missing_columns_shutdown;
      }
      if (key >= '1' && key <= '3') {
        selected = key - '1';
      } else if (key == 'l' || key == 'L') {
        use_right_grid = false;
      } else if (key == 'r' || key == 'R') {
        use_right_grid = true;
      } else {
        continue;
      }

      gpio_set_level(PIN_BLK, 1);
      candidate = &candidates[selected];
      const uint8_t selected_grid = use_right_grid
          ? candidate->right_grid : candidate->left_grid;
      memset(active, 0, sizeof(active));
      mn12832l_select_grid(active, selected_grid);
      for (int row = 9; row <= 24; ++row)
        mn12832l_set_anode(active, (uint8_t)row, candidate->anode);

      last_input = esp_timer_get_time();
      frame_count = 0;
      measurement_start = last_input;
      printf("TEST COL %u G%u/%c (%c), alternate G%u\n",
             (unsigned)candidate->column, (unsigned)selected_grid,
             candidate->anode, use_right_grid ? 'R' : 'L',
             (unsigned)(use_right_grid ? candidate->left_grid
                                       : candidate->right_grid));
      fflush(stdout);
      break;
    }
  }

missing_columns_shutdown:
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);
  printf("MN12832L missing-column comparison ended: %s\n",
         esp_err_to_name(err));
  return err;
}

/* Targeted absent-column check.
 *
 * For each requested physical column, use the same anode and compare the
 * two adjacent grids independently: 10=G4/D or G5/D,
 * 11=G4/E or G5/E, 12=G4/F or G5/F.
 *
 * These are candidate physical addresses from the measured/inferred weave;
 * no grid preference is assumed. Keep one 250us active slot per 44-scan
 * frame, regardless of grid, with other slots fully blank and time-matched.
 * No continuous-on electrode drive. No encoder mapping changed here.
 */
esp_err_t mn12832l_spi_three_column_grid_test(void) {
  enum {
    SLOT_US = 250,
    MAX_IDLE_US = 300000000,
    MAX_ON_US = 900000000
  };
  typedef struct {
    uint8_t column;
    uint8_t left_grid;
    uint8_t right_grid;
    char anode;
  } candidate_t;
  static const candidate_t candidates[] = {
      {10, 4, 5, 'D'},
      {11, 4, 5, 'E'},
      {12, 4, 5, 'F'}
  };
  static const uint8_t blank[MN12832L_FRAME_BYTES] = {0};
  static uint8_t active[MN12832L_FRAME_BYTES];
  esp_err_t err = mn12832l_spi_init();
  if (err != ESP_OK) return err;

  int selected = 0; /* Physical column 10. */
  bool use_right_grid = false;
  uint32_t frame_count = 0;
  int64_t on_start, last_input, measurement_start;

  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);

  memset(active, 0, sizeof(active));
  mn12832l_select_grid(active, candidates[selected].left_grid);
  for (int row = 9; row <= 24; ++row)
    mn12832l_set_anode(active, (uint8_t)row, candidates[selected].anode);

  printf("MN12832L three-column grid comparison (fixed 250us/slot)\n");
  printf("1=col10 D, 2=col11 E, 3=col12 F; L=lower grid, R=higher grid; Q=quit\n");
  printf("Only one grid is active per frame. Auto-off: 5min idle, 15min HV maximum.\n");
  fflush(stdout);

  gpio_set_level(PIN_EF, 1);
  vTaskDelay(pdMS_TO_TICKS(2000));
  gpio_set_level(PIN_HV, 1);
  on_start = esp_timer_get_time();
  last_input = measurement_start = on_start;
  printf("TEST COL 10: G4/D (L); R selects G5/D\n");
  fflush(stdout);

  for (;;) {
    const int64_t now = esp_timer_get_time();
    if (now - on_start >= MAX_ON_US || now - last_input >= MAX_IDLE_US) {
      err = ESP_ERR_TIMEOUT;
      printf("Three-column test timeout; powering down.\n");
      break;
    }

    const candidate_t *candidate = &candidates[selected];
    const int grid = use_right_grid ? candidate->right_grid
                                    : candidate->left_grid;
    for (int slot = 0; slot < MN12832L_SCAN_COUNT; ++slot) {
      bool active_slot = slot == grid - 1;
      if (active_slot) {
        err = mn12832l_spi_scan_dwell(active, true, SLOT_US);
      } else {
        err = mn12832l_spi_scan_dwell(blank, false, 0);
        if (err == ESP_OK) esp_rom_delay_us(SLOT_US);
      }
      if (err != ESP_OK) goto three_column_grid_shutdown;
    }
    vTaskDelay(1); /* Allow IDLE0; preserve task watchdog. */

    ++frame_count;
    const int64_t measured_now = esp_timer_get_time();
    if (measured_now - measurement_start >= 2000000) {
      printf("COL %u G%d/%c (%c): %.2f frames/s\n",
             (unsigned)candidate->column, grid, candidate->anode,
             use_right_grid ? 'R' : 'L',
             (double)frame_count * 1000000.0 /
                 (double)(measured_now - measurement_start));
      fflush(stdout);
      frame_count = 0;
      measurement_start = measured_now;
    }

    char key;
    while (read(STDIN_FILENO, &key, 1) == 1) {
      if (key == 'q' || key == 'Q') {
        printf("Quit requested.\n");
        goto three_column_grid_shutdown;
      }
      if (key >= '1' && key <= '3') {
        selected = key - '1';
      } else if (key == 'l' || key == 'L') {
        use_right_grid = false;
      } else if (key == 'r' || key == 'R') {
        use_right_grid = true;
      } else {
        continue;
      }

      gpio_set_level(PIN_BLK, 1);
      candidate = &candidates[selected];
      const uint8_t selected_grid = use_right_grid
          ? candidate->right_grid : candidate->left_grid;
      memset(active, 0, sizeof(active));
      mn12832l_select_grid(active, selected_grid);
      for (int row = 9; row <= 24; ++row)
        mn12832l_set_anode(active, (uint8_t)row, candidate->anode);

      last_input = esp_timer_get_time();
      frame_count = 0;
      measurement_start = last_input;
      printf("TEST COL %u G%u/%c (%c), alternate G%u\n",
             (unsigned)candidate->column, (unsigned)selected_grid,
             candidate->anode, use_right_grid ? 'R' : 'L',
             (unsigned)(use_right_grid ? candidate->left_grid
                                       : candidate->right_grid));
      fflush(stdout);
      break;
    }
  }

three_column_grid_shutdown:
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);
  printf("MN12832L three-column comparison ended: %s\n",
         esp_err_to_name(err));
  return err;
}

/* A/B/C/D grid-phase diagnostic. All modes do two SPI transfers per scan
 * with two 250us slots:
 * A = paired grids / blank
 * B = previous grid / next grid
 * C = previous grid / blank
 * D = next grid / blank.
 * For scan 0, only G1 exists: B and D select G1, C stays blank.
 * C/D are intentionally less luminous than B; compare position first.
 */
static esp_err_t mn12832l_manual_timed_scan(
    const uint8_t frame[MN12832L_FRAME_BYTES], int scan, char mode) {
  enum { PHASE_US = 250 };
  static const uint8_t blank[MN12832L_FRAME_BYTES] = {0};
  esp_err_t err;
  if (mode == 'A') {
    err = mn12832l_spi_scan_dwell(frame, true, PHASE_US);
    if (err != ESP_OK) return err;
    err = mn12832l_spi_scan_dwell(blank, false, 0);
    if (err != ESP_OK) return err;
    esp_rom_delay_us(PHASE_US);
    return ESP_OK;
  }

  if (mode != 'B' && mode != 'C' && mode != 'D') {
    return ESP_ERR_INVALID_ARG;
  }
  uint8_t isolated[MN12832L_FRAME_BYTES];
  memcpy(isolated, frame, sizeof(isolated));
  memset(isolated + 24, 0, 6);
  bool first_active = false;
  if (mode == 'D' || (mode == 'B' && scan == 0)) {
    mn12832l_select_grid(isolated, (uint8_t)(scan + 1));
    first_active = true;
  } else if (scan > 0) {
    mn12832l_select_grid(isolated, (uint8_t)scan);
    first_active = true;
  }

  err = mn12832l_spi_scan_dwell(isolated, first_active, PHASE_US);
  if (err != ESP_OK) return err;

  if (mode == 'B' && scan > 0) {
    memset(isolated + 24, 0, 6);
    mn12832l_select_grid(isolated, (uint8_t)(scan + 1));
    return mn12832l_spi_scan_dwell(isolated, true, PHASE_US);
  }
  err = mn12832l_spi_scan_dwell(blank, false, 0);
  if (err != ESP_OK) return err;
  esp_rom_delay_us(PHASE_US);
  return ESP_OK;
}

/* Independent single-column A/B comparison for logical columns 4..9.
 * Numeric keys 4..9 select ONLY that column (1-based); n/p walk the range,
 * 0 blanks the pattern; a/b toggle paired vs individually driven grids.
 *
 * Keep the current mapping/scan driver unchanged, so this isolates the
 * influence of one logical-column request without cumulative interactions.
 * Always multiplex full frames and automatically shut off after timeouts.
 */
esp_err_t mn12832l_spi_single_column_ab_test(void) {
  enum {
    FIRST_COLUMN = 4,
    LAST_COLUMN = 9,
    ROW_START = 8,
    ROW_END = 24,
    MAX_ON_US = 900000000,
    MAX_IDLE_US = 300000000
  };
  esp_err_t err = mn12832l_spi_init();
  if (err != ESP_OK) {
    return err;
  }

  /* Keep diagnostic data off the relatively small app_main task stack. */
  static uint8_t framebuffer[128 * 32 / 8];
  static uint8_t scans[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES];
  int selected_column = FIRST_COLUMN; // One-based human-readable column.
  char grid_mode = 'A';
  uint32_t measured_frames = 0;
  int64_t measurement_start = 0;
  int64_t on_start = 0;
  int64_t last_input = 0;

  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);

  memset(framebuffer, 0xFF, sizeof(framebuffer));
  for (int y = ROW_START; y < ROW_END; ++y) {
    int x = selected_column - 1;
    framebuffer[y * 16 + x / 8] &= (uint8_t)~(0x80u >> (x % 8));
  }
  mn12832l_encode_frame(framebuffer, scans);

  printf("MN12832L single-column A/B/C/D: columns 4..9, Y=8..23\n");
  printf("Keys: 4..9=select only that column; n=next; p=previous; "
         "0=blank; a=paired; b=split; c=Gx; d=Gx+1; q=quit\n");
  printf("A=paired/blank, B=Gx/Gx+1, C=Gx/blank, D=Gx+1/blank.\n");
  printf("Auto-off: 5min without command, 15min maximum HV-on.\n");
  fflush(stdout);

  gpio_set_level(PIN_EF, 1);
  vTaskDelay(pdMS_TO_TICKS(2000));
  gpio_set_level(PIN_HV, 1);
  on_start = esp_timer_get_time();
  last_input = on_start;
  measurement_start = on_start;
  printf("SELECTED COLUMN %d ONLY, MODE A\n", selected_column);
  fflush(stdout);

  for (;;) {
    int64_t now = esp_timer_get_time();
    if (now - on_start >= MAX_ON_US || now - last_input >= MAX_IDLE_US) {
      err = ESP_ERR_TIMEOUT;
      printf("Single-column test timed out, shutting down.\n");
      break;
    }

    for (int scan = 0; scan < MN12832L_SCAN_COUNT; ++scan) {
      err = mn12832l_manual_timed_scan(scans[scan], scan, grid_mode);
      if (err != ESP_OK) {
        goto shutdown_single_ab;
      }
    }
    /* Give IDLE0 an opportunity to run; do not mask the watchdog. */
    vTaskDelay(1);

    ++measured_frames;
    int64_t measure_now = esp_timer_get_time();
    if (measure_now - measurement_start >= 2000000) {
      printf("REFRESH MODE %c COLUMN %d: %.2f frames/s\n",
             grid_mode, selected_column,
             (double)measured_frames * 1000000.0 /
                 (double)(measure_now - measurement_start));
      fflush(stdout);
      measured_frames = 0;
      measurement_start = measure_now;
    }

    char key;
    while (read(STDIN_FILENO, &key, 1) == 1) {
      if (key == 'q' || key == 'Q') {
        printf("Quit requested.\n");
        goto shutdown_single_ab;
      }
      if (key == 'a' || key == 'A' || key == 'b' || key == 'B' ||
          key == 'c' || key == 'C' || key == 'd' || key == 'D') {
        grid_mode = (key >= 'a' && key <= 'z') ? key - ('a' - 'A') : key;
        last_input = esp_timer_get_time();
        measurement_start = last_input;
        measured_frames = 0;
        gpio_set_level(PIN_BLK, 1);
        printf("MODE %c, COLUMN %d ONLY\n",
               grid_mode, selected_column);
        fflush(stdout);
        break;
      }

      int next_column = selected_column;
      if (key >= '4' && key <= '9') {
        next_column = key - '0';
      } else if (key == 'n' || key == 'N' || key == ' ') {
        next_column = selected_column == LAST_COLUMN
                          ? FIRST_COLUMN
                          : (selected_column == 0 ? FIRST_COLUMN
                                                  : selected_column + 1);
      } else if (key == 'p' || key == 'P') {
        next_column = selected_column <= FIRST_COLUMN
                          ? LAST_COLUMN
                          : selected_column - 1;
      } else if (key == '0') {
        next_column = 0; // All columns off.
      } else {
        continue;
      }
      last_input = esp_timer_get_time();
      selected_column = next_column;
      gpio_set_level(PIN_BLK, 1);
      memset(framebuffer, 0xFF, sizeof(framebuffer));
      if (selected_column != 0) {
        int x = selected_column - 1;
        for (int y = ROW_START; y < ROW_END; ++y) {
          framebuffer[y * 16 + x / 8] &=
              (uint8_t)~(0x80u >> (x % 8));
        }
      }
      mn12832l_encode_frame(framebuffer, scans);
      printf("SELECTED COLUMN %d ONLY, MODE %c; x=%d\n",
             selected_column, grid_mode,
             selected_column == 0 ? -1 : selected_column - 1);
      fflush(stdout);
      break;
    }
  }

shutdown_single_ab:
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);
  printf("MN12832L single-column A/B/C/D complete: %s\n", esp_err_to_name(err));
  return err;
}

/* Four-pattern overlap check, 1-based logical columns:
 * 1: 8
 * 2: 16
 * 3: 8 and 16
 * 4: 8, 16 and 24
 *
 * The existing logical-to-electrode encoder is deliberately untouched.
 * Grid modes A/B/C/D are identical to the previous isolation experiment.
 * A full 44-scan refresh runs even when only one column is active.
 */
esp_err_t mn12832l_spi_overlap_pattern_test(void) {
  enum {
    ROW_START = 8, ROW_END = 24,
    MAX_ON_US = 900000000,
    MAX_IDLE_US = 300000000
  };
  static const int logical_columns[3] = {8, 16, 24};
  static const uint8_t patterns[4] = {
      0x01, /* 8 */
      0x02, /* 16 */
      0x03, /* 8+16 */
      0x07  /* 8+16+24 */
  };
  static const char *names[4] = {
      "8", "16", "8 + 16", "8 + 16 + 24"
  };
  /* Static storage avoids overflowing the main ESP-IDF task stack. */
  static uint8_t framebuffer[128 * 32 / 8];
  static uint8_t scans[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES];

  esp_err_t err = mn12832l_spi_init();
  if (err != ESP_OK) return err;

  int selected = 0;
  char mode = 'A';
  uint32_t measured_frames = 0;
  int64_t on_start = 0, last_input = 0, measurement_start = 0;

  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);

  /* Construct pattern 1 before power-up. Preserve active-low I1 format. */
  memset(framebuffer, 0xFF, sizeof(framebuffer));
  for (int y = ROW_START; y < ROW_END; ++y) {
    int x = 7;
    framebuffer[y * 16 + x / 8] &= (uint8_t)~(0x80u >> (x % 8));
  }
  mn12832l_encode_frame(framebuffer, scans);

  printf("MN12832L four-pattern test, Y=8..23\n");
  printf("1=8; 2=16; 3=8+16; 4=8+16+24; SPACE/N=next; P=previous\n");
  printf("A=paired; B=sequential; C=previous grid; D=next grid; Q=quit\n");
  printf("5min inactivity / 15min total HV-on timeout.\n");
  fflush(stdout);

  gpio_set_level(PIN_EF, 1);
  vTaskDelay(pdMS_TO_TICKS(2000));
  gpio_set_level(PIN_HV, 1);
  on_start = esp_timer_get_time();
  last_input = measurement_start = on_start;
  printf("PATTERN 1/4: %s, MODE A\n", names[selected]);
  fflush(stdout);

  while (true) {
    int64_t now = esp_timer_get_time();
    if (now - on_start >= MAX_ON_US || now - last_input >= MAX_IDLE_US) {
      printf("Four-pattern timeout; shutting down.\n");
      err = ESP_ERR_TIMEOUT;
      break;
    }

    for (int scan = 0; scan < MN12832L_SCAN_COUNT; ++scan) {
      err = mn12832l_manual_timed_scan(scans[scan], scan, mode);
      if (err != ESP_OK) goto overlap_shutdown;
    }
    /* Maintain the idle-task watchdog and blanked state between frames. */
    vTaskDelay(1);

    measured_frames++;
    int64_t measured_now = esp_timer_get_time();
    if (measured_now - measurement_start >= 2000000) {
      printf("REFRESH MODE %c PATTERN %d: %.2f frames/s\n",
             mode, selected + 1,
             (double)measured_frames * 1000000.0 /
                 (double)(measured_now - measurement_start));
      fflush(stdout);
      measured_frames = 0;
      measurement_start = measured_now;
    }

    char key;
    while (read(STDIN_FILENO, &key, 1) == 1) {
      if (key == 'q' || key == 'Q') {
        printf("Quit requested.\n");
        goto overlap_shutdown;
      }
      if ((key >= 'a' && key <= 'd') ||
          (key >= 'A' && key <= 'D')) {
        mode = (key >= 'a' && key <= 'd')
                   ? (char)(key - 'a' + 'A') : key;
        last_input = esp_timer_get_time();
        measured_frames = 0;
        measurement_start = last_input;
        gpio_set_level(PIN_BLK, 1);
        printf("PATTERN %d/4: %s; MODE %c\n",
               selected + 1, names[selected], mode);
        fflush(stdout);
        break;
      }

      int next = selected;
      if (key >= '1' && key <= '4') {
        next = key - '1';
      } else if (key == ' ' || key == 'n' || key == 'N') {
        next = (selected + 1) % 4;
      } else if (key == 'p' || key == 'P') {
        next = (selected + 3) % 4;
      } else {
        continue;
      }
      last_input = esp_timer_get_time();
      selected = next;
      gpio_set_level(PIN_BLK, 1);
      memset(framebuffer, 0xFF, sizeof(framebuffer));
      for (int i = 0; i < 3; ++i) {
        if ((patterns[selected] & (1u << i)) == 0) continue;
        int x = logical_columns[i] - 1;  /* UI is 1-based. */
        for (int y = ROW_START; y < ROW_END; ++y) {
          framebuffer[y * 16 + x / 8] &=
              (uint8_t)~(0x80u >> (x % 8));
        }
      }
      mn12832l_encode_frame(framebuffer, scans);
      printf("PATTERN %d/4: %s; MODE %c\n",
             selected + 1, names[selected], mode);
      fflush(stdout);
      break;
    }
  }

overlap_shutdown:
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);
  printf("MN12832L four-pattern test ended: %s\n", esp_err_to_name(err));
  return err;
}

/* Manual 16-column cumulative fill for frame-by-frame observations.
 * Send a space through the primary ESP-IDF serial console to add a column;
 * 'q' exits. The displayed frame is continuously multiplexed even while
 * waiting for input, with a global and idle timeout for supervised operation.
 *
 * Uses raw POSIX read rather than getchar() so a space need not be followed
 * by Enter on the ESP32 console. The host terminal must transmit keystrokes
 * immediately (idf.py monitor normally does).
 */
esp_err_t mn12832l_spi_manual_fill_test(void) {
  enum {
    WIDTH = 16,
    STRIP_Y_START = 8,
    STRIP_Y_END = 24,
    MAX_ON_US = 900000000,
    MAX_IDLE_US = 300000000
  };
  esp_err_t err = mn12832l_spi_init();
  if (err != ESP_OK) {
    return err;
  }

  /* This bench-only function runs on app_main's relatively small stack.
   * Keep the 3.2 KiB of diagnostic buffers in static DRAM instead. */
  static uint8_t framebuffer[128 * 32 / 8];
  static uint8_t scans[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES];
  static uint8_t old_scans[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES];
  int lit_columns = 1;
  char grid_mode = 'A';  /* Mode A by default. */
  uint32_t measured_frames = 0;
  int64_t measurement_start = 0;
  int64_t on_start = 0;
  int64_t last_input = 0;
  bool finished = false;

  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);

  /* First column: draw a 16-row vertical segment.
   * Each subsequent space adds a column to the RIGHT. */
  memset(framebuffer, 0xFF, sizeof(framebuffer));
  for (int y = STRIP_Y_START; y < STRIP_Y_END; ++y) {
    framebuffer[y * 16] &= (uint8_t)~0x80u;
  }
  mn12832l_encode_frame(framebuffer, scans);

  printf("MN12832L manual fill: X=0..15, Y=8..23.\n");
  printf("SPACE=add column; a=paired; b=split; c=Gx; d=Gx+1; q=quit.\n");
  printf("MODE A: paired 250us, then blank 250us (2 SPI transfers).\n");
  printf("MODE B: Gx/Gx+1. Modes C/D: one grid then blank; 250us/phase.\n");
  printf("Mode-matched timing; 2s measured refresh-rate reports.\n");
  printf("After column 16, another SPACE ends the test.\n");
  printf("Auto-off: 30 seconds idle or 120 seconds total HV-on.\n");
  fflush(stdout);

  gpio_set_level(PIN_EF, 1);
  vTaskDelay(pdMS_TO_TICKS(2000));
  gpio_set_level(PIN_HV, 1);
  on_start = esp_timer_get_time();
  last_input = on_start;
  measurement_start = on_start;
  printf("MODE A ACTIVE: paired grids\n");
  printf("COLUMN 1/16: logical X=0 lit\n");
  printf("FRAMEBUFFER y=8 x=0..15, 1=ON: 1000000000000000   I1 raw: 7F FF\n");
  fflush(stdout);

  while (!finished) {
    int64_t now = esp_timer_get_time();
    if (now - on_start >= MAX_ON_US ||
        now - last_input >= MAX_IDLE_US) {
      printf("Manual fill timeout; shutting down.\n");
      err = ESP_ERR_TIMEOUT;
      break;
    }

    /* Refresh the full scan continuously, never wait for keyboard
     * input with the currently selected grid held unblanked. */
    for (int scan = 0; scan < MN12832L_SCAN_COUNT; ++scan) {
      err = mn12832l_manual_timed_scan(scans[scan], scan, grid_mode);
      if (err != ESP_OK) {
        goto shutdown_manual_fill;
      }
    }

    /* The refresh loop otherwise runs continuously on CPU0 and starves
     * IDLE0, triggering the ESP-IDF task watchdog. Briefly yield once per
     * full 44-scan frame; the display remains blanked after the last scan.
     * Never disable the task watchdog to conceal a scheduling bug. */
    vTaskDelay(1);

    ++measured_frames;
    int64_t measure_now = esp_timer_get_time();
    if (measure_now - measurement_start >= 2000000) {
      printf("REFRESH MODE %c: %.2f frames/s (%lu frames in %lld ms)\n",
             grid_mode,
             (double)measured_frames * 1000000.0 /
                 (double)(measure_now - measurement_start),
             (unsigned long)measured_frames,
             (long long)((measure_now - measurement_start) / 1000));
      fflush(stdout);
      measured_frames = 0;
      measurement_start = measure_now;
    }

    /* The default ESP-IDF console VFS is non-blocking. Direct read
     * bypasses stdio's newline-oriented input buffering. */
    char key;
    while (read(STDIN_FILENO, &key, 1) == 1) {
      if (key == 'q' || key == 'Q') {
        printf("Quit requested.\n");
        finished = true;
        break;
      }
      if (key == 'a' || key == 'A' || key == 'b' || key == 'B' ||
          key == 'c' || key == 'C' || key == 'd' || key == 'D') {
        grid_mode = (key >= 'a' && key <= 'z') ? key - ('a' - 'A') : key;
        last_input = esp_timer_get_time();
        measured_frames = 0;
        measurement_start = last_input;
        gpio_set_level(PIN_BLK, 1);
        printf("MODE %c ACTIVE, COLUMN %d/16: %s\n",
               grid_mode, lit_columns,
               grid_mode == 'A' ? "paired/blank" :
               grid_mode == 'B' ? "Gx/Gx+1" :
               grid_mode == 'C' ? "Gx only" : "Gx+1 only");
        fflush(stdout);
        break;
      }
      if (key != ' ') {
        continue;
      }
      last_input = esp_timer_get_time();
      if (lit_columns >= WIDTH) {
        finished = true;
        break;
      }
      int x = lit_columns;
      for (int y = STRIP_Y_START; y < STRIP_Y_END; ++y) {
        framebuffer[y * 16 + x / 8] &=
            (uint8_t)~(0x80u >> (x % 8));
      }
      ++lit_columns;
      memcpy(old_scans, scans, sizeof(old_scans));
      mn12832l_encode_frame(framebuffer, scans);
      gpio_set_level(PIN_BLK, 1);
      mn12832l_manual_dump_transition(framebuffer, old_scans, scans,
                                      lit_columns);
      printf("COLUMN %d/%d: logical X=0..%d lit\n",
             lit_columns, WIDTH, lit_columns - 1);
      fflush(stdout);
      /* Only one advance per refresh cycle even if the console
       * delivers multiple spaces; prevents accidental rapid skipping. */
      break;
    }
  }

shutdown_manual_fill:
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);
  printf("MN12832L manual fill ended at %d columns: %s\n",
         lit_columns, esp_err_to_name(err));
  return err;
}

static void mn12832l_refresh_dry_run(
    uint8_t scans[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES]) {
  // Keep the display safely disabled.
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);

  for (uint8_t scan = 0; scan < MN12832L_SCAN_COUNT; scan++) {

    // Send 240 bits for the current scan.
    mn12832l_shift_bits(scans[scan], 240, NULL, NULL);

    // Transfer shift-register contents to output latches.
    gpio_set_level(PIN_LAT, 1);
    esp_rom_delay_us(1);
    gpio_set_level(PIN_LAT, 0);

    // Simulate the display period.
    // esp_rom_delay_us(2000);
  }

  printf("MN12832L: Completed 44-scan dry refresh\n");
}

void mn12832l_test_power(void) {
  // Keep every display output blanked.
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);

  printf("Power test: supplies OFF\n");
  vTaskDelay(pdMS_TO_TICKS(3000));

  // Enable supplies, but NEVER unblank.
  gpio_set_level(PIN_EF, 1);
  vTaskDelay(pdMS_TO_TICKS(100));

  gpio_set_level(PIN_HV, 1);

  printf("Power test: supplies ON, display BLANKED\n");
  vTaskDelay(pdMS_TO_TICKS(3000));

  // Return to the safe state.
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);

  printf("Power test complete\n");
}

void mn12832l_test_blank_polarity(void) {
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);

  printf("BLK = HIGH\n");
  gpio_set_level(PIN_BLK, 1);
  vTaskDelay(pdMS_TO_TICKS(3000));

  printf("BLK = LOW\n");
  gpio_set_level(PIN_BLK, 0);
  vTaskDelay(pdMS_TO_TICKS(3000));

  // Return to the datasheet-defined safe state.
  gpio_set_level(PIN_BLK, 1);

  printf("BLK polarity test complete\n");
}

void mn12832l_test_illumination(const uint8_t *framebuffer) {
  if (framebuffer == NULL) {
    printf("MN12832L: No framebuffer!\n");
    return;
  }

  uint8_t scans[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES];

  mn12832l_encode_frame(framebuffer, scans);

  printf("MN12832L: Starting illumination test\n");

  // Start with everything disabled.
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);

  // Enable filament first.
  gpio_set_level(PIN_EF, 1);

  // Allow the filament to warm before applying display voltage.
  vTaskDelay(pdMS_TO_TICKS(2000));

  // Enable display voltage.
  gpio_set_level(PIN_HV, 1);

  // Run five complete refresh cycles.
  for (int cycle = 0; cycle < 5; cycle++) {

    for (int scan = 0; scan < MN12832L_SCAN_COUNT; scan++) {

      // Blank while updating the driver.
      gpio_set_level(PIN_BLK, 1);

      // Shift the next scan frame.
      mn12832l_shift_bits(scans[scan], 240, NULL, NULL);

      // Latch the new data.
      gpio_set_level(PIN_LAT, 1);
      esp_rom_delay_us(1);
      gpio_set_level(PIN_LAT, 0);

      // Briefly illuminate this scan.
      gpio_set_level(PIN_BLK, 0);
      esp_rom_delay_us(100);

      // Blank before advancing.
      gpio_set_level(PIN_BLK, 1);
    }
  }

  // Shut everything down.
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);

  printf("MN12832L: Illumination test complete\n");
}

void mn12832l_test_refresh(const uint8_t *framebuffer) {
  if (framebuffer == NULL) {
    printf("MN12832L: No framebuffer provided!\n");
    return;
  }

  uint8_t scans[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES];

  // Convert the supplied framebuffer into 44 scan frames.
  mn12832l_encode_frame(framebuffer, scans);

  // Count the scans containing illuminated pixels.
  unsigned active_scans = 0;

  for (unsigned scan = 0; scan < MN12832L_SCAN_COUNT; scan++) {

    for (unsigned i = 0; i < 24; i++) {

      if (scans[scan][i] != 0) {
        active_scans++;
        break;
      }
    }
  }

  printf("MN12832L: %u scans contain illuminated pixels\n", active_scans);

  mn12832l_refresh_dry_run(scans);
}

void mn12832l_test_full_frame(void) {
  uint8_t framebuffer[512];
  memset(framebuffer, 0xFF, sizeof(framebuffer));

  // Turn on three pixels in different parts of the image.
  const unsigned pixels[][2] = {{0, 0}, {64, 16}, {127, 31}};

  for (unsigned i = 0; i < 3; i++) {
    unsigned x = pixels[i][0];
    unsigned y = pixels[i][1];

    unsigned byte_index = y * 16 + x / 8;

    framebuffer[byte_index] &= ~(0x80u >> (x % 8));
  }

  // Generate all 44 scan frames.
  uint8_t scans[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES];

  mn12832l_encode_frame(framebuffer, scans);

  // Print only scans containing illuminated anodes.
  for (unsigned scan = 0; scan < MN12832L_SCAN_COUNT; scan++) {

    bool has_pixels = false;

    // First 24 bytes contain the anode data.
    for (unsigned i = 0; i < 24; i++) {
      if (scans[scan][i] != 0) {
        has_pixels = true;
        break;
      }
    }

    if (has_pixels) {
      printf("Scan %u contains illuminated pixels\n", scan);
    }
  }
}

void mn12832l_test_scan_encoder(void) {
  uint8_t framebuffer[512];
  memset(framebuffer, 0xFF, sizeof(framebuffer));

  // Our test pixel.
  unsigned x = 64;
  unsigned y = 16;

  // Turn on pixel (64,16).
  unsigned byte_index = y * 16 + x / 8;
  uint8_t mask = 0x80u >> (x % 8);

  framebuffer[byte_index] &= ~mask;

  // Determine which scan contains this pixel.
  uint8_t scan = x / 3;

  uint8_t scan_frame[MN12832L_FRAME_BYTES];

  mn12832l_encode_scan(framebuffer, scan, scan_frame);

  printf("Pixel (%u,%u), scan %u:\n", x, y, scan);

  for (int i = 0; i < MN12832L_FRAME_BYTES; i++) {
    printf("%02X ", scan_frame[i]);
  }

  printf("\n");
}

void mn12832l_test_anode_encoder(void) {
  uint8_t frame[MN12832L_FRAME_BYTES] = {0};

  mn12832l_set_anode(frame, 1, 'A');
  mn12832l_set_anode(frame, 1, 'B');
  mn12832l_set_anode(frame, 1, 'C');

  printf("Anode test: ");

  for (int i = 0; i < MN12832L_FRAME_BYTES; i++) {
    printf("%02X ", frame[i]);
  }

  printf("\n");
}

void mn12832l_test_grid_encoder(void) {
  uint8_t frame[MN12832L_FRAME_BYTES] = {0};

  mn12832l_select_grid(frame, 1);
  mn12832l_select_grid(frame, 2);

  printf("Grid test: ");

  for (int i = 0; i < MN12832L_FRAME_BYTES; i++) {
    printf("%02X ", frame[i]);
  }

  printf("\n");
}

#define N_BITS 1024
#define MAX_LAG 320
#define HALF_CLOCK_US 10

static uint8_t sent[N_BITS], so1[N_BITS], so2[N_BITS];

static uint32_t prng(uint32_t *state) {
  uint32_t x = *state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  return *state = x;
}

static void assess(const char *name, const uint8_t *received) {
  unsigned high = 0, edges = 0;
  for (int i = 0; i < N_BITS; ++i) {
    high += received[i];
    if (i && received[i] != received[i - 1])
      ++edges;
  }
  printf("%s: high=%u/%d, transitions=%u\n", name, high, N_BITS, edges);
  if (edges < 8) {
    printf(
        "  No meaningful activity; verify power, pinout, and logic levels.\n");
    return;
  }
  int best_lag = -1, best_invert = 0;
  unsigned best_matches = 0, best_total = 0;
  /* Use a fixed comparison window to avoid unfairly favoring large delays. */
  const int start = MAX_LAG;
  for (int lag = 0; lag <= MAX_LAG; ++lag) {
    unsigned match = 0, inverse = 0;
    for (int i = start; i < N_BITS; ++i) {
      match += (received[i] == sent[i - lag]);
      inverse += (received[i] != sent[i - lag]);
    }
    if (match > best_matches) {
      best_matches = match;
      best_lag = lag;
      best_invert = 0;
    }
    if (inverse > best_matches) {
      best_matches = inverse;
      best_lag = lag;
      best_invert = 1;
    }
  }
  best_total = N_BITS - start;
  printf("  best match: delay=%d clock(s), %u/%u (%.1f%%), inverted=%s\n",
         best_lag, best_matches, best_total, 100.0 * best_matches / best_total,
         best_invert ? "yes" : "no");
  if (best_matches < best_total * 9 / 10)
    printf("  No convincing shifted-pattern match found.\n");
}

esp_err_t mn12832l_init(void) {
  gpio_config_t outputs = {.pin_bit_mask =
                               (1ULL << PIN_SIN) | (1ULL << PIN_CLK) |
                               (1ULL << PIN_BLK) | (1ULL << PIN_LAT) |
                               (1ULL << PIN_HV) | (1ULL << PIN_EF),

                           .mode = GPIO_MODE_OUTPUT,
                           .pull_up_en = GPIO_PULLUP_DISABLE,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .intr_type = GPIO_INTR_DISABLE};

  gpio_config_t inputs = {.pin_bit_mask = (1ULL << PIN_SO1) | (1ULL << PIN_SO2),

                          .mode = GPIO_MODE_INPUT,
                          .pull_up_en = GPIO_PULLUP_DISABLE,
                          .pull_down_en = GPIO_PULLDOWN_DISABLE,
                          .intr_type = GPIO_INTR_DISABLE};

  // Establish safe output levels
  ESP_ERROR_CHECK(gpio_set_level(PIN_BLK, 1));
  ESP_ERROR_CHECK(gpio_set_level(PIN_LAT, 0));
  ESP_ERROR_CHECK(gpio_set_level(PIN_HV, 0));
  ESP_ERROR_CHECK(gpio_set_level(PIN_EF, 0));
  ESP_ERROR_CHECK(gpio_set_level(PIN_CLK, 0));
  ESP_ERROR_CHECK(gpio_set_level(PIN_SIN, 0));

  // Configure the GPIO hardware
  ESP_ERROR_CHECK(gpio_config(&outputs));
  ESP_ERROR_CHECK(gpio_config(&inputs));

  return ESP_OK;
}

static void mn12832l_shift_bits(const uint8_t *data, size_t bit_count,
                                uint8_t *so1_capture, uint8_t *so2_capture) {
  for (size_t i = 0; i < bit_count; i++) {

    size_t byte_index = i / 8;
    uint8_t bit_index = 7 - (i % 8);

    uint8_t bit = (data[byte_index] >> bit_index) & 1;

    gpio_set_level(PIN_SIN, bit);

    esp_rom_delay_us(1);
    gpio_set_level(PIN_CLK, 1);
    esp_rom_delay_us(1);
    gpio_set_level(PIN_CLK, 0);
    esp_rom_delay_us(1);

    // Capture serial outputs if requested
    if (so1_capture != NULL) {
      so1_capture[i] = gpio_get_level(PIN_SO1);
    }

    if (so2_capture != NULL) {
      so2_capture[i] = gpio_get_level(PIN_SO2);
    }
  }
}

void mn12832l_test_sweep(void)
{
    uint8_t framebuffer[512];
    uint8_t scans[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES];

    // Start safely blanked.
    gpio_set_level(PIN_BLK, 1);
    gpio_set_level(PIN_HV, 0);
    gpio_set_level(PIN_EF, 0);

    // Warm the filament once.
    gpio_set_level(PIN_EF, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));

    gpio_set_level(PIN_HV, 1);

    // Move one vertical line across eight positions.
    for (int x = 100; x <= 107; x++) {

        memset(framebuffer, 0xFF, sizeof(framebuffer));

        // Draw the vertical line.
        for (int y = 8; y <= 23; y++) {
            int byte_index = y * 16 + x / 8;
            uint8_t mask = 0x80u >> (x % 8);

            framebuffer[byte_index] &= ~mask;
        }

        mn12832l_encode_frame(framebuffer, scans);

        printf("Displaying X=%d\n", x);

        // Refresh this position for 12 cycles.
        for (int cycle = 0; cycle < 12; cycle++) {

            for (int scan = 0; scan < MN12832L_SCAN_COUNT; scan++) {

                gpio_set_level(PIN_BLK, 1);

                mn12832l_shift_bits(scans[scan], 240, NULL, NULL);

                gpio_set_level(PIN_LAT, 1);
                esp_rom_delay_us(1);
                gpio_set_level(PIN_LAT, 0);

                gpio_set_level(PIN_BLK, 0);
                esp_rom_delay_us(100);
                gpio_set_level(PIN_BLK, 1);
            }
        }
    }

    // Shut down after the sweep.
    gpio_set_level(PIN_BLK, 1);
    gpio_set_level(PIN_HV, 0);
    gpio_set_level(PIN_EF, 0);

    printf("MN12832L: Sweep complete\n");
}

esp_err_t mn12832l_dry_run(void) {
  // Ensure the display remains disabled
  gpio_set_level(PIN_BLK, 1);
  gpio_set_level(PIN_HV, 0);
  gpio_set_level(PIN_EF, 0);

  uint8_t test_pattern[30];
  uint8_t zeros[30] = {0};

  uint8_t received_so1[240];
  uint8_t received_so2[240];

  // Create our alternating test pattern
  for (int i = 0; i < 30; i++) {
    test_pattern[i] = 0xAA;
  }

  // First transfer: load the known pattern
  mn12832l_shift_bits(test_pattern, 240, NULL, NULL);

  // Second transfer: capture the previous pattern
  mn12832l_shift_bits(zeros, 240, received_so1, received_so2);

  int matches_so1 = 0;
  int matches_so2 = 0;

  // Our previous test established a 239-clock delay.
  // Compare the first 239 returned bits.
  for (int i = 0; i < 239; i++) {

    // 0xAA starts with 1,0,1,0...
    // With our sampling convention, the first
    // returned bit corresponds to the second input bit.
    uint8_t expected = i & 1;

    if (received_so1[i] == expected) {
      matches_so1++;
    }

    if (received_so2[i] == expected) {
      matches_so2++;
    }
  }

  printf("SO1 frame readback: %d/239 bits correct\n", matches_so1);

  printf("SO2 frame readback: %d/239 bits correct\n", matches_so2);

  // Pulse the latch while the display is blanked
  gpio_set_level(PIN_LAT, 1);
  esp_rom_delay_us(5);
  gpio_set_level(PIN_LAT, 0);

  printf("MN12832L: 240-bit dry run complete\n");

  return ESP_OK;
}

esp_err_t mn12832l_run_diagnostic(void) {
  printf("\nMN12832L serial-chain probe (no HV/EF/LAT control)\n");
  // gpio_config_t outputs = {
  //     .pin_bit_mask =
  //         (1ULL << PIN_SIN) |
  //         (1ULL << PIN_CLK) |
  //         (1ULL << PIN_BLK) |
  //         (1ULL << PIN_LAT) |
  //         (1ULL << PIN_HV)  |
  //         (1ULL << PIN_EF),
  //     .mode = GPIO_MODE_OUTPUT,
  //     .pull_up_en = GPIO_PULLUP_DISABLE,
  //     .pull_down_en = GPIO_PULLDOWN_DISABLE,
  //     .intr_type = GPIO_INTR_DISABLE
  // };
  // gpio_config_t inputs = {
  //     .pin_bit_mask = (1ULL << PIN_SO1) | (1ULL << PIN_SO2),
  //     .mode = GPIO_MODE_INPUT,
  //     .pull_up_en = GPIO_PULLUP_DISABLE,
  //     .pull_down_en = GPIO_PULLDOWN_DISABLE,
  //     .intr_type = GPIO_INTR_DISABLE
  // };
  // /* BLK high is the documented inactive-output state for the glass driver.
  // */ ESP_ERROR_CHECK(gpio_set_level(PIN_BLK, 1));
  // ESP_ERROR_CHECK(gpio_set_level(PIN_CLK, 0));
  // ESP_ERROR_CHECK(gpio_set_level(PIN_SIN, 0));
  // ESP_ERROR_CHECK(gpio_config(&outputs));
  // ESP_ERROR_CHECK(gpio_config(&inputs));

  uint32_t seed = 0xBEEF1234u;
  for (int i = 0; i < N_BITS; ++i)
    sent[i] = (uint8_t)(prng(&seed) & 1u);

  /* Input bits are clocked at rising edges. Sample SO after each falling
     edge; the returned bit may reflect an internal pipeline delay. */
  for (int i = 0; i < N_BITS; ++i) {
    gpio_set_level(PIN_SIN, sent[i]);
    esp_rom_delay_us(HALF_CLOCK_US);
    gpio_set_level(PIN_CLK, 1);
    esp_rom_delay_us(HALF_CLOCK_US);
    gpio_set_level(PIN_CLK, 0);
    esp_rom_delay_us(HALF_CLOCK_US);
    so1[i] = gpio_get_level(PIN_SO1) ? 1 : 0;
    so2[i] = gpio_get_level(PIN_SO2) ? 1 : 0;
  }
  gpio_set_level(PIN_SIN, 0);
  gpio_set_level(PIN_CLK, 0);
  gpio_set_level(PIN_BLK, 1);
  assess("SO1", so1);
  assess("SO2", so2);
  unsigned disagreement = 0;
  for (int i = 0; i < N_BITS; ++i)
    disagreement += (so1[i] != so2[i]);
  printf("SO1/SO2 disagree at %u/%d samples.\n", disagreement, N_BITS);
  unsigned after_240 = 0;

  for (int i = 240; i < N_BITS; i++) {
    if (so1[i] != so2[i]) {
      after_240++;
    }
  }

  printf("Disagreements after clock 240: %u\n", after_240);

  for (int i = 0; i < 240; i++) {
    if (so1[i] != so2[i]) {
      printf("Clock %d: SO1=%u SO2=%u\n", i, so1[i], so2[i]);
    }
  }

  return ESP_OK;
}
