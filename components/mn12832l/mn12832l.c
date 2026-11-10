/* MN12832L board serial-output probe; ESP-IDF 5.x/6.x, ESP32-S3.
 * Does not control filament, high voltage, or display latches.
 * IMPORTANT: Verify logic levels and BLK safe state before connecting.
 */
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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

#define MN12832L_SCAN_COUNT 44

static void mn12832l_encode_frame(
    const uint8_t *framebuffer,
    uint8_t output[MN12832L_SCAN_COUNT][MN12832L_FRAME_BYTES]) {
  for (uint8_t scan = 0; scan < MN12832L_SCAN_COUNT; scan++) {
    mn12832l_encode_scan(framebuffer, scan, output[scan]);
  }
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
