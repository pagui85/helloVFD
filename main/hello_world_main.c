/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "lvgl.h"
#include "mn12832l.h"
#include "vfd_framebuffer.h"

static void vfd_flush_cb(lv_display_t *display, const lv_area_t *area,
                         uint8_t *px_map) {
  printf("LVGL rendered an area!\n");
  printf("X: %ld to %ld\n", (long)area->x1, (long)area->x2);
  printf("Y: %ld to %ld\n", (long)area->y1, (long)area->y2);

  // Skip LVGL's 8-byte palette
  const uint8_t *pixels = px_map + 8;

  vfd_framebuffer_copy(pixels);

  for (int y = 0; y < 32; y++) {
    for (int x = 0; x < 128; x++) {

      int byte_index = y * 16 + (x / 8);
      int bit_index = 7 - (x % 8);

      bool pixel_on = (pixels[byte_index] >> bit_index) & 1;

      putchar(pixel_on ? ' ' : '#');
    }

    putchar('\n');
  }

  lv_display_flush_ready(display);
}

void app_main(void) {
  size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);

  printf("PSRAM available: %zu bytes\n", psram_size);

  lv_init();
  vfd_framebuffer_init();

  lv_display_t *display = lv_display_create(128, 32);
  lv_display_set_color_format(display, LV_COLOR_FORMAT_I1);
  static uint8_t draw_buffer[520];
  lv_display_set_buffers(display, draw_buffer, NULL, sizeof(draw_buffer),
                         LV_DISPLAY_RENDER_MODE_FULL);

  lv_display_set_flush_cb(display, vfd_flush_cb);

  lv_obj_t *label = lv_label_create(lv_screen_active());

  lv_label_set_text(label, "HELLO VFD!");

  lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, 0);

  lv_timer_handler();

  const uint8_t *frame = vfd_framebuffer_get();

  //   printf("First 16 framebuffer bytes:\n");

  //   for (int i = 5 * 16; i < 6 * 16; i++) {
  //     printf("%02X ", frame[i]);
  //   }

  ESP_ERROR_CHECK(mn12832l_init());

  // SPI owns the SIN/CLK pins after this point. Run any GPIO bit-bang
  // diagnostics BEFORE the first SPI call, never concurrently with it.
  ESP_ERROR_CHECK(mn12832l_spi_dry_run(frame));

  // Experiment: preserve grid and SPI timing, shift only the anode
  // payload two scan slots to match the reference Arduino driver's phase.
  ESP_ERROR_CHECK(mn12832l_spi_phase_shift_test(frame));
  // ESP_ERROR_CHECK(mn12832l_spi_grid_walk());
  // ESP_ERROR_CHECK(mn12832l_spi_column_walk());
  // ESP_ERROR_CHECK(mn12832l_spi_illumination_test(frame));
}
