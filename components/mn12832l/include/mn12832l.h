#pragma once

#include "esp_err.h"
#include <stdint.h>

esp_err_t mn12832l_init(void);
esp_err_t mn12832l_run_diagnostic(void);
esp_err_t mn12832l_dry_run(void);
void mn12832l_test_grid_encoder(void);
void mn12832l_test_anode_encoder(void);
void mn12832l_test_scan_encoder(void);
void mn12832l_test_full_frame(void);
void mn12832l_test_refresh(const uint8_t *framebuffer);
void mn12832l_test_illumination(const uint8_t *framebuffer);
void mn12832l_test_power(void);
void mn12832l_test_blank_polarity(void);
void mn12832l_test_sweep(void);
/* Hardware-SPI transport tests. The illumination test powers down on return. */
esp_err_t mn12832l_spi_dry_run(const uint8_t *framebuffer);
esp_err_t mn12832l_spi_illumination_test(const uint8_t *framebuffer);
esp_err_t mn12832l_spi_column_walk(void);
esp_err_t mn12832l_spi_grid_walk(void);
