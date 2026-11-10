#pragma once

#include "esp_err.h"

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