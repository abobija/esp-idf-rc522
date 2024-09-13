#pragma once

#include "rc522_private.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t rc522_pcd_init(rc522_handle_t rc522);

esp_err_t rc522_pcd_soft_reset(rc522_handle_t rc522, uint32_t timeout_ms);

esp_err_t rc522_pcd_antenna_on(rc522_handle_t rc522);

esp_err_t rc522_pcd_write_n(rc522_handle_t rc522, uint8_t addr, uint8_t n, uint8_t *data);

esp_err_t rc522_pcd_write(rc522_handle_t rc522, uint8_t addr, uint8_t val);

esp_err_t rc522_pcd_read_n(rc522_handle_t rc522, uint8_t addr, uint8_t n, uint8_t *buffer);

esp_err_t rc522_pcd_read(rc522_handle_t rc522, uint8_t addr, uint8_t *value_ref);

esp_err_t rc522_pcd_set_bitmask(rc522_handle_t rc522, uint8_t addr, uint8_t mask);

esp_err_t rc522_pcd_clear_bitmask(rc522_handle_t rc522, uint8_t addr, uint8_t mask);

esp_err_t rc522_pcd_write_map(rc522_handle_t rc522, const uint8_t map[][2], uint8_t map_length);

#ifdef __cplusplus
}
#endif
