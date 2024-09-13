#pragma once

#include "rc522_private.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    RC522_PCD_FIRMWARE_CLONE = 0x88,       // clone
    RC522_PCD_FIRMWARE_00 = 0x90,          // v0.0
    RC522_PCD_FIRMWARE_10 = 0x91,          // v1.0
    RC522_PCD_FIRMWARE_20 = 0x92,          // v2.0
    RC522_PCD_FIRMWARE_COUNTERFEIT = 0x12, // counterfeit chip
} rc522_pcd_firmware_t;

esp_err_t rc522_pcd_init(rc522_handle_t rc522);

esp_err_t rc522_pcd_soft_reset(rc522_handle_t rc522, uint32_t timeout_ms);

esp_err_t rc522_pcd_antenna_on(rc522_handle_t rc522);

esp_err_t rc522_pcd_firmware(rc522_handle_t rc522, rc522_pcd_firmware_t *result);

char *rc522_pcd_firmware_name(rc522_pcd_firmware_t firmware);

esp_err_t rc522_pcd_stop_active_command(rc522_handle_t rc522);

esp_err_t rc522_pcd_fifo_write(rc522_handle_t rc522, uint8_t *data, uint8_t data_length);

esp_err_t rc522_pcd_fifo_flush(rc522_handle_t rc522);

esp_err_t rc522_pcd_start_data_transmission(rc522_handle_t rc522);

esp_err_t rc522_pcd_stop_data_transmission(rc522_handle_t rc522);

esp_err_t rc522_rw_test(rc522_handle_t rc522);

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
