#pragma once

#include "rc522_private.h"
#include "rc522_registers.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    bool is_present;
} rc522_picc_presence_t;

typedef enum
{
    RC522_FW_CLONE = 0x88,       // clone
    RC522_FW_00 = 0x90,          // v0.0
    RC522_FW_10 = 0x91,          // v1.0
    RC522_FW_20 = 0x92,          // v2.0
    RC522_FW_COUNTERFEIT = 0x12, // counterfeit chip
} rc522_firmware_t;

esp_err_t rc522_picc_presence(rc522_handle_t rc522, rc522_picc_presence_t *result);

esp_err_t rc522_picc_uid(rc522_handle_t rc522, rc522_tag_uid_t *uid);

esp_err_t rc522_firmware(rc522_handle_t rc522, rc522_firmware_t *result);

char *rc522_firmware_name(rc522_firmware_t firmware);

esp_err_t rc522_antenna_on(rc522_handle_t rc522);

esp_err_t rc522_stop_active_command(rc522_handle_t rc522);

esp_err_t rc522_fifo_write(rc522_handle_t rc522, uint8_t *data, uint8_t data_length);

esp_err_t rc522_fifo_flush(rc522_handle_t rc522);

esp_err_t rc522_soft_reset(rc522_handle_t rc522, uint32_t timeout_ms);

esp_err_t rc522_start_data_transmission(rc522_handle_t rc522);

esp_err_t rc522_stop_data_transmission(rc522_handle_t rc522);

#ifdef __cplusplus
}
#endif
