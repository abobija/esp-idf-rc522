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

esp_err_t rc522_picc_presence(rc522_handle_t rc522, rc522_picc_presence_t *result);

esp_err_t rc522_firmware(rc522_handle_t rc522, uint8_t *result);

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
