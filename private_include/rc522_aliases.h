#pragma once

#include "rc522_private.h"
#include "rc522_registers.h"

#ifdef __cplusplus
extern "C" {
#endif

bool rc522_is_able_to_start(rc522_handle_t rc522);

esp_err_t rc522_firmware(rc522_handle_t rc522, uint8_t *result);

esp_err_t rc522_antenna_on(rc522_handle_t rc522, rc522_rx_gain_t gain);

esp_err_t rc522_stop_active_command(rc522_handle_t rc522);

esp_err_t rc522_flush_fifo_buffer(rc522_handle_t rc522);

esp_err_t rc522_soft_reset(rc522_handle_t rc522);

esp_err_t rc522_configure_timer(rc522_handle_t rc522, uint8_t mode, uint16_t prescaler_value);

esp_err_t rc522_set_timer_reload_value(rc522_handle_t rc522, uint16_t value);

#ifdef __cplusplus
}
#endif
