#include <esp_system.h>
#include <esp_check.h>
#include <string.h>

#include "comm/rc522_comm.h"
#include "rc522_io.h"

static const char *TAG = "rc522_comm";

inline esp_err_t rc522_firmware(rc522_handle_t rc522, uint8_t *result)
{
    uint8_t value;
    ESP_RETURN_ON_ERROR(rc522_read(rc522, RC522_VERSION_REG, &value), TAG, "");

    *result = value & 0x03;
    return ESP_OK;
}

inline esp_err_t rc522_antenna_on(rc522_handle_t rc522, rc522_rx_gain_t gain)
{
    ESP_RETURN_ON_ERROR(rc522_set_bitmask(rc522, RC522_TX_CONTROL_REG, RC522_TX1_RF_EN | RC522_TX2_RF_EN), TAG, "");

    return rc522_write(rc522, RC522_RF_CFG_REG, gain);
}

inline esp_err_t rc522_stop_active_command(rc522_handle_t rc522)
{
    return rc522_write(rc522, RC522_COMMAND_REG, RC522_CMD_IDLE);
}

inline esp_err_t rc522_flush_fifo_buffer(rc522_handle_t rc522)
{
    return rc522_set_bitmask(rc522, RC522_FIFO_LEVEL_REG, RC522_FLUSH_BUFFER);
}

inline esp_err_t rc522_configure_timer(rc522_handle_t rc522, uint8_t mode, uint16_t prescaler_value)
{
    return rc522_write_map(rc522,
        (uint8_t[][2]) {
            { RC522_TIMER_MODE_REG, (uint8_t)((mode & 0xF0) | ((prescaler_value >> 8) & 0x0F)) },
            { RC522_TIMER_PRESCALER_REG, (uint8_t)prescaler_value },
        },
        2);
}

inline esp_err_t rc522_set_timer_reload_value(rc522_handle_t rc522, uint16_t value)
{
    return rc522_write_map(rc522,
        (uint8_t[][2]) {
            { RC522_TIMER_RELOAD_MSB_REG, (uint8_t)(value >> 8) },
            { RC522_TIMER_RELOAD_LSB_REG, (uint8_t)value },
        },
        2);
}
