#include <esp_system.h>
#include <esp_check.h>
#include <string.h>

#include "rc522_crc.h"
#include "rc522_io.h"
#include "comm/rc522_comm.h"
#include "rc522_registers.h"

static const char *TAG = "rc522_crc";

esp_err_t rc522_calculate_crc(rc522_handle_t rc522, uint8_t *data, uint8_t n, uint8_t *buffer)
{
    ESP_RETURN_ON_ERROR(rc522_stop_active_command(rc522), TAG, "");
    ESP_RETURN_ON_ERROR(rc522_clear_bitmask(rc522, RC522_DIV_INT_REQ_REG, RC522_CRC_IRQ), TAG, "");
    ESP_RETURN_ON_ERROR(rc522_flush_fifo_buffer(rc522), TAG, "");
    ESP_RETURN_ON_ERROR(rc522_write_n(rc522, RC522_FIFO_DATA_REG, n, data), TAG, "");
    ESP_RETURN_ON_ERROR(rc522_write(rc522, RC522_COMMAND_REG, RC522_CMD_CALC_CRC), TAG, "");

    uint32_t deadline_ms = rc522_millis() + 90;
    bool calculation_done = false;

    do {
        uint8_t irq;
        ESP_RETURN_ON_ERROR(rc522_read(rc522, RC522_DIV_INT_REQ_REG, &irq), TAG, "");

        if (RC522_CRC_IRQ & irq) {
            calculation_done = true;
            break;
        }

        taskYIELD();
    }
    while (rc522_millis() < deadline_ms);

    if (!calculation_done) { // Deadline reached
        return ESP_ERR_TIMEOUT;
    }

    ESP_RETURN_ON_ERROR(rc522_stop_active_command(rc522), TAG, "");
    ESP_RETURN_ON_ERROR(rc522_read(rc522, RC522_CRC_RESULT_LSB_REG, buffer), TAG, "");
    ESP_RETURN_ON_ERROR(rc522_read(rc522, RC522_CRC_RESULT_MSB_REG, buffer + 1), TAG, "");

    return ESP_OK;
}
