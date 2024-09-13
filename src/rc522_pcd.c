#include <esp_system.h>
#include <esp_check.h>
#include <string.h>

#include "rc522_pcd_private.h"
#include "rc522_registers.h"

RC522_LOG_DEFINE_BASE();

esp_err_t rc522_pcd_calculate_crc(rc522_handle_t rc522, uint8_t *data, uint8_t n, uint8_t *buffer)
{
    ESP_RETURN_ON_ERROR(rc522_pcd_stop_active_command(rc522), TAG, "");
    ESP_RETURN_ON_ERROR(rc522_pcd_clear_bitmask(rc522, RC522_DIV_INT_REQ_REG, RC522_CRC_IRQ), TAG, "");
    ESP_RETURN_ON_ERROR(rc522_pcd_fifo_flush(rc522), TAG, "");
    ESP_RETURN_ON_ERROR(rc522_pcd_write_n(rc522, RC522_FIFO_DATA_REG, n, data), TAG, "");
    ESP_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_COMMAND_REG, RC522_CMD_CALC_CRC), TAG, "");

    uint32_t deadline_ms = rc522_millis() + 90;
    bool calculation_done = false;

    do {
        uint8_t irq;
        ESP_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_DIV_INT_REQ_REG, &irq), TAG, "");

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

    ESP_RETURN_ON_ERROR(rc522_pcd_stop_active_command(rc522), TAG, "");
    ESP_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_CRC_RESULT_LSB_REG, buffer), TAG, "");
    ESP_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_CRC_RESULT_MSB_REG, buffer + 1), TAG, "");

    return ESP_OK;
}

esp_err_t rc522_pcd_init(rc522_handle_t rc522)
{
    // TODO: Implement hard reset via RST pin
    //       and ability to choose between hard and soft reset

    RC522_RETURN_ON_ERROR(rc522_pcd_soft_reset(rc522, 150));

    // Reset baud rates
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_TX_MODE_REG, 0x00));
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_RX_MODE_REG, 0x00));

    // Reset modulation width
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_MOD_WIDTH_REG, RC522_MOD_WIDTH_RESET_VALUE));

    // When communicating with a PICC we need a timeout if something goes wrong.
    // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
    // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.

    // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_TIMER_MODE_REG, RC522_T_AUTO));

    // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_TIMER_PRESCALER_REG, 0xA9));

    // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_TIMER_RELOAD_MSB_REG, 0x03));
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_TIMER_RELOAD_LSB_REG, 0xE8));

    // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_TX_ASK_REG, 0x40));

    // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3
    // part 6.2.4)
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_MODE_REG, 0x3D));

    // Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
    RC522_RETURN_ON_ERROR(rc522_pcd_antenna_on(rc522));

    return ESP_OK;
}

esp_err_t rc522_pcd_soft_reset(rc522_handle_t rc522, uint32_t timeout_ms)
{
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_COMMAND_REG, RC522_CMD_SOFT_RESET));

    bool power_down_bit = true;
    uint32_t start_ms = rc522_millis();

    // Wait for the PowerDown bit in CommandReg to be cleared
    do {
        rc522_delay_ms(25);
        taskYIELD();

        uint8_t cmd;
        RC522_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_COMMAND_REG, &cmd));

        if (!(power_down_bit = (cmd & RC522_POWER_DOWN))) {
            break;
        }
    }
    while ((rc522_millis() - start_ms) < timeout_ms);

    return power_down_bit ? ESP_ERR_TIMEOUT : ESP_OK;
}

esp_err_t rc522_pcd_antenna_on(rc522_handle_t rc522)
{
    uint8_t value;
    RC522_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_TX_CONTROL_REG, &value));

    if ((value & 0x03) != 0x03) {
        RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_TX_CONTROL_REG, value | 0x03));
    }

    return ESP_OK;
}

esp_err_t rc522_pcd_firmware(rc522_handle_t rc522, rc522_pcd_firmware_t *fw)
{
    ESP_RETURN_ON_FALSE(fw != NULL, ESP_ERR_INVALID_ARG, TAG, "fw is null");

    uint8_t value;
    RC522_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_VERSION_REG, &value));

    *fw = (rc522_pcd_firmware_t)value;
    return ESP_OK;
}

char *rc522_pcd_firmware_name(rc522_pcd_firmware_t firmware)
{
    switch (firmware) {
        case RC522_PCD_FIRMWARE_CLONE:
            return "clone";
        case RC522_PCD_FIRMWARE_00:
            return "v0.0";
        case RC522_PCD_FIRMWARE_10:
            return "v1.0";
        case RC522_PCD_FIRMWARE_20:
            return "v2.0";
        case RC522_PCD_FIRMWARE_COUNTERFEIT:
            return "counterfeit_chip";
    }

    return "unknown";
}

inline esp_err_t rc522_pcd_stop_active_command(rc522_handle_t rc522)
{
    return rc522_pcd_write(rc522, RC522_COMMAND_REG, RC522_CMD_IDLE);
}

inline esp_err_t rc522_pcd_fifo_write(rc522_handle_t rc522, uint8_t *data, uint8_t data_length)
{
    return rc522_pcd_write_n(rc522, RC522_FIFO_DATA_REG, data_length, data);
}

inline esp_err_t rc522_pcd_fifo_flush(rc522_handle_t rc522)
{
    return rc522_pcd_write(rc522, RC522_FIFO_LEVEL_REG, RC522_FLUSH_BUFFER);
}

inline esp_err_t rc522_pcd_start_data_transmission(rc522_handle_t rc522)
{
    return rc522_pcd_set_bitmask(rc522, RC522_BIT_FRAMING_REG, RC522_START_SEND);
}

inline esp_err_t rc522_pcd_stop_data_transmission(rc522_handle_t rc522)
{
    return rc522_pcd_clear_bitmask(rc522, RC522_BIT_FRAMING_REG, RC522_START_SEND);
}

esp_err_t rc522_pcd_rw_test(rc522_handle_t rc522)
{
    uint8_t tmp;

    ESP_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_FIFO_LEVEL_REG, &tmp), TAG, "Cannot read FIFO length");

    ESP_RETURN_ON_ERROR(rc522_pcd_fifo_flush(rc522), TAG, "Cannot flush FIFO");

    uint8_t buffer1[] = { 0x13, 0x33, 0x37 };
    const uint8_t buffer_size = sizeof(buffer1);
    uint8_t buffer2[buffer_size];

    ESP_RETURN_ON_ERROR(rc522_pcd_write_n(rc522, RC522_FIFO_DATA_REG, buffer_size, buffer1),
        TAG,
        "Cannot write to FIFO");

    RC522_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_FIFO_LEVEL_REG, &tmp));

    ESP_RETURN_ON_FALSE(tmp == buffer_size, ESP_FAIL, TAG, "FIFO length missmatch after write");

    RC522_RETURN_ON_ERROR(rc522_pcd_read_n(rc522, RC522_FIFO_DATA_REG, buffer_size, buffer2));

    bool buffers_content_equal = true;
    for (uint8_t i = 0; i < buffer_size; i++) {
        if (buffer1[i] != buffer2[i]) {
            buffers_content_equal = false;
            break;
        }
    }

    if (!buffers_content_equal) {
        RC522_LOGE("Buffers content missmatch");

        RC522_LOGE("Buffer1: ");
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, buffer1, buffer_size, ESP_LOG_ERROR);
        RC522_LOGE("Buffer2: ");
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, buffer2, buffer_size, ESP_LOG_ERROR);

        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t rc522_pcd_write_n(rc522_handle_t rc522, uint8_t addr, uint8_t n, uint8_t *data)
{
    if (n > 1) {
        RC522_LOGV("\t[0x%02x] <=", addr);
        ESP_LOG_BUFFER_HEX_LEVEL(RC522_LOG_TAG, data, n, ESP_LOG_VERBOSE);
    }
    else {
        RC522_LOGV("\t[0x%02x] <= 0x%02x", addr, *data);
    }

    uint8_t *buffer = NULL;

    // TODO: Find a way to send address + data without memory allocation
    buffer = (uint8_t *)malloc(n + 1);
    ESP_RETURN_ON_FALSE(buffer != NULL, ESP_ERR_NO_MEM, TAG, "No memory");

    buffer[0] = addr;
    memcpy(buffer + 1, data, n);

    esp_err_t ret = rc522->config->send_handler(buffer, n + 1);

    FREE(buffer);

    return ret;
}

inline esp_err_t rc522_pcd_write(rc522_handle_t rc522, uint8_t addr, uint8_t val)
{
    return rc522_pcd_write_n(rc522, addr, 1, &val);
}

esp_err_t rc522_pcd_read_n(rc522_handle_t rc522, uint8_t addr, uint8_t n, uint8_t *buffer)
{
    esp_err_t ret = rc522->config->receive_handler(addr, buffer, n);

    if (n > 1) {
        RC522_LOGV("\t[0x%02x] =>", addr);
        ESP_LOG_BUFFER_HEX_LEVEL(RC522_LOG_TAG, buffer, n, ESP_LOG_VERBOSE);
    }
    else {
        RC522_LOGV("\t[0x%02x] => 0x%02x", addr, *buffer);
    }

    return ret;
}

inline esp_err_t rc522_pcd_read(rc522_handle_t rc522, uint8_t addr, uint8_t *value_ref)
{
    return rc522_pcd_read_n(rc522, addr, 1, value_ref);
}

esp_err_t rc522_pcd_set_bitmask(rc522_handle_t rc522, uint8_t addr, uint8_t mask)
{
    uint8_t value;
    ESP_RETURN_ON_ERROR(rc522_pcd_read(rc522, addr, &value), TAG, "");

    return rc522_pcd_write(rc522, addr, value | mask);
}

esp_err_t rc522_pcd_clear_bitmask(rc522_handle_t rc522, uint8_t addr, uint8_t mask)
{
    uint8_t value;
    ESP_RETURN_ON_ERROR(rc522_pcd_read(rc522, addr, &value), TAG, "");

    return rc522_pcd_write(rc522, addr, value & ~mask);
}

esp_err_t rc522_pcd_write_map(rc522_handle_t rc522, const uint8_t map[][2], uint8_t map_length)
{
    for (uint8_t i = 0; i < map_length; i++) {
        const uint8_t address = map[i][0];
        const uint8_t value = map[i][1];

        ESP_RETURN_ON_ERROR(rc522_pcd_write(rc522, address, value),
            TAG,
            "Failed to write %d into 0x%20X register",
            value,
            address);
    }

    return ESP_OK;
}
