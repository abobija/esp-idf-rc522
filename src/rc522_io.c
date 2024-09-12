#include <esp_system.h>
#include <esp_check.h>
#include <string.h>

#include "rc522_io.h"

RC522_LOG_DEFINE_BASE();

esp_err_t rc522_write_n(rc522_handle_t rc522, uint8_t addr, uint8_t n, uint8_t *data)
{
    if (n > 1) {
        RC522_LOGD("Write %d byte(s) into 0x%02x", n, addr);
        ESP_LOG_BUFFER_HEX_LEVEL(RC522_LOG_TAG, data, n, ESP_LOG_DEBUG);
    }
    else {
        RC522_LOGD("Write 0x%02x into 0x%02x", *data, addr);
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

inline esp_err_t rc522_write(rc522_handle_t rc522, uint8_t addr, uint8_t val)
{
    return rc522_write_n(rc522, addr, 1, &val);
}

esp_err_t rc522_read_n(rc522_handle_t rc522, uint8_t addr, uint8_t n, uint8_t *buffer)
{
    RC522_LOGD("Read %d byte(s) from 0x%02x", n, addr);

    esp_err_t ret = rc522->config->receive_handler(addr, buffer, n);

    ESP_LOG_BUFFER_HEX_LEVEL(RC522_LOG_TAG, buffer, n, ESP_LOG_DEBUG);

    return ret;
}

inline esp_err_t rc522_read(rc522_handle_t rc522, uint8_t addr, uint8_t *value_ref)
{
    return rc522_read_n(rc522, addr, 1, value_ref);
}

esp_err_t rc522_set_bitmask(rc522_handle_t rc522, uint8_t addr, uint8_t mask)
{
    uint8_t value;
    ESP_RETURN_ON_ERROR(rc522_read(rc522, addr, &value), TAG, "");

    return rc522_write(rc522, addr, value | mask);
}

esp_err_t rc522_clear_bitmask(rc522_handle_t rc522, uint8_t addr, uint8_t mask)
{
    uint8_t value;
    ESP_RETURN_ON_ERROR(rc522_read(rc522, addr, &value), TAG, "");

    return rc522_write(rc522, addr, value & ~mask);
}

esp_err_t rc522_write_map(rc522_handle_t rc522, const uint8_t map[][2], uint8_t map_length)
{
    for (uint8_t i = 0; i < map_length; i++) {
        const uint8_t address = map[i][0];
        const uint8_t value = map[i][1];

        ESP_RETURN_ON_ERROR(rc522_write(rc522, address, value),
            TAG,
            "Failed to write %d into 0x%20X register",
            value,
            address);
    }

    return ESP_OK;
}
