#include <esp_system.h>
#include <esp_log.h>
#include <esp_check.h>
#include <string.h>
#include <sys/time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "rc522_types_internal.h"
#include "rc522_helpers_internal.h"

RC522_LOG_DEFINE_BASE();

uint32_t rc522_millis()
{
    struct timeval now;
    gettimeofday(&now, NULL);

    return (uint32_t)((now.tv_sec * 1000000 + now.tv_usec) / 1000);
}

void rc522_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

esp_err_t rc522_buffer_to_hex_str(
    const uint8_t *buffer, uint8_t buffer_length, char *str_buffer, uint8_t str_buffer_length)
{
    RC522_CHECK(buffer == NULL);
    RC522_CHECK(buffer_length < 1);
    RC522_CHECK(str_buffer == NULL);

    const char *format = "%02" RC522_X " ";
    const uint8_t formatted_length = 3;

    RC522_CHECK(str_buffer_length < (buffer_length * formatted_length));

    uint16_t len = 0;
    for (uint16_t i = 0; i < buffer_length; i++) {
        len += sprintf(str_buffer + len, format, buffer[i]);
    }

    str_buffer[len - 1] = 0x00;

    return ESP_OK;
}

inline esp_err_t rc522_nibbles(uint8_t byte, uint8_t *msb, uint8_t *lsb)
{
    RC522_CHECK(msb == NULL && lsb == NULL);

    if (msb) {
        *msb = byte >> 4;
    }

    if (lsb) {
        *lsb = byte & 0x0F;
    }

    return ESP_OK;
}
