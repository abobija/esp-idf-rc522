#include <esp_system.h>
#include <esp_log.h>
#include <esp_check.h>
#include <string.h>
#include <sys/time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "rc522_helpers_private.h"

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

void rc522_buffer_to_hex_str(uint8_t *buffer, uint8_t buffer_length, char *str_buffer, uint8_t str_buffer_length)
{
    const char *format = "%02x ";
    const uint8_t formatted_length = 3;
    uint8_t length = 0;

    for (uint8_t i = 0; i < buffer_length; i++) {
        // TODO: Check buffer overflow using str_buffer_length
        sprintf(str_buffer + (i * formatted_length), format, buffer[i]);
        length += formatted_length;
    }

    str_buffer[length - 1] = 0x00;
}
