#pragma once

#include <esp_err.h>
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t rc522_millis();

void rc522_delay_ms(uint32_t ms);

esp_err_t rc522_buffer_to_hex_str(
    const uint8_t *buffer, uint8_t buffer_length, char *str_buffer, uint8_t str_buffer_length);

esp_err_t rc522_nibbles(uint8_t byte, uint8_t *msb, uint8_t *lsb);

#ifdef __cplusplus
}
#endif
