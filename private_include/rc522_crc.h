#pragma once

#include "rc522_private.h"

#ifdef __cplusplus
extern "C" {
#endif

// Buffer should be at least 2 bytes long
// Only first 2 elements will be used where the result will be stored
// TODO: Use uint16_t type for the result instead of buffer array?
esp_err_t rc522_calculate_crc(rc522_handle_t rc522, uint8_t *data, uint8_t n, uint8_t *buffer);

#ifdef __cplusplus
}
#endif
