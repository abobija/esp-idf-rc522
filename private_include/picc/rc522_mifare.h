#pragma once

#include "rc522_types.h"
#include "rc522_picc.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t rc522_mifare_dump_data_to_log(rc522_handle_t rc522, rc522_picc_t *picc, uint8_t *key, uint8_t key_length);

#ifdef __cplusplus
}
#endif
