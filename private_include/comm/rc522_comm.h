#pragma once

#include "rc522_private.h"
#include "rc522_registers.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t rc522_picc_find(rc522_handle_t rc522, rc522_picc_t *picc);

esp_err_t rc522_picc_fetch(rc522_handle_t rc522, rc522_picc_t *picc);

#ifdef __cplusplus
}
#endif
