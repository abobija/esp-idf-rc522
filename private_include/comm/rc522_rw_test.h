#pragma once

#include "rc522_private.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t rc522_rw_test(rc522_handle_t rc522, uint8_t test_register, uint8_t times);

#ifdef __cplusplus
}
#endif
