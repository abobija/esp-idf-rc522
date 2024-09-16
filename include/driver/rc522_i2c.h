#pragma once

#include "rc522_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t rc522_i2c_create(rc522_driver_config_t *config, rc522_driver_handle_t *driver);

esp_err_t rc522_i2c_destroy(rc522_driver_handle_t driver);

#ifdef __cplusplus
}
#endif
