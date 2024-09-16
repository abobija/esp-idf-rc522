#pragma once

#include "rc522_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t rc522_spi_create(rc522_driver_config_t *config, rc522_driver_handle_t *driver);

esp_err_t rc522_spi_destroy(rc522_driver_handle_t driver);

#ifdef __cplusplus
}
#endif
