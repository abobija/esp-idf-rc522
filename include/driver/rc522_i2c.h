#pragma once

#include <driver/i2c.h> // TODO: Migrate to new i2c API
#include "rc522_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    i2c_config_t config;
    i2c_port_t port;
    uint8_t device_address;
    uint32_t rw_timeout_ms;
} rc522_i2c_config_t;

esp_err_t rc522_i2c_create(rc522_i2c_config_t *config, rc522_driver_handle_t *driver);

#ifdef __cplusplus
}
#endif
