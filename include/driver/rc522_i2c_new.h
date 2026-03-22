#pragma once

#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "rc522_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    i2c_master_bus_handle_t i2c_bus_handle;
    const i2c_master_bus_config_t i2c_bus_config; // Only required if i2c_bus_handle is null

    i2c_device_config_t i2c_device_config;
    gpio_num_t rst_io_num;
} rc522_new_i2c_config_t;

esp_err_t rc522_new_i2c_create(const rc522_new_i2c_config_t *config, rc522_driver_handle_t *driver);

#ifdef __cplusplus
}
#endif
