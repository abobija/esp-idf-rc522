#pragma once

#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "rc522_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    i2c_master_bus_handle_t bus;
    i2c_master_bus_config_t *bus_config; // Only required if `bus` is NULL
    i2c_device_config_t dev_config;

    /**
     * GPIO number of the RC522 RST pin.
     * Set to -1 if the RST pin is not connected.
     */
    gpio_num_t rst_io_num;
} rc522_i2c_config_t;

esp_err_t rc522_i2c_create(const rc522_i2c_config_t *config, rc522_driver_handle_t *driver);

#ifdef __cplusplus
}
#endif
