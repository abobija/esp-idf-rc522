#pragma once

#include <driver/spi_master.h>
#include "rc522_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    spi_host_device_t host_id;
    spi_bus_config_t *bus_config;
    spi_device_interface_config_t dev_config;
    spi_dma_chan_t dma_chan;
} rc522_spi_config_t;

esp_err_t rc522_spi_create(rc522_spi_config_t *config, rc522_driver_handle_t *driver);

esp_err_t rc522_spi_destroy(rc522_driver_handle_t driver);

#ifdef __cplusplus
}
#endif
