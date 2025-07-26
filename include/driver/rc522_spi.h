#pragma once

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "rc522_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC522_SPI_WRITE (0)
#define RC522_SPI_READ  (1)

typedef struct
{
    spi_host_device_t host_id;
    spi_bus_config_t *bus_config;
    spi_device_interface_config_t dev_config;
    spi_dma_chan_t dma_chan;

    /**
     * GPIO number of the RC522 RST pin.
     * Set to -1 if the RST pin is not connected.
     */
    gpio_num_t rst_io_num;
} rc522_spi_config_t;

esp_err_t rc522_spi_create(const rc522_spi_config_t *config, rc522_driver_handle_t *driver);

#ifdef __cplusplus
}
#endif
