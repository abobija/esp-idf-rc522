#pragma once

#include <driver/spi_master.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct rc522_driver_handle *rc522_driver_handle_t;

typedef struct
{
    union
    {
        struct
        {
            spi_host_device_t host_id;
            spi_bus_config_t *bus_config;
            spi_device_interface_config_t dev_config;
            spi_dma_chan_t dma_chan;
        } spi;

        // struct {

        // } i2c;
    };
} rc522_driver_config_t;

esp_err_t rc522_driver_install(rc522_driver_handle_t driver);

esp_err_t rc522_driver_send(rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length);

esp_err_t rc522_driver_receive(rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length);

esp_err_t rc522_driver_uninstall(rc522_driver_handle_t driver);

#ifdef __cplusplus
}
#endif
