#include <string.h>
#include "rc522_types_private.h"
#include "rc522_driver_private.h"
#include "driver/rc522_spi.h"

RC522_LOG_DEFINE_BASE();

static esp_err_t rc522_spi_install(rc522_driver_handle_t driver)
{
    ESP_RETURN_ON_FALSE(driver != NULL, ESP_ERR_INVALID_ARG, TAG, "driver is null");

    if (driver->config->spi.bus_config) {
        RC522_RETURN_ON_ERROR(spi_bus_initialize(driver->config->spi.host_id,
            driver->config->spi.bus_config,
            driver->config->spi.dma_chan));
    }

    RC522_RETURN_ON_ERROR(spi_bus_add_device(driver->config->spi.host_id,
        &driver->config->spi.dev_config,
        (spi_device_handle_t *)(&driver->device)));

    return ESP_OK;
}

static esp_err_t rc522_spi_send(rc522_driver_handle_t driver, uint8_t _address, uint8_t *buffer, uint8_t length)
{
    // ignore _address parameter since buffer[0] is address sent by library

    uint8_t *address = (buffer + 0);
    uint8_t address_origin = *address;

    *address <<= 1;
    *address &= (~0x80);

    esp_err_t ret = spi_device_polling_transmit((spi_device_handle_t)(driver->device),
        &(spi_transaction_t) {
            .length = 8 * length,
            .tx_buffer = buffer,
        });

    *address = address_origin;

    return ret;
}

static esp_err_t rc522_spi_receive(rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length)
{
    address <<= 1;
    address |= 0x80;

    RC522_RETURN_ON_ERROR(spi_device_acquire_bus((spi_device_handle_t)(driver->device), portMAX_DELAY));

    for (uint8_t i = 0; i < length; i++) {
        RC522_RETURN_ON_ERROR(spi_device_polling_transmit((spi_device_handle_t)(driver->device),
            &(spi_transaction_t) {
                .length = 8,
                .tx_buffer = &address,
                .rxlength = 8,
                .rx_buffer = (buffer + i),
            }));
    }

    spi_device_release_bus((spi_device_handle_t)(driver->device));

    return ESP_OK;
}

static esp_err_t rc522_spi_uninstall(rc522_driver_handle_t driver)
{
    ESP_RETURN_ON_FALSE(driver != NULL, ESP_ERR_INVALID_ARG, TAG, "driver is null");

    RC522_RETURN_ON_ERROR(spi_bus_remove_device((spi_device_handle_t)(driver->device)));
    driver->device = NULL;

    if (driver->config->spi.bus_config) {
        RC522_RETURN_ON_ERROR(spi_bus_free(driver->config->spi.host_id));
    }

    return ESP_OK;
}

esp_err_t rc522_spi_create(rc522_driver_config_t *config, rc522_driver_handle_t *driver)
{
    RC522_RETURN_ON_ERROR(rc522_driver_create(config, driver));

    (*driver)->install = rc522_spi_install;
    (*driver)->send = rc522_spi_send;
    (*driver)->receive = rc522_spi_receive;
    (*driver)->uninstall = rc522_spi_uninstall;

    return ESP_OK;
}

inline esp_err_t rc522_spi_destroy(rc522_driver_handle_t driver)
{
    return rc522_driver_destroy(driver);
}
