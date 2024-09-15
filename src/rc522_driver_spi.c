#include <string.h>
#include "rc522_types_private.h"
#include "rc522_driver_private.h"
#include "rc522_driver_spi.h"

RC522_LOG_DEFINE_BASE();

static esp_err_t rc522_driver_spi_install(rc522_driver_handle_t driver)
{
    ESP_RETURN_ON_FALSE(driver != NULL, ESP_ERR_INVALID_ARG, TAG, "driver is null");

    if (driver->config->spi.bus_config) {
        RC522_RETURN_ON_ERROR(spi_bus_initialize(driver->config->spi.host_id,
            driver->config->spi.bus_config,
            driver->config->spi.dma_chan));
    }

    RC522_RETURN_ON_ERROR(
        spi_bus_add_device(driver->config->spi.host_id, &driver->config->spi.dev_config, &driver->spi));

    return ESP_OK;
}

static esp_err_t rc522_driver_spi_send(rc522_driver_handle_t driver, uint8_t _address, uint8_t *buffer, uint8_t length)
{
    // ignore _address parameter since buffer[0] is address sent by library

    uint8_t *address = (buffer + 0);
    uint8_t address_origin = *address;

    *address <<= 1;
    *address &= (~0x80);

    esp_err_t ret = spi_device_polling_transmit(driver->spi,
        &(spi_transaction_t) {
            .length = 8 * length,
            .tx_buffer = buffer,
        });

    *address = address_origin;

    return ret;
}

static esp_err_t rc522_driver_spi_receive(
    rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length)
{
    address <<= 1;
    address |= 0x80;

    RC522_RETURN_ON_ERROR(spi_device_acquire_bus(driver->spi, portMAX_DELAY));

    for (uint8_t i = 0; i < length; i++) {
        RC522_RETURN_ON_ERROR(spi_device_polling_transmit(driver->spi,
            &(spi_transaction_t) {
                .length = 8,
                .tx_buffer = &address,
                .rxlength = 8,
                .rx_buffer = (buffer + i),
            }));
    }

    spi_device_release_bus(driver->spi);

    return ESP_OK;
}

static esp_err_t rc522_driver_spi_uninstall(rc522_driver_handle_t driver)
{
    ESP_RETURN_ON_FALSE(driver != NULL, ESP_ERR_INVALID_ARG, TAG, "driver is null");

    RC522_RETURN_ON_ERROR(spi_bus_remove_device(driver->spi));
    driver->spi = NULL;

    if (driver->config->spi.bus_config) {
        RC522_RETURN_ON_ERROR(spi_bus_free(driver->config->spi.host_id));
    }

    return ESP_OK;
}

esp_err_t rc522_driver_spi_create(rc522_driver_config_t *config, rc522_driver_handle_t *driver)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config is null");
    ESP_RETURN_ON_FALSE(driver != NULL, ESP_ERR_INVALID_ARG, TAG, "driver is null");

    esp_err_t ret;

    rc522_driver_handle_t _driver = calloc(1, sizeof(struct rc522_driver_handle));
    ESP_RETURN_ON_FALSE(_driver != NULL, ESP_ERR_NO_MEM, TAG, "no mem");

    _driver->config = calloc(1, sizeof(rc522_driver_config_t));
    ESP_GOTO_ON_FALSE(_driver->config != NULL, ESP_ERR_NO_MEM, error, TAG, "no mem");

    memcpy(_driver->config, config, sizeof(rc522_driver_config_t));

    _driver->install = rc522_driver_spi_install;
    _driver->send = rc522_driver_spi_send;
    _driver->receive = rc522_driver_spi_receive;
    _driver->uninstall = rc522_driver_spi_uninstall;

    goto success;
error:
    rc522_driver_spi_destroy(_driver);
    goto exit;
success:
    *driver = _driver;
exit:
    return ret;
}

esp_err_t rc522_driver_spi_destroy(rc522_driver_handle_t driver)
{
    ESP_RETURN_ON_FALSE(driver != NULL, ESP_ERR_INVALID_ARG, TAG, "driver is null");

    FREE(driver->config);
    FREE(driver);

    return ESP_OK;
}
