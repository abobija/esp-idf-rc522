#include <string.h>
#include "rc522_types_private.h"
#include "rc522_driver_private.h"
#include "rc522_driver_i2c.h"

RC522_LOG_DEFINE_BASE();

static esp_err_t rc522_driver_i2c_install(rc522_driver_handle_t driver)
{
    ESP_RETURN_ON_FALSE(driver != NULL, ESP_ERR_INVALID_ARG, TAG, "driver is null");

    RC522_RETURN_ON_ERROR(i2c_param_config(driver->config->i2c.port, &driver->config->i2c.config));

    RC522_RETURN_ON_ERROR(i2c_driver_install(driver->config->i2c.port, driver->config->i2c.config.mode, 0, 0, 0x00));

    return ESP_OK;
}

static esp_err_t rc522_driver_i2c_send(rc522_driver_handle_t driver, uint8_t _address, uint8_t *buffer, uint8_t length)
{
    // ignore _address parameter since buffer[0] is address sent by library

    RC522_RETURN_ON_ERROR(i2c_master_write_to_device(driver->config->i2c.port,
        driver->config->i2c.device_address,
        buffer,
        length,
        pdMS_TO_TICKS(driver->config->i2c.rw_timeout_ms)));

    return ESP_OK;
}

static esp_err_t rc522_driver_i2c_receive(
    rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length)
{
    RC522_RETURN_ON_ERROR(i2c_master_write_read_device(driver->config->i2c.port,
        driver->config->i2c.device_address,
        &address,
        1,
        buffer,
        length,
        pdMS_TO_TICKS(driver->config->i2c.rw_timeout_ms)));

    return ESP_OK;
}

static esp_err_t rc522_driver_i2c_uninstall(rc522_driver_handle_t driver)
{
    ESP_RETURN_ON_FALSE(driver != NULL, ESP_ERR_INVALID_ARG, TAG, "driver is null");

    RC522_RETURN_ON_ERROR(spi_bus_remove_device(driver->spi));
    driver->spi = NULL;

    if (driver->config->spi.bus_config) {
        RC522_RETURN_ON_ERROR(spi_bus_free(driver->config->spi.host_id));
    }

    return ESP_OK;
}

esp_err_t rc522_driver_i2c_create(rc522_driver_config_t *config, rc522_driver_handle_t *driver)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config is null");
    ESP_RETURN_ON_FALSE(driver != NULL, ESP_ERR_INVALID_ARG, TAG, "driver is null");

    esp_err_t ret;

    rc522_driver_handle_t _driver = calloc(1, sizeof(struct rc522_driver_handle));
    ESP_RETURN_ON_FALSE(_driver != NULL, ESP_ERR_NO_MEM, TAG, "no mem");

    _driver->config = calloc(1, sizeof(rc522_driver_config_t));
    ESP_GOTO_ON_FALSE(_driver->config != NULL, ESP_ERR_NO_MEM, error, TAG, "no mem");

    memcpy(_driver->config, config, sizeof(rc522_driver_config_t));

    _driver->install = rc522_driver_i2c_install;
    _driver->send = rc522_driver_i2c_send;
    _driver->receive = rc522_driver_i2c_receive;
    _driver->uninstall = rc522_driver_i2c_uninstall;

    goto success;
error:
    rc522_driver_i2c_destroy(_driver);
    goto exit;
success:
    *driver = _driver;
exit:
    return ret;
}

esp_err_t rc522_driver_i2c_destroy(rc522_driver_handle_t driver)
{
    ESP_RETURN_ON_FALSE(driver != NULL, ESP_ERR_INVALID_ARG, TAG, "driver is null");

    FREE(driver->config);
    FREE(driver);

    return ESP_OK;
}
