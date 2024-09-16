#include <string.h>
#include "rc522_types_private.h"
#include "rc522_driver_private.h"
#include "driver/rc522_i2c.h"

// TODO: Migrate to new i2c API

RC522_LOG_DEFINE_BASE();

static esp_err_t rc522_i2c_install(rc522_driver_handle_t driver)
{
    ESP_RETURN_ON_FALSE(driver != NULL, ESP_ERR_INVALID_ARG, TAG, "driver is null");

    // TODO: Skip bus initialization if it's already configured by the user
    //       Do this once when we migrate to new i2c API
    RC522_RETURN_ON_ERROR(i2c_param_config(driver->config->i2c.port, &driver->config->i2c.config));

    RC522_RETURN_ON_ERROR(i2c_driver_install(driver->config->i2c.port, driver->config->i2c.config.mode, 0, 0, 0x00));

    return ESP_OK;
}

static esp_err_t rc522_i2c_send(rc522_driver_handle_t driver, uint8_t _address, uint8_t *buffer, uint8_t length)
{
    // ignore _address parameter since buffer[0] is address sent by library

    RC522_RETURN_ON_ERROR(i2c_master_write_to_device(driver->config->i2c.port,
        driver->config->i2c.device_address,
        buffer,
        length,
        pdMS_TO_TICKS(driver->config->i2c.rw_timeout_ms)));

    return ESP_OK;
}

static esp_err_t rc522_i2c_receive(rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length)
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

static esp_err_t rc522_i2c_uninstall(rc522_driver_handle_t driver)
{
    ESP_RETURN_ON_FALSE(driver != NULL, ESP_ERR_INVALID_ARG, TAG, "driver is null");

    RC522_RETURN_ON_ERROR(i2c_driver_delete(driver->config->i2c.port));

    return ESP_OK;
}

esp_err_t rc522_i2c_create(rc522_driver_config_t *config, rc522_driver_handle_t *driver)
{
    RC522_RETURN_ON_ERROR(rc522_driver_create(config, driver));

    (*driver)->install = rc522_i2c_install;
    (*driver)->send = rc522_i2c_send;
    (*driver)->receive = rc522_i2c_receive;
    (*driver)->uninstall = rc522_i2c_uninstall;

    return ESP_OK;
}

inline esp_err_t rc522_i2c_destroy(rc522_driver_handle_t driver)
{
    return rc522_driver_destroy(driver);
}
