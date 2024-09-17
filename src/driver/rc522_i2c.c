#include <string.h>
#include "rc522_types_private.h"
#include "rc522_driver_private.h"
#include "driver/rc522_i2c.h"

RC522_LOG_DEFINE_BASE();

static esp_err_t rc522_i2c_install(rc522_driver_handle_t driver)
{
    ESP_RETURN_ON_FALSE(driver != NULL, ESP_ERR_INVALID_ARG, TAG, "driver is null");

    rc522_i2c_config_t *conf = (rc522_i2c_config_t *)(driver->config);

    // TODO: Skip bus initialization if it's already configured by the user
    //       Do this once when we migrate to new i2c API
    RC522_RETURN_ON_ERROR(i2c_param_config(conf->port, &conf->config));

    RC522_RETURN_ON_ERROR(i2c_driver_install(conf->port, conf->config.mode, 0, 0, 0x00));

    return ESP_OK;
}

static esp_err_t rc522_i2c_send(rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length)
{
    // FIXME: Find a way to send [address + buffer]
    //        without need for second buffer
    uint8_t buffer2[64];

    buffer2[0] = address;
    memcpy(buffer2 + 1, buffer, length);

    rc522_i2c_config_t *conf = (rc522_i2c_config_t *)(driver->config);

    RC522_RETURN_ON_ERROR(i2c_master_write_to_device(conf->port,
        conf->device_address,
        buffer2,
        (length + 1),
        pdMS_TO_TICKS(conf->rw_timeout_ms)));

    return ESP_OK;
}

static esp_err_t rc522_i2c_receive(rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length)
{
    rc522_i2c_config_t *conf = (rc522_i2c_config_t *)(driver->config);

    RC522_RETURN_ON_ERROR(i2c_master_write_read_device(conf->port,
        conf->device_address,
        &address,
        1,
        buffer,
        length,
        pdMS_TO_TICKS(conf->rw_timeout_ms)));

    return ESP_OK;
}

static esp_err_t rc522_i2c_uninstall(rc522_driver_handle_t driver)
{
    ESP_RETURN_ON_FALSE(driver != NULL, ESP_ERR_INVALID_ARG, TAG, "driver is null");

    rc522_i2c_config_t *conf = (rc522_i2c_config_t *)(driver->config);

    RC522_RETURN_ON_ERROR(i2c_driver_delete(conf->port));

    return ESP_OK;
}

esp_err_t rc522_i2c_create(rc522_i2c_config_t *config, rc522_driver_handle_t *driver)
{
    RC522_RETURN_ON_ERROR(rc522_driver_create(config, sizeof(rc522_i2c_config_t), driver));

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
