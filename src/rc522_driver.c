#include <string.h>
#include "rc522_types_private.h"
#include "rc522_driver_private.h"

RC522_LOG_DEFINE_BASE();

inline esp_err_t rc522_driver_install(rc522_driver_handle_t driver)
{
    return driver->install(driver);
}

inline esp_err_t rc522_driver_send(rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length)
{
    return driver->send(driver, address, buffer, length);
}

inline esp_err_t rc522_driver_receive(rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length)
{
    return driver->receive(driver, address, buffer, length);
}

inline esp_err_t rc522_driver_uninstall(rc522_driver_handle_t driver)
{
    return driver->uninstall(driver);
}

esp_err_t rc522_driver_create(rc522_driver_config_t *config, rc522_driver_handle_t *driver)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config is null");
    ESP_RETURN_ON_FALSE(driver != NULL, ESP_ERR_INVALID_ARG, TAG, "driver is null");

    esp_err_t ret = ESP_OK;

    rc522_driver_handle_t _driver = calloc(1, sizeof(struct rc522_driver_handle));
    ESP_RETURN_ON_FALSE(_driver != NULL, ESP_ERR_NO_MEM, TAG, "no mem");

    _driver->config = calloc(1, sizeof(rc522_driver_config_t));
    ESP_GOTO_ON_FALSE(_driver->config != NULL, ESP_ERR_NO_MEM, error, TAG, "no mem");

    memcpy(_driver->config, config, sizeof(rc522_driver_config_t));

    goto success;
error:
    rc522_driver_destroy(_driver);
    goto exit;
success:
    *driver = _driver;
exit:
    return ret;
}

esp_err_t rc522_driver_destroy(rc522_driver_handle_t driver)
{
    ESP_RETURN_ON_FALSE(driver != NULL, ESP_ERR_INVALID_ARG, TAG, "driver is null");

    FREE(driver->config);

    driver->install = NULL;
    driver->send = NULL;
    driver->receive = NULL;
    driver->uninstall = NULL;

    driver->spi = NULL;

    FREE(driver);

    return ESP_OK;
}
