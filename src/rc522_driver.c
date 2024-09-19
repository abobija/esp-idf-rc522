#include <string.h>
#include <driver/gpio.h>
#include "rc522_types_private.h"
#include "rc522_driver_private.h"

RC522_LOG_DEFINE_BASE();

esp_err_t rc522_driver_init_rst_pin(gpio_num_t rst_io_num)
{
    RC522_CHECK(rst_io_num < 0);

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << rst_io_num),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };

    RC522_RETURN_ON_ERROR(gpio_config(&io_conf));
    RC522_RETURN_ON_ERROR(gpio_set_level(rst_io_num, 0));

    return ESP_OK;
}

inline esp_err_t rc522_driver_install(rc522_driver_handle_t driver)
{
    RC522_CHECK(driver == NULL);

    return driver->install(driver);
}

inline esp_err_t rc522_driver_send(rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(buffer == NULL);
    RC522_CHECK(length < 1);

    return driver->send(driver, address, buffer, length);
}

inline esp_err_t rc522_driver_receive(rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(buffer == NULL);
    RC522_CHECK(length < 1);

    return driver->receive(driver, address, buffer, length);
}

inline esp_err_t rc522_driver_reset(rc522_driver_handle_t driver)
{
    RC522_CHECK(driver == NULL);

    return driver->reset(driver);
}

inline esp_err_t rc522_driver_uninstall(rc522_driver_handle_t driver)
{
    RC522_CHECK(driver == NULL);

    return driver->uninstall(driver);
}

esp_err_t rc522_driver_create(void *config, size_t config_size, rc522_driver_handle_t *driver)
{
    RC522_CHECK(config == NULL);
    RC522_CHECK(config_size == 0);
    RC522_CHECK(driver == NULL);

    esp_err_t ret = ESP_OK;

    rc522_driver_handle_t _driver = calloc(1, sizeof(struct rc522_driver_handle));
    ESP_RETURN_ON_FALSE(_driver != NULL, ESP_ERR_NO_MEM, TAG, "nomem");

    _driver->config = calloc(1, config_size);
    ESP_GOTO_ON_FALSE(_driver->config != NULL, ESP_ERR_NO_MEM, error, TAG, "nomem");

    memcpy(_driver->config, config, config_size);

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
    RC522_CHECK(driver == NULL);

    if (driver->config) {
        free(driver->config);
        driver->config = NULL;
    }

    driver->install = NULL;
    driver->send = NULL;
    driver->receive = NULL;
    driver->uninstall = NULL;

    driver->device = NULL;

    free(driver);

    return ESP_OK;
}
