#include <string.h>
#include <driver/gpio.h>
#include "rc522_types_internal.h"
#include "rc522_driver_internal.h"

RC522_LOG_DEFINE_BASE();

esp_err_t rc522_driver_init_rst_pin(gpio_num_t rst_io_num)
{
    RC522_CHECK(rst_io_num < 0);

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << rst_io_num),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    RC522_RETURN_ON_ERROR(gpio_config(&io_conf));
    RC522_RETURN_ON_ERROR(gpio_set_level(rst_io_num, !RC522_DRIVER_HARD_RST_PIN_PWR_DOWN_LEVEL));

    return ESP_OK;
}

inline esp_err_t rc522_driver_install(const rc522_driver_handle_t driver)
{
    RC522_CHECK(driver == NULL);

    return driver->install(driver);
}

inline esp_err_t rc522_driver_send(const rc522_driver_handle_t driver, uint8_t address, const rc522_bytes_t *bytes)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK_BYTES(bytes);

    return driver->send(driver, address, bytes);
}

inline esp_err_t rc522_driver_receive(const rc522_driver_handle_t driver, uint8_t address, rc522_bytes_t *bytes)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK_BYTES(bytes);

    return driver->receive(driver, address, bytes);
}

inline esp_err_t rc522_driver_reset(const rc522_driver_handle_t driver)
{
    RC522_CHECK(driver == NULL);

    return driver->reset(driver);
}

inline esp_err_t rc522_driver_uninstall(const rc522_driver_handle_t driver)
{
    RC522_CHECK(driver == NULL);

    return driver->uninstall(driver);
}

esp_err_t rc522_driver_create(const void *config, size_t config_size, rc522_driver_handle_t *driver)
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
