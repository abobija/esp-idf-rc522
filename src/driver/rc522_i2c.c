#include <string.h>
#include "rc522_helpers_internal.h"
#include "rc522_types_internal.h"
#include "rc522_driver_internal.h"
#include "driver/rc522_i2c.h"

#define RC522_I2C_RW_TIMEOUT_MS 1000

RC522_LOG_DEFINE_BASE();

static esp_err_t rc522_i2c_install(const rc522_driver_handle_t driver)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(driver->config == NULL);

    rc522_i2c_config_t *conf = (rc522_i2c_config_t *)(driver->config);

    if (!conf->bus) {
        RC522_CHECK(conf->bus_config == NULL);
        RC522_RETURN_ON_ERROR(i2c_new_master_bus(conf->bus_config, &conf->bus));
    }

    RC522_RETURN_ON_ERROR(
        i2c_master_bus_add_device(conf->bus, &conf->dev_config, (i2c_master_dev_handle_t *)(&driver->device)));

    if (conf->rst_io_num > GPIO_NUM_NC) {
        RC522_RETURN_ON_ERROR(rc522_driver_init_rst_pin(conf->rst_io_num));
    }

    return ESP_OK;
}

static esp_err_t rc522_i2c_send(const rc522_driver_handle_t driver, uint8_t address, const rc522_bytes_t *bytes)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(driver->config == NULL);
    RC522_CHECK_BYTES(bytes);

    // FIXME: Find a way to send [address + buffer]
    //        without need for second buffer
    uint8_t buffer2[64];

    buffer2[0] = address;
    memcpy(buffer2 + 1, bytes->ptr, bytes->length);

    RC522_RETURN_ON_ERROR(i2c_master_transmit((i2c_master_dev_handle_t)driver->device,
        buffer2,
        bytes->length + 1,
        RC522_I2C_RW_TIMEOUT_MS));

    return ESP_OK;
}

static esp_err_t rc522_i2c_receive(const rc522_driver_handle_t driver, uint8_t address, rc522_bytes_t *bytes)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(driver->config == NULL);
    RC522_CHECK_BYTES(bytes);

    ESP_ERROR_CHECK(i2c_master_transmit_receive((i2c_master_dev_handle_t)driver->device,
        &address,
        1,
        bytes->ptr,
        bytes->length,
        RC522_I2C_RW_TIMEOUT_MS));

    return ESP_OK;
}

static esp_err_t rc522_i2c_reset(const rc522_driver_handle_t driver)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(driver->config == NULL);

    rc522_i2c_config_t *conf = (rc522_i2c_config_t *)(driver->config);

    if (conf->rst_io_num < 0) {
        return RC522_ERR_RST_PIN_UNUSED;
    }

    RC522_RETURN_ON_ERROR(gpio_set_level(conf->rst_io_num, RC522_DRIVER_HARD_RST_PIN_PWR_DOWN_LEVEL));
    rc522_delay_ms(RC522_DRIVER_HARD_RST_PULSE_DURATION_MS);
    RC522_RETURN_ON_ERROR(gpio_set_level(conf->rst_io_num, !RC522_DRIVER_HARD_RST_PIN_PWR_DOWN_LEVEL));
    rc522_delay_ms(RC522_DRIVER_HARD_RST_PULSE_DURATION_MS);

    return ESP_OK;
}

static esp_err_t rc522_i2c_uninstall(const rc522_driver_handle_t driver)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(driver->config == NULL);

    RC522_RETURN_ON_ERROR(i2c_master_bus_rm_device((i2c_master_dev_handle_t)driver->device));
    // TODO: remove bus if handle was NULL?

    return ESP_OK;
}

esp_err_t rc522_i2c_create(const rc522_i2c_config_t *config, rc522_driver_handle_t *driver)
{
    RC522_CHECK(config == NULL);
    RC522_CHECK(driver == NULL);

    RC522_RETURN_ON_ERROR(rc522_driver_create(config, sizeof(rc522_i2c_config_t), driver));

    (*driver)->install = rc522_i2c_install;
    (*driver)->send = rc522_i2c_send;
    (*driver)->receive = rc522_i2c_receive;
    (*driver)->reset = rc522_i2c_reset;
    (*driver)->uninstall = rc522_i2c_uninstall;

    return ESP_OK;
}
