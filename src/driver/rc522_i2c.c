#include <string.h>
#include "rc522_helpers_internal.h"
#include "rc522_types_internal.h"
#include "rc522_driver_internal.h"
#include "driver/rc522_i2c.h"

RC522_LOG_DEFINE_BASE();

static esp_err_t rc522_i2c_install(const rc522_driver_handle_t driver)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(driver->config == NULL);

    rc522_i2c_config_t *conf = (rc522_i2c_config_t *)(driver->config);

    // TODO: Skip bus initialization if it's already configured by the user
    //       Do this once when we migrate to new i2c API
    RC522_RETURN_ON_ERROR(i2c_param_config(conf->port, &conf->config));

    RC522_RETURN_ON_ERROR(i2c_driver_install(conf->port, conf->config.mode, 0, 0, 0x00));

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

    rc522_i2c_config_t *conf = (rc522_i2c_config_t *)(driver->config);

    RC522_RETURN_ON_ERROR(i2c_master_write_to_device(conf->port,
        conf->device_address,
        buffer2,
        (bytes->length + 1),
        pdMS_TO_TICKS(conf->rw_timeout_ms)));

    return ESP_OK;
}

static esp_err_t rc522_i2c_receive(const rc522_driver_handle_t driver, uint8_t address, rc522_bytes_t *bytes)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(driver->config == NULL);
    RC522_CHECK_BYTES(bytes);

    rc522_i2c_config_t *conf = (rc522_i2c_config_t *)(driver->config);

    RC522_RETURN_ON_ERROR(i2c_master_write_read_device(conf->port,
        conf->device_address,
        &address,
        1,
        bytes->ptr,
        bytes->length,
        pdMS_TO_TICKS(conf->rw_timeout_ms)));

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

    rc522_i2c_config_t *conf = (rc522_i2c_config_t *)(driver->config);

    RC522_RETURN_ON_ERROR(i2c_driver_delete(conf->port));

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
