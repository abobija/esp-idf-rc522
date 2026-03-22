#include "driver/rc522_i2c_new.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_err.h"
#include "rc522_driver.h"
#include "rc522_helpers_internal.h"
#include "rc522_types_internal.h"
#include "rc522_driver_internal.h"
#include "soc/gpio_num.h"

#ifdef __cplusplus
extern "C" {
#endif

RC522_LOG_DEFINE_BASE();

static esp_err_t rc522_new_i2c_install(const rc522_driver_handle_t driver)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(driver->config == NULL);

    rc522_new_i2c_config_t *conf = (rc522_new_i2c_config_t *)(driver->config);

    if (conf->i2c_bus_handle == NULL) {
        i2c_new_master_bus(&conf->i2c_bus_config, &conf->i2c_bus_handle);
    }

    RC522_RETURN_ON_ERROR(i2c_master_bus_add_device(conf->i2c_bus_handle,
        &conf->i2c_device_config,
        (i2c_master_dev_handle_t *)(&driver->device)));

    if (conf->rst_io_num < GPIO_NUM_NC) {
        RC522_RETURN_ON_ERROR(rc522_driver_init_rst_pin(conf->rst_io_num));
    }

    return ESP_OK;
}

static esp_err_t rc522_new_i2c_send(const rc522_driver_handle_t driver, uint8_t address, const rc522_bytes_t *bytes)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(driver->config == NULL);
    RC522_CHECK_BYTES(bytes);

    uint8_t buffer[64] = { address };
    memcpy(&buffer[1], bytes->ptr, bytes->length);

    RC522_RETURN_ON_ERROR(
        i2c_master_transmit((i2c_master_dev_handle_t)driver->device, buffer, bytes->length + 0x01, 1000));

    return ESP_OK;
}

static esp_err_t rc522_new_i2c_receive(const rc522_driver_handle_t driver, uint8_t address, rc522_bytes_t *bytes)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(driver->config == NULL);
    RC522_CHECK_BYTES(bytes);

    ESP_ERROR_CHECK(i2c_master_transmit_receive((i2c_master_dev_handle_t)driver->device,
        &address,
        1,
        bytes->ptr,
        bytes->length,
        1000));

    return ESP_OK;
}

static esp_err_t rc522_new_i2c_reset(const rc522_driver_handle_t driver)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(driver->config == NULL);

    rc522_new_i2c_config_t *conf = (rc522_new_i2c_config_t *)(driver->config);

    if (conf->rst_io_num < 0) {
        return RC522_ERR_RST_PIN_UNUSED;
    }

    RC522_RETURN_ON_ERROR(gpio_set_level(conf->rst_io_num, RC522_DRIVER_HARD_RST_PIN_PWR_DOWN_LEVEL));
    rc522_delay_ms(RC522_DRIVER_HARD_RST_PULSE_DURATION_MS);
    RC522_RETURN_ON_ERROR(gpio_set_level(conf->rst_io_num, !RC522_DRIVER_HARD_RST_PIN_PWR_DOWN_LEVEL));
    rc522_delay_ms(RC522_DRIVER_HARD_RST_PULSE_DURATION_MS);

    return ESP_OK;
}

static esp_err_t rc522_new_i2c_uninstall(const rc522_driver_handle_t driver)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(driver->config == NULL);

    RC522_RETURN_ON_ERROR(i2c_master_bus_rm_device((i2c_master_dev_handle_t)driver->device));

    return ESP_OK;
}

esp_err_t rc522_new_i2c_create(const rc522_new_i2c_config_t *config, rc522_driver_handle_t *driver)
{
    RC522_CHECK(config == NULL);
    RC522_CHECK(driver == NULL);

    RC522_RETURN_ON_ERROR(rc522_driver_create(config, sizeof(rc522_new_i2c_config_t), driver));

    (*driver)->install = rc522_new_i2c_install;
    (*driver)->send = rc522_new_i2c_send;
    (*driver)->receive = rc522_new_i2c_receive;
    (*driver)->reset = rc522_new_i2c_reset;
    (*driver)->uninstall = rc522_new_i2c_uninstall;

    return ESP_OK;
}

#ifdef __cplusplus
}
#endif
