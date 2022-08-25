#include "rc522/helpers/i2c.h"

static int _sda_gpio;
static int _scl_gpio;

static esp_err_t rc522_i2c_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = _sda_gpio,
        .scl_io_num = _scl_gpio,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = RC522_I2C_CLK_SPEED,
    };

    esp_err_t ret;

    if(ESP_OK != (ret = i2c_param_config(RC522_I2C_PORT, &conf))) {
        return ret;
    }

    return i2c_driver_install(RC522_I2C_PORT, conf.mode, false, false, 0x00);
}

static esp_err_t rc522_i2c_send(uint8_t* buffer, uint8_t length)
{
    return i2c_master_write_to_device(RC522_I2C_PORT, RC522_I2C_ADDRESS, buffer, length, RC522_I2C_RW_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t rc522_i2c_receive(uint8_t* buffer, uint8_t length, uint8_t addr)
{
    return i2c_master_write_read_device(RC522_I2C_PORT, RC522_I2C_ADDRESS, &addr, 1, buffer, length, RC522_I2C_RW_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static void rc522_i2c_remove()
{
    i2c_driver_delete(RC522_I2C_PORT);
}

rc522_transport_t* rc522_i2c(int sda_gpio, int scl_gpio)
{
    _sda_gpio = sda_gpio;
    _scl_gpio = scl_gpio;

    rc522_transport_t* trans = calloc(1, sizeof(rc522_transport_t)); // FIXME: memcheck

    trans->init = rc522_i2c_init;
    trans->send = rc522_i2c_send;
    trans->receive = rc522_i2c_receive;
    trans->remove = rc522_i2c_remove;

    return trans;
}
