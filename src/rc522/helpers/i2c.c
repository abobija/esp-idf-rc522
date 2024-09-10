#include "rc522/helpers/i2c.h"

esp_err_t rc522_i2c_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = RC522_I2C_GPIO_SDA,
        .scl_io_num = RC522_I2C_GPIO_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = RC522_I2C_CLOCK_SPEED_HZ,
    };

    esp_err_t err = i2c_param_config(RC522_I2C_PORT, &conf);

    if (err != ESP_OK) {
        return err;
    }

    return i2c_driver_install(RC522_I2C_PORT, conf.mode, false, false, 0x00);
}

esp_err_t rc522_i2c_send(uint8_t *buffer, uint8_t length)
{
    return i2c_master_write_to_device(RC522_I2C_PORT,
        RC522_I2C_ADDRESS,
        buffer,
        length,
        RC522_I2C_RW_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t rc522_i2c_receive(uint8_t *buffer, uint8_t length, uint8_t address)
{
    return i2c_master_write_read_device(RC522_I2C_PORT,
        RC522_I2C_ADDRESS,
        &address,
        1,
        buffer,
        length,
        RC522_I2C_RW_TIMEOUT_MS / portTICK_PERIOD_MS);
}
