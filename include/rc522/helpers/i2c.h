#pragma once

#include <driver/i2c.h>

#ifndef RC522_I2C_ADDRESS
#define RC522_I2C_ADDRESS (0x28)
#endif

#ifndef RC522_I2C_RW_TIMEOUT_MS
#define RC522_I2C_RW_TIMEOUT_MS (1000)
#endif

#ifndef RC522_I2C_CLOCK_SPEED_HZ
#define RC522_I2C_CLOCK_SPEED_HZ (100000)
#endif

#ifndef RC522_I2C_PORT
#define RC522_I2C_PORT (I2C_NUM_0)
#endif

#ifndef RC522_I2C_GPIO_SDA
#define RC522_I2C_GPIO_SDA (18)
#endif

#ifndef RC522_I2C_GPIO_SCL
#define RC522_I2C_GPIO_SCL (19)
#endif

/**
 * Initialize I2C
 */
esp_err_t rc522_i2c_init();

/**
 * Send data to RC522 via I2C
 */
esp_err_t rc522_i2c_send(uint8_t *buffer, uint8_t length);

/**
 * Receive data from RC522 via I2C
 */
esp_err_t rc522_i2c_receive(uint8_t *buffer, uint8_t length, uint8_t address);
