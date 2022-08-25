#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <driver/i2c.h>
#include "rc522.h"

#define RC522_I2C_ADDRESS (0x28)

#ifndef RC522_I2C_PORT
    #define RC522_I2C_PORT I2C_NUM_0
#endif

#ifndef RC522_I2C_CLK_SPEED
    #define RC522_I2C_CLK_SPEED (100000)
#endif

#ifndef RC522_I2C_RW_TIMEOUT_MS
    #define RC522_I2C_RW_TIMEOUT_MS (1000)
#endif

rc522_transport_t* rc522_i2c(int sda_gpio, int scl_gpio);

#ifdef __cplusplus
}
#endif
