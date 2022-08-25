#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "rc522.h"

#define RC522_I2C_ADDRESS (0x28)

#ifndef RC522_I2C_PORT
    #define RC522_I2C_PORT I2C_NUM_0
#endif

#ifndef RC522_I2C_SDA_GPIO
    #define RC522_I2C_SDA_GPIO (18)
#endif

#ifndef RC522_I2C_SCL_GPIO
    #define RC522_I2C_SCL_GPIO (19)
#endif

#ifndef RC522_I2C_CLK_SPEED
    #define RC522_I2C_CLK_SPEED (100000)
#endif

#ifndef RC522_I2C_RW_TIMEOUT_MS
    #define RC522_I2C_RW_TIMEOUT_MS (1000)
#endif

rc522_transport_t* rc522_i2c();

#ifdef __cplusplus
}
#endif
