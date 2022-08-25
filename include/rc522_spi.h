#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <driver/spi_master.h>
#include "rc522.h"

#ifndef RC522_SPI_HOST
    #define RC522_SPI_HOST VSPI_HOST
#endif

#ifndef RC522_SPI_MISO_GPIO
    #define RC522_SPI_MISO_GPIO (25)
#endif

#ifndef RC522_SPI_MOSI_GPIO
    #define RC522_SPI_MOSI_GPIO (23)
#endif

#ifndef RC522_SPI_SCK_GPIO
    #define RC522_SPI_SCK_GPIO (19)
#endif

#ifndef RC522_SPI_SDA_GPIO
    #define RC522_SPI_SDA_GPIO (22)
#endif

#ifndef RC522_SPI_CLOCK_SPEED_HZ
    #define RC522_SPI_CLOCK_SPEED_HZ (5000000)
#endif

rc522_transport_t* rc522_spi();

#ifdef __cplusplus
}
#endif
