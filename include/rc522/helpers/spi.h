#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <driver/spi_master.h>
#include "rc522.h"

#ifndef RC522_SPI_CLOCK_SPEED_HZ
    #define RC522_SPI_CLOCK_SPEED_HZ (5000000)
#endif

rc522_transport_t* rc522_spi(spi_host_device_t host, int miso_gpio, int mosi_gpio, int sck_gpio, int sda_gpio);

#ifdef __cplusplus
}
#endif
