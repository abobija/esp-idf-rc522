#pragma once

#include <driver/spi_master.h>

#ifndef RC522_SPI_DEVICE_SPEED_HZ
#define RC522_SPI_DEVICE_SPEED_HZ (5000000)
#endif

#ifndef RC522_SPI_DEVICE_MODE
#define RC522_SPI_DEVICE_MODE (0)
#endif

#ifndef RC522_SPI_DEVICE_GPIO_SDA
#define RC522_SPI_DEVICE_GPIO_SDA (22)
#endif

#ifndef RC522_SPI_DEVICE_QUEUE_SIZE
#define RC522_SPI_DEVICE_QUEUE_SIZE (7)
#endif

#ifndef RC522_SPI_DEVICE_FLAGS
#define RC522_SPI_DEVICE_FLAGS (0x00)
#endif

#ifndef RC522_SPI_HOST
#define RC522_SPI_HOST (VSPI_HOST)
#endif

#ifndef RC522_SPI_BUS_GPIO_MISO
#define RC522_SPI_BUS_GPIO_MISO (25)
#endif

#ifndef RC522_SPI_BUS_GPIO_MOSI
#define RC522_SPI_BUS_GPIO_MOSI (23)
#endif

#ifndef RC522_SPI_BUS_GPIO_SCLK
#define RC522_SPI_BUS_GPIO_SCLK (19)
#endif

/**
 * Initialize SPI bus
 */
esp_err_t rc522_spi_init();

/**
 * Attach RC522 to the SPI bus
 */
esp_err_t rc522_spi_attach(spi_device_handle_t *spi_device_handle);

/**
 * Send data to RC522 via SPI
 */
esp_err_t rc522_spi_send(spi_device_handle_t spi_device_handle, uint8_t *buffer, uint8_t length);

/**
 * Receive data from RC522 via SPI
 */
esp_err_t rc522_spi_receive(spi_device_handle_t spi_device_handle, uint8_t *buffer, uint8_t length, uint8_t address);
