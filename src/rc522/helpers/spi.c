#include "rc522/helpers/spi.h"

esp_err_t rc522_spi_init()
{
    spi_bus_config_t buscfg = {
        .miso_io_num = RC522_SPI_BUS_GPIO_MISO,
        .mosi_io_num = RC522_SPI_BUS_GPIO_MOSI,
        .sclk_io_num = RC522_SPI_BUS_GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    return spi_bus_initialize(RC522_SPI_HOST, &buscfg, 0);
}

esp_err_t rc522_spi_attach(spi_device_handle_t *spi_device_handle)
{
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = RC522_SPI_DEVICE_SPEED_HZ,
        .mode = RC522_SPI_DEVICE_MODE,
        .spics_io_num = RC522_SPI_DEVICE_GPIO_SDA,
        .queue_size = RC522_SPI_DEVICE_QUEUE_SIZE,
        .flags = RC522_SPI_DEVICE_FLAGS,
    };

    return spi_bus_add_device(RC522_SPI_HOST, &devcfg, spi_device_handle);
}

esp_err_t rc522_spi_send(spi_device_handle_t spi_device_handle, uint8_t *buffer, uint8_t length)
{
    uint8_t first_byte_origin = buffer[0];

    buffer[0] = (buffer[0] << 1) & 0x7E;

    esp_err_t ret = spi_device_transmit(spi_device_handle,
        &(spi_transaction_t) {
            .length = 8 * length,
            .tx_buffer = buffer,
        });

    buffer[0] = first_byte_origin;

    return ret;
}

esp_err_t rc522_spi_receive(spi_device_handle_t spi_device_handle, uint8_t *buffer, uint8_t length, uint8_t address)
{
    address = ((address << 1) & 0x7E) | 0x80;

    // Halfduplex

    if (SPI_DEVICE_HALFDUPLEX & RC522_SPI_DEVICE_FLAGS) {
        return spi_device_transmit(spi_device_handle,
            &(spi_transaction_t) {
                .flags = SPI_TRANS_USE_TXDATA,
                .length = 8,
                .tx_data[0] = address,
                .rxlength = 8 * length,
                .rx_buffer = buffer,
            });
    }

    // Fullduplex

    esp_err_t ret = spi_device_transmit(spi_device_handle,
        &(spi_transaction_t) {
            .flags = SPI_TRANS_USE_TXDATA,
            .length = 8,
            .tx_data[0] = address,
        });

    if (ret != ESP_OK) {
        return ret;
    }

    return spi_device_transmit(spi_device_handle,
        &(spi_transaction_t) {
            .flags = 0x00,
            .length = 8,
            .rxlength = 8 * length,
            .rx_buffer = buffer,
        });
}
