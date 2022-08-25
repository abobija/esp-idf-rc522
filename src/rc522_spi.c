#include "rc522_spi.h"

static spi_device_handle_t spi;

static esp_err_t rc522_spi_init()
{
    spi_bus_config_t buscfg = {
        .miso_io_num = RC522_SPI_MISO_GPIO,
        .mosi_io_num = RC522_SPI_MOSI_GPIO,
        .sclk_io_num = RC522_SPI_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = RC522_SPI_CLOCK_SPEED_HZ,
        .mode = 0,
        .spics_io_num = RC522_SPI_SDA_GPIO,
        .queue_size = 7,
        .flags = SPI_DEVICE_HALFDUPLEX,
    };

    esp_err_t ret;

    if(ESP_OK != (ret = spi_bus_initialize(RC522_SPI_HOST, &buscfg, 0))) {
        return ret;
    }

    return spi_bus_add_device(RC522_SPI_HOST, &devcfg, &spi);
}

static esp_err_t rc522_spi_send(uint8_t* buffer, uint8_t length)
{
    buffer[0] = (buffer[0] << 1) & 0x7E;

    return spi_device_transmit(spi, &(spi_transaction_t){
        .length = 8 * length,
        .tx_buffer = buffer,
    });
}

static esp_err_t rc522_spi_receive(uint8_t* buffer, uint8_t length, uint8_t addr)
{
    addr = ((addr << 1) & 0x7E) | 0x80;

    return spi_device_transmit(spi, &(spi_transaction_t){
        .flags = SPI_TRANS_USE_TXDATA,
        .length = 8,
        .tx_data[0] = addr,
        .rxlength = 8 * length,
        .rx_buffer = buffer,
    });
}

static void rc522_spi_remove()
{
    spi_bus_remove_device(spi);
    spi_bus_free(RC522_SPI_HOST);
}

rc522_transport_t* rc522_spi()
{
    rc522_transport_t* trans = calloc(1, sizeof(rc522_transport_t)); // FIXME: memcheck

    trans->init = rc522_spi_init;
    trans->send = rc522_spi_send;
    trans->receive = rc522_spi_receive;
    trans->remove = rc522_spi_remove;

    return trans;
}
