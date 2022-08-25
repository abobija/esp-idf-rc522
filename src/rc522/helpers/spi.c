#include "rc522/helpers/spi.h"

static spi_host_device_t _host;
static int _miso_gpio;
static int _mosi_gpio;
static int _sck_gpio;
static int _sda_gpio;

static spi_device_handle_t _spi;

static esp_err_t rc522_spi_init()
{
    spi_bus_config_t buscfg = {
        .miso_io_num = _miso_gpio,
        .mosi_io_num = _mosi_gpio,
        .sclk_io_num = _sck_gpio,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = RC522_SPI_CLOCK_SPEED_HZ,
        .mode = 0,
        .spics_io_num = _sda_gpio,
        .queue_size = 7,
        .flags = SPI_DEVICE_HALFDUPLEX,
    };

    esp_err_t ret;

    if(ESP_OK != (ret = spi_bus_initialize(_host, &buscfg, 0))) {
        return ret;
    }

    return spi_bus_add_device(_host, &devcfg, &_spi);
}

static esp_err_t rc522_spi_send(uint8_t* buffer, uint8_t length)
{
    buffer[0] = (buffer[0] << 1) & 0x7E;

    return spi_device_transmit(_spi, &(spi_transaction_t){
        .length = 8 * length,
        .tx_buffer = buffer,
    });
}

static esp_err_t rc522_spi_receive(uint8_t* buffer, uint8_t length, uint8_t addr)
{
    addr = ((addr << 1) & 0x7E) | 0x80;

    return spi_device_transmit(_spi, &(spi_transaction_t){
        .flags = SPI_TRANS_USE_TXDATA,
        .length = 8,
        .tx_data[0] = addr,
        .rxlength = 8 * length,
        .rx_buffer = buffer,
    });
}

static void rc522_spi_remove()
{
    spi_bus_remove_device(_spi);
    spi_bus_free(_host);
}

rc522_transport_t* rc522_spi(spi_host_device_t host, int miso_gpio, int mosi_gpio, int sck_gpio, int sda_gpio)
{
    _host = host;
    _miso_gpio = miso_gpio;
    _mosi_gpio = mosi_gpio;
    _sck_gpio = sck_gpio;
    _sda_gpio = sda_gpio;

    rc522_transport_t* trans = calloc(1, sizeof(rc522_transport_t)); // FIXME: memcheck

    trans->init = rc522_spi_init;
    trans->send = rc522_spi_send;
    trans->receive = rc522_spi_receive;
    trans->remove = rc522_spi_remove;

    return trans;
}
