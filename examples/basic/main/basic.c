#include <esp_log.h>
#include <driver/spi_master.h>
#include "rc522.h"

static const char *TAG = "rc522-basic-example";

#define RC522_SPI_HOST            (VSPI_HOST)
#define RC522_SPI_BUS_GPIO_MISO   (25)
#define RC522_SPI_BUS_GPIO_MOSI   (23)
#define RC522_SPI_BUS_GPIO_SCLK   (19)
#define RC522_SPI_DEVICE_GPIO_SDA (22)

static spi_bus_config_t spi_bus_config = {
    .miso_io_num = RC522_SPI_BUS_GPIO_MISO,
    .mosi_io_num = RC522_SPI_BUS_GPIO_MOSI,
    .sclk_io_num = RC522_SPI_BUS_GPIO_SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
};

static spi_device_interface_config_t spi_rc522_config = {
    .clock_speed_hz = 5000000,
    .mode = 0,
    .spics_io_num = RC522_SPI_DEVICE_GPIO_SDA,
    .queue_size = 7,
    .flags = SPI_DEVICE_HALFDUPLEX,
};

static rc522_handle_t rc522_handle;
static spi_device_handle_t rc522_spi_handle;

static esp_err_t rc522_send(uint8_t *buffer, uint8_t length)
{
    uint8_t first_byte_origin = buffer[0];

    buffer[0] = (buffer[0] << 1) & 0x7E;

    esp_err_t ret = spi_device_transmit(rc522_spi_handle,
        &(spi_transaction_t) {
            .length = 8 * length,
            .tx_buffer = buffer,
        });

    buffer[0] = first_byte_origin;

    return ret;
}

static esp_err_t rc522_receive(uint8_t *buffer, uint8_t length, uint8_t address)
{
    address = ((address << 1) & 0x7E) | 0x80;

    return spi_device_transmit(rc522_spi_handle,
        &(spi_transaction_t) {
            .flags = SPI_TRANS_USE_TXDATA,
            .length = 8,
            .tx_data[0] = address,
            .rxlength = 8 * length,
            .rx_buffer = buffer,
        });
}

static void rc522_event_handler(void *arg, esp_event_base_t base, int32_t event_id, void *event_data)
{
    rc522_event_data_t *data = (rc522_event_data_t *)event_data;

    switch (event_id) {
        case RC522_EVENT_TAG_SCANNED: {
            ESP_LOGI(TAG, "Tag scanned!");

            rc522_tag_t *tag = (rc522_tag_t *)data->ptr;
            ESP_LOG_BUFFER_HEX(TAG, tag->uid.bytes, tag->uid.length);
        } break;
    }
}

void app_main()
{
    spi_bus_initialize(RC522_SPI_HOST, &spi_bus_config, 0);
    spi_bus_add_device(RC522_SPI_HOST, &spi_rc522_config, &rc522_spi_handle);

    rc522_config_t config = {
        .send_handler = &rc522_send,
        .receive_handler = &rc522_receive,
    };

    rc522_create(&config, &rc522_handle);
    rc522_register_events(rc522_handle, RC522_EVENT_ANY, rc522_event_handler, NULL);
    rc522_start(rc522_handle);
}
