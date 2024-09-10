#include <esp_log.h>
#include <inttypes.h>
#include <driver/spi_master.h>
#include "rc522.h"

static const char *TAG = "rc522-demo";

static rc522_handle_t rc522_handle;
static spi_device_handle_t rc522_spi_handle;

#define RC522_SPI_DEVICE_SPEED_HZ   (5000000)
#define RC522_SPI_DEVICE_MODE       (0)
#define RC522_SPI_DEVICE_GPIO_SDA   (22)
#define RC522_SPI_DEVICE_QUEUE_SIZE (7)
#define RC522_SPI_DEVICE_FLAGS      (0x00)
#define RC522_SPI_HOST              (VSPI_HOST)
#define RC522_SPI_BUS_GPIO_MISO     (25)
#define RC522_SPI_BUS_GPIO_MOSI     (23)
#define RC522_SPI_BUS_GPIO_SCLK     (19)

/*
 * Initialize SPI bus
 */
static esp_err_t spi_init()
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

/*
 * Attach RC522 to the SPI bus
 */
static esp_err_t spi_attach()
{
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = RC522_SPI_DEVICE_SPEED_HZ,
        .mode = RC522_SPI_DEVICE_MODE,
        .spics_io_num = RC522_SPI_DEVICE_GPIO_SDA,
        .queue_size = RC522_SPI_DEVICE_QUEUE_SIZE,
        .flags = RC522_SPI_DEVICE_FLAGS,
    };

    return spi_bus_add_device(RC522_SPI_HOST, &devcfg, &rc522_spi_handle);
}

/*
 * Send data to RC522
 */
static esp_err_t send(uint8_t *buffer, uint8_t length)
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

/*
 * Receive data from RC522
 */
static esp_err_t receive(uint8_t *buffer, uint8_t length, uint8_t address)
{
    address = ((address << 1) & 0x7E) | 0x80;

    // Halfduplex

    if (SPI_DEVICE_HALFDUPLEX & RC522_SPI_DEVICE_FLAGS) {
        return spi_device_transmit(rc522_spi_handle,
            &(spi_transaction_t) {
                .flags = SPI_TRANS_USE_TXDATA,
                .length = 8,
                .tx_data[0] = address,
                .rxlength = 8 * length,
                .rx_buffer = buffer,
            });
    }

    // Fullduplex

    esp_err_t ret = spi_device_transmit(rc522_spi_handle,
        &(spi_transaction_t) {
            .flags = SPI_TRANS_USE_TXDATA,
            .length = 8,
            .tx_data[0] = address,
        });

    if (ret != ESP_OK) {
        return ret;
    }

    return spi_device_transmit(rc522_spi_handle,
        &(spi_transaction_t) {
            .flags = 0x00,
            .length = 8,
            .rxlength = 8 * length,
            .rx_buffer = buffer,
        });
}

/*
 * Handle RC522 events, e.g. when tag is scanned, etc...
 */
static void rc522_event_handler(void *arg, esp_event_base_t base, int32_t event_id, void *event_data)
{
    rc522_event_data_t *data = (rc522_event_data_t *)event_data;

    switch (event_id) {
        case RC522_EVENT_TAG_SCANNED: {
            rc522_tag_t *tag = (rc522_tag_t *)data->ptr;
            ESP_LOGI(TAG, "Tag scanned (sn: %" PRIu64 ")", tag->serial_number);
        } break;
    }
}

void app_main()
{
    spi_init();
    spi_attach();

    rc522_config_t config = {
        .send_handler = &send,
        .receive_handler = &receive,
    };

    rc522_create(&config, &rc522_handle);
    rc522_register_events(rc522_handle, RC522_EVENT_ANY, rc522_event_handler, NULL);
    rc522_start(rc522_handle);
}
