#include <esp_log.h>
#include <driver/spi_master.h>
#include "rc522.h"
#include "rc522_picc.h"

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
    uint8_t *address = (buffer + 0);
    uint8_t address_origin = *address;

    *address <<= 1;
    *address &= (~0x80);

    esp_err_t ret = spi_device_polling_transmit(rc522_spi_handle,
        &(spi_transaction_t) {
            .length = 8 * length,
            .tx_buffer = buffer,
        });

    *address = address_origin;

    return ret;
}

static esp_err_t rc522_receive(uint8_t address, uint8_t *buffer, uint8_t length)
{
    address <<= 1;
    address |= 0x80;

    spi_device_acquire_bus(rc522_spi_handle, portMAX_DELAY);

    for (uint8_t i = 0; i < length; i++) {
        spi_device_polling_transmit(rc522_spi_handle,
            &(spi_transaction_t) {
                .length = 8,
                .tx_buffer = &address,
                .rxlength = 8,
                .rx_buffer = (buffer + i),
            });
    }

    spi_device_release_bus(rc522_spi_handle);

    return ESP_OK;
}

static void rc522_event_handler(void *arg, esp_event_base_t base, int32_t event_id, void *event_data)
{
    rc522_event_data_t *data = (rc522_event_data_t *)event_data;

    switch (event_id) {
        case RC522_EVENT_PICC_SELECTED: {
            rc522_picc_t *picc = (rc522_picc_t *)data->ptr;

            ESP_LOGI(TAG, "PICC (sak=%02x, type=%s) scanned!", picc->sak, rc522_picc_type_name(picc->type));
            ESP_LOGI(TAG, "UID:");
            ESP_LOG_BUFFER_HEX(TAG, picc->uid.bytes, picc->uid.bytes_length);
        } break;
    }
}

void app_main()
{
    esp_log_level_set("*", ESP_LOG_INFO);
    // esp_log_level_set("rc522", ESP_LOG_DEBUG);

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
