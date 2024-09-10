#include <esp_log.h>
#include <inttypes.h>

#include "rc522.h"
#include "rc522/helpers/spi.h"

static const char *TAG = "rc522-demo";

static rc522_handle_t rc522_handle;
static spi_device_handle_t rc522_spi_handle;

/**
 * Send data to RC522
 */
static esp_err_t send(uint8_t *buffer, uint8_t length)
{
    return rc522_spi_send(rc522_spi_handle, buffer, length);
}

/**
 * Receive data from RC522
 */
static esp_err_t receive(uint8_t *buffer, uint8_t length, uint8_t address)
{
    return rc522_spi_receive(rc522_spi_handle, buffer, length, address);
}

/**
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
    rc522_spi_init();
    rc522_spi_attach(&rc522_spi_handle);

    rc522_config_t config = {
        .send_handler = &send,
        .receive_handler = &receive,
    };

    rc522_create(&config, &rc522_handle);
    rc522_register_events(rc522_handle, RC522_EVENT_ANY, rc522_event_handler, NULL);
    rc522_start(rc522_handle);
}
