#include <esp_log.h>
#include <inttypes.h>

#include "rc522.h"
#include "rc522/helpers/i2c.h"

static const char *TAG = "rc522-i2c-example";

static rc522_handle_t rc522_handle;

static esp_err_t send(uint8_t *buffer, uint8_t length)
{
    return rc522_i2c_send(buffer, length);
}

static esp_err_t receive(uint8_t *buffer, uint8_t length, uint8_t address)
{
    return rc522_i2c_receive(buffer, length, address);
}

static void rc522_event_handler(void *arg, esp_event_base_t base, int32_t event_id, void *event_data)
{
    rc522_event_data_t *data = (rc522_event_data_t *)event_data;

    switch (event_id) {
        case RC522_EVENT_TAG_SCANNED: {
            rc522_tag_t *tag = (rc522_tag_t *)data->ptr;
            ESP_LOGI(TAG, "Tag scanned (UID: 0x%" PRIX64 ")", tag->uid);
        } break;
    }
}

void app_main()
{
    rc522_i2c_init();

    rc522_config_t config = {
        .send_handler = &send,
        .receive_handler = &receive,
    };

    rc522_create(&config, &rc522_handle);
    rc522_register_events(rc522_handle, RC522_EVENT_ANY, rc522_event_handler, NULL);
    rc522_start(rc522_handle);
}
