#include <esp_log.h>
#include <inttypes.h>
#include <driver/i2c.h>
#include "rc522.h"

static const char *TAG = "rc522-i2c-example";

#define RC522_I2C_ADDRESS        (0x28)
#define RC522_I2C_RW_TIMEOUT_MS  (1000)
#define RC522_I2C_CLOCK_SPEED_HZ (100000)
#define RC522_I2C_PORT           (I2C_NUM_0)
#define RC522_I2C_GPIO_SDA       (21)
#define RC522_I2C_GPIO_SCL       (22)

static rc522_handle_t rc522_handle;

static esp_err_t rc522_i2c_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = RC522_I2C_GPIO_SDA,
        .scl_io_num = RC522_I2C_GPIO_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = RC522_I2C_CLOCK_SPEED_HZ,
    };

    esp_err_t err = i2c_param_config(RC522_I2C_PORT, &conf);

    if (err != ESP_OK) {
        return err;
    }

    return i2c_driver_install(RC522_I2C_PORT, conf.mode, false, false, 0x00);
}

static esp_err_t rc522_i2c_send(uint8_t *buffer, uint8_t length)
{
    return i2c_master_write_to_device(RC522_I2C_PORT,
        RC522_I2C_ADDRESS,
        buffer,
        length,
        RC522_I2C_RW_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t rc522_i2c_receive(uint8_t *buffer, uint8_t length, uint8_t address)
{
    return i2c_master_write_read_device(RC522_I2C_PORT,
        RC522_I2C_ADDRESS,
        &address,
        1,
        buffer,
        length,
        RC522_I2C_RW_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t rc522_send(uint8_t *buffer, uint8_t length)
{
    return rc522_i2c_send(buffer, length);
}

static esp_err_t rc522_receive(uint8_t *buffer, uint8_t length, uint8_t address)
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
        .send_handler = &rc522_send,
        .receive_handler = &rc522_receive,
    };

    rc522_create(&config, &rc522_handle);
    rc522_register_events(rc522_handle, RC522_EVENT_ANY, rc522_event_handler, NULL);
    rc522_start(rc522_handle);
}
