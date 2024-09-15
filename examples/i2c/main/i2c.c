#include <esp_log.h>
#include <driver/i2c.h>
#include "rc522.h"
#include "picc/rc522_mifare.h"

static const char *TAG = "rc522-i2c-example";

#define RC522_I2C_ADDRESS       (0x28)
#define RC522_I2C_RW_TIMEOUT_MS (1000)
#define RC522_I2C_PORT          (I2C_NUM_0)
#define RC522_I2C_GPIO_SDA      (18)
#define RC522_I2C_GPIO_SCL      (21)

static i2c_config_t i2c_config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = RC522_I2C_GPIO_SDA,
    .scl_io_num = RC522_I2C_GPIO_SCL,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000,
};

static rc522_handle_t rc522_handle;

static esp_err_t rc522_send(uint8_t *buffer, uint8_t length)
{
    return i2c_master_write_to_device(RC522_I2C_PORT,
        RC522_I2C_ADDRESS,
        buffer,
        length,
        RC522_I2C_RW_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t rc522_receive(uint8_t address, uint8_t *buffer, uint8_t length)
{
    return i2c_master_write_read_device(RC522_I2C_PORT,
        RC522_I2C_ADDRESS,
        &address,
        1,
        buffer,
        length,
        RC522_I2C_RW_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t rc522_picc_dump(rc522_handle_t rc522, rc522_picc_t *picc)
{
    ESP_LOGI(TAG, "PICC (sak=%02x, type=%s)", picc->sak, rc522_picc_type_name(picc->type));
    ESP_LOGI(TAG, "UID:");
    ESP_LOG_BUFFER_HEX(TAG, picc->uid.bytes, picc->uid.bytes_length);

    if (rc522_mifare_type_is_classic_compatible(picc->type)) {
        return rc522_mifare_dump(rc522,
            picc,
            &(rc522_mifare_key_t) {
                .value = RC522_MIFARE_DEFAULT_KEY_VALUE,
            });
    }

    ESP_LOGW(TAG, "Dumping data not implemented for PICC of type %02x", picc->type);

    return ESP_ERR_INVALID_ARG;
}

static void rc522_event_handler(void *arg, esp_event_base_t base, int32_t event_id, void *event_data)
{
    rc522_event_data_t *data = (rc522_event_data_t *)event_data;

    switch (event_id) {
        case RC522_EVENT_PICC_SELECTED: {
            rc522_picc_t *picc = (rc522_picc_t *)data->ptr;

            ESP_LOGI(TAG, "PICC scanned");
            rc522_picc_dump(rc522_handle, picc);
        } break;
    }
}

void app_main()
{
    i2c_param_config(RC522_I2C_PORT, &i2c_config);
    i2c_driver_install(RC522_I2C_PORT, i2c_config.mode, 0, 0, 0x00);

    rc522_config_t config = {
        .send_handler = &rc522_send,
        .receive_handler = &rc522_receive,
    };

    rc522_create(&config, &rc522_handle);
    rc522_register_events(rc522_handle, RC522_EVENT_ANY, rc522_event_handler, NULL);
    rc522_start(rc522_handle);
}
