#include <esp_log.h>
#include "rc522.h"
#include "rc522_driver_i2c.h"
#include "rc522_picc.h"

static const char *TAG = "rc522-basic-i2c-example";

#define RC522_I2C_ADDRESS  (0x28)
#define RC522_I2C_GPIO_SDA (18)
#define RC522_I2C_GPIO_SCL (21)

static rc522_driver_config_t driver_config = {
    .i2c = {
        .port = I2C_NUM_0,
        .device_address = RC522_I2C_ADDRESS,
        .rw_timeout_ms = 1000,
        .config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = RC522_I2C_GPIO_SDA,
            .scl_io_num = RC522_I2C_GPIO_SCL,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 100000,
        },
    },
};

static rc522_driver_handle_t driver;
static rc522_handle_t rc522;

static void on_picc_selected(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    rc522_picc_t *picc = (rc522_picc_t *)data;

    ESP_LOGI(TAG, "PICC detected (sak=%02x, type=%s)", picc->sak, rc522_picc_type_name(picc->type));
    ESP_LOGI(TAG, "UID:");
    ESP_LOG_BUFFER_HEX(TAG, picc->uid.value, picc->uid.length);
}

void app_main()
{
    rc522_driver_i2c_create(&driver_config, &driver);
    rc522_driver_install(driver);

    rc522_config_t config = {
        .driver = driver,
    };

    rc522_create(&config, &rc522);
    rc522_register_events(rc522, RC522_EVENT_PICC_SELECTED, on_picc_selected, NULL);
    rc522_start(rc522);
}
