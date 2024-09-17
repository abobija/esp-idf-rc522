#include <esp_log.h>
#include "rc522.h"
#include "driver/rc522_i2c.h"
#include "rc522_picc.h"

static const char *TAG = "rc522-basic-i2c-example";

#define RC522_I2C_ADDRESS  (0x28)
#define RC522_I2C_GPIO_SDA (18)
#define RC522_I2C_GPIO_SCL (21)

static rc522_i2c_config_t driver_config = {
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
};

static rc522_driver_handle_t driver;
static rc522_handle_t rc522;

static void on_picc_activated(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    rc522_picc_t *picc = (rc522_picc_t *)data;

    char uid_str[RC522_PICC_UID_STR_BUFFER_SIZE];
    rc522_picc_uid_to_str(&picc->uid, uid_str);

    ESP_LOGI(TAG, "PICC (type=%s, uid=%s, sak=%02x) detected", rc522_picc_type_name(picc->type), uid_str, picc->sak);
}

void app_main()
{
    rc522_i2c_create(&driver_config, &driver);
    rc522_driver_install(driver);

    rc522_config_t config = {
        .driver = driver,
    };

    rc522_create(&config, &rc522);
    rc522_register_events(rc522, RC522_EVENT_PICC_ACTIVATED, on_picc_activated, NULL);
    rc522_start(rc522);
}
