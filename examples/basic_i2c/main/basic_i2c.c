#include <esp_log.h>
#include "rc522.h"
#include "driver/rc522_i2c.h"
#include "rc522_picc.h"

static const char *TAG = "rc522-basic-i2c-example";

#define RC522_I2C_ADDRESS      (0x28)
#define RC522_I2C_GPIO_SDA     (18)
#define RC522_I2C_GPIO_SCL     (21)
#define RC522_SCANNER_GPIO_RST (-1) // soft-reset

static rc522_i2c_config_t driver_config = { 
    .bus_config = &(i2c_master_bus_config_t){
        .i2c_port = I2C_NUM_0,
        .sda_io_num = RC522_I2C_GPIO_SDA,
        .scl_io_num = RC522_I2C_GPIO_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    },
    .dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = RC522_I2C_ADDRESS,
        .scl_speed_hz = 100000,
    },
    .rst_io_num = RC522_SCANNER_GPIO_RST,
};

static rc522_driver_handle_t driver;
static rc522_handle_t scanner;

static void on_picc_state_changed(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    rc522_picc_state_changed_event_t *event = (rc522_picc_state_changed_event_t *)data;

    if (event->picc->state != RC522_PICC_STATE_ACTIVE) {
        return;
    }

    rc522_picc_print(event->picc);
}

void app_main()
{
    rc522_i2c_create(&driver_config, &driver);
    rc522_driver_install(driver);

    rc522_config_t scanner_config = {
        .driver = driver,
    };

    rc522_create(&scanner_config, &scanner);
    rc522_register_events(scanner, RC522_EVENT_PICC_STATE_CHANGED, on_picc_state_changed, NULL);
    rc522_start(scanner);
}
