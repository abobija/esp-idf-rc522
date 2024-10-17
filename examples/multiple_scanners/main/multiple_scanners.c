#include <esp_log.h>
#include "rc522.h"
#include "driver/rc522_spi.h"
#include "rc522_picc.h"

static const char *TAG = "rc522-multiple-scanners-example";

// SPI bus GPIOs

#define RC522_SPI_BUS_GPIO_MISO (25)
#define RC522_SPI_BUS_GPIO_MOSI (23)
#define RC522_SPI_BUS_GPIO_SCLK (19)

// CS GPIOs for each scanner

#define RC522_SPI_SCANNER_1_GPIO_SDA (22)
#define RC522_SPI_SCANNER_2_GPIO_SDA (26)

// {{ Scanner configurations

static rc522_spi_config_t scanner_1_config = {
    .host_id = SPI3_HOST,
    .bus_config = &(spi_bus_config_t){
        .miso_io_num = RC522_SPI_BUS_GPIO_MISO,
        .mosi_io_num = RC522_SPI_BUS_GPIO_MOSI,
        .sclk_io_num = RC522_SPI_BUS_GPIO_SCLK,
    },
    .dev_config = {
        .spics_io_num = RC522_SPI_SCANNER_1_GPIO_SDA,
    },
    .rst_io_num = -1, // soft-reset
};

// Second scanner does not need bus configuration,
// since first scanner will initialize the bus.

static rc522_spi_config_t scanner_2_config = {
    .host_id = SPI3_HOST,
    .dev_config = {
        .spics_io_num = RC522_SPI_SCANNER_2_GPIO_SDA,
    },
    .rst_io_num = -1, // soft-reset
};

// }}

// Drivers

static rc522_driver_handle_t driver_1;
static rc522_driver_handle_t driver_2;

// Scanners

static rc522_handle_t scanner_1;
static rc522_handle_t scanner_2;

// Event handler

static void on_picc_state_changed(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    rc522_handle_t scanner = (rc522_handle_t)arg;
    rc522_picc_state_changed_event_t *event = (rc522_picc_state_changed_event_t *)data;
    rc522_picc_t *picc = event->picc;

    uint8_t scanner_no = 1;

    if (scanner == scanner_2) {
        scanner_no = 2;
    }

    if (picc->state == RC522_PICC_STATE_ACTIVE) {
        ESP_LOGI(TAG, "Card detected on the scanner #%d", scanner_no);
        rc522_picc_print(picc);
    }
    else if (picc->state == RC522_PICC_STATE_IDLE && event->old_state >= RC522_PICC_STATE_ACTIVE) {
        ESP_LOGI(TAG, "Card has been removed from the scanner #%d", scanner_no);
    }
}

// App entry point

void app_main()
{
    // Create drivers

    rc522_spi_create(&scanner_1_config, &driver_1);
    rc522_spi_create(&scanner_2_config, &driver_2);

    // Install drivers

    rc522_driver_install(driver_1);
    rc522_driver_install(driver_2);

    // Create scanners

    rc522_create(
        &(rc522_config_t) {
            .driver = driver_1,
        },
        &scanner_1);

    rc522_create(
        &(rc522_config_t) {
            .driver = driver_2,
        },
        &scanner_2);

    // Register events for each scanner

    rc522_register_events(scanner_1, RC522_EVENT_PICC_STATE_CHANGED, on_picc_state_changed, scanner_1);
    rc522_register_events(scanner_2, RC522_EVENT_PICC_STATE_CHANGED, on_picc_state_changed, scanner_2);

    // Start scanners

    rc522_start(scanner_1);
    rc522_start(scanner_2);
}
