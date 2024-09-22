#include <esp_log.h>
#include "rc522.h"
#include "driver/rc522_spi.h"
#include "rc522_picc.h"

static const char *TAG = "rc522-multiple-readers-example";

// SPI bus GPIOs

#define RC522_SPI_BUS_GPIO_MISO (25)
#define RC522_SPI_BUS_GPIO_MOSI (23)
#define RC522_SPI_BUS_GPIO_SCLK (19)

// CS GPIOs for each reader

#define RC522_SPI_READER_1_GPIO_SDA (22)
#define RC522_SPI_READER_2_GPIO_SDA (26)

// {{ Reader configurations

static rc522_spi_config_t reader_1_config = {
    .host_id = VSPI_HOST,
    .bus_config = &(spi_bus_config_t){
        .miso_io_num = RC522_SPI_BUS_GPIO_MISO,
        .mosi_io_num = RC522_SPI_BUS_GPIO_MOSI,
        .sclk_io_num = RC522_SPI_BUS_GPIO_SCLK,
    },
    .dev_config = {
        .spics_io_num = RC522_SPI_READER_1_GPIO_SDA,
    },
    .rst_io_num = -1, // soft-reset
};

// Second reader does not need bus configuration,
// since first reader will initialize the bus.

static rc522_spi_config_t reader_2_config = {
    .host_id = VSPI_HOST,
    .dev_config = {
        .spics_io_num = RC522_SPI_READER_2_GPIO_SDA,
    },
    .rst_io_num = -1, // soft-reset
};

// }}

// Drivers

static rc522_driver_handle_t driver_1;
static rc522_driver_handle_t driver_2;

// Readers

static rc522_handle_t reader_1;
static rc522_handle_t reader_2;

// Event handler

static void on_picc_state_changed(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    rc522_handle_t reader = (rc522_handle_t)arg;
    rc522_picc_state_changed_event_t *event = (rc522_picc_state_changed_event_t *)data;
    rc522_picc_t *picc = event->picc;

    uint8_t reader_no = 1;

    if (reader == reader_2) {
        reader_no = 2;
    }

    if (picc->state == RC522_PICC_STATE_ACTIVE) {
        ESP_LOGI(TAG, "Card detected on the scanner #%d", reader_no);
        rc522_picc_print(picc);
    }
    else if (picc->state == RC522_PICC_STATE_IDLE && event->old_state >= RC522_PICC_STATE_ACTIVE) {
        ESP_LOGI(TAG, "Card has been removed from the scanner #%d", reader_no);
    }
}

// App entry point

void app_main()
{
    // Create drivers

    rc522_spi_create(&reader_1_config, &driver_1);
    rc522_spi_create(&reader_2_config, &driver_2);

    // Install drivers

    rc522_driver_install(driver_1);
    rc522_driver_install(driver_2);

    // Create readers

    rc522_create(
        &(rc522_config_t) {
            .driver = driver_1,
        },
        &reader_1);

    rc522_create(
        &(rc522_config_t) {
            .driver = driver_2,
        },
        &reader_2);

    // Register events for each reader

    rc522_register_events(reader_1, RC522_EVENT_PICC_STATE_CHANGED, on_picc_state_changed, reader_1);
    rc522_register_events(reader_2, RC522_EVENT_PICC_STATE_CHANGED, on_picc_state_changed, reader_2);

    // Start readers

    rc522_start(reader_1);
    rc522_start(reader_2);
}
