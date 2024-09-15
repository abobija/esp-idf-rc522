#include <esp_log.h>
#include "rc522.h"
#include "rc522_driver_spi.h"
#include "picc/rc522_mifare.h"

static const char *TAG = "rc522-basic-example";

#define RC522_SPI_BUS_GPIO_MISO   (25)
#define RC522_SPI_BUS_GPIO_MOSI   (23)
#define RC522_SPI_BUS_GPIO_SCLK   (19)
#define RC522_SPI_DEVICE_GPIO_SDA (22)

static rc522_driver_config_t driver_config = {
    .spi = {
        .host_id = VSPI_HOST,
        .dma_chan = SPI_DMA_DISABLED,
        .bus_config = &(spi_bus_config_t){
            .miso_io_num = RC522_SPI_BUS_GPIO_MISO,
            .mosi_io_num = RC522_SPI_BUS_GPIO_MOSI,
            .sclk_io_num = RC522_SPI_BUS_GPIO_SCLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
        },
        .dev_config = {
            .clock_speed_hz = 5000000,
            .mode = 0,
            .spics_io_num = RC522_SPI_DEVICE_GPIO_SDA,
            .queue_size = 7,
            .flags = SPI_DEVICE_HALFDUPLEX,
        },
    },
};

static rc522_driver_handle_t driver;
static rc522_handle_t rc522;

static esp_err_t rc522_picc_dump(rc522_handle_t _rc522, rc522_picc_t *picc)
{
    ESP_LOGI(TAG, "PICC (sak=%02x, type=%s)", picc->sak, rc522_picc_type_name(picc->type));
    ESP_LOGI(TAG, "UID:");
    ESP_LOG_BUFFER_HEX(TAG, picc->uid.value, picc->uid.length);

    if (rc522_mifare_type_is_classic_compatible(picc->type)) {
        return rc522_mifare_dump(_rc522,
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

            ESP_LOGI(TAG, "PICC detected");
            rc522_picc_dump(rc522, picc);
        } break;
    }
}

void app_main()
{
    rc522_driver_spi_create(&driver_config, &driver);
    rc522_driver_install(driver);

    rc522_config_t config = {
        .driver = driver,
    };

    rc522_create(&config, &rc522);
    rc522_register_events(rc522, RC522_EVENT_ANY, rc522_event_handler, NULL);
    rc522_start(rc522);
}
