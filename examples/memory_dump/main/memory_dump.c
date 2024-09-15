#include <esp_log.h>
#include "rc522.h"
#include "rc522_driver_spi.h"
#include "picc/rc522_mifare.h"

static const char *TAG = "rc522-memory-dump-example";

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

static void rc522_on_picc_selected(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    rc522_picc_t *picc = (rc522_picc_t *)data;

    ESP_LOGI(TAG, "PICC detected (sak=%02x, type=%s)", picc->sak, rc522_picc_type_name(picc->type));
    ESP_LOGI(TAG, "UID:");
    ESP_LOG_BUFFER_HEX(TAG, picc->uid.value, picc->uid.length);

    if (!rc522_mifare_type_is_classic_compatible(picc->type)) {
        ESP_LOGW(TAG, "Dumping data not implemented for PICC of type %02x", picc->type);
        return;
    }

    rc522_mifare_stream_memory_dump(rc522,
        picc,
        &(rc522_mifare_key_t) {
            .value = RC522_MIFARE_DEFAULT_KEY_VALUE,
        });
}

static void rc522_on_streaming(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    esp_log_write(ESP_LOG_INFO, TAG, "%s", (const char *)data);
}

void app_main()
{
    rc522_driver_spi_create(&driver_config, &driver);
    rc522_driver_install(driver);

    rc522_config_t config = {
        .driver = driver,
    };

    rc522_create(&config, &rc522);
    rc522_register_events(rc522, RC522_EVENT_PICC_SELECTED, rc522_on_picc_selected, NULL);
    rc522_register_events(rc522, RC522_EVENT_STREAMING, rc522_on_streaming, NULL);
    rc522_start(rc522);
}
