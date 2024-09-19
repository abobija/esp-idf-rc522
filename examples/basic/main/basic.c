#include <esp_log.h>
#include "rc522.h"
#include "driver/rc522_spi.h"
#include "rc522_picc.h"

static const char *TAG = "rc522-basic-example";

#define RC522_SPI_BUS_GPIO_MISO   (25)
#define RC522_SPI_BUS_GPIO_MOSI   (23)
#define RC522_SPI_BUS_GPIO_SCLK   (19)
#define RC522_SPI_DEVICE_GPIO_SDA (22)

static rc522_spi_config_t driver_config = {
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
};

static rc522_driver_handle_t driver;
static rc522_handle_t rc522;

static void on_picc_activated(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    rc522_picc_t *picc = (rc522_picc_t *)data;

    char uid_str[RC522_PICC_UID_STR_BUFFER_SIZE_MAX];
    rc522_picc_uid_to_str(&picc->uid, uid_str, RC522_PICC_UID_STR_BUFFER_SIZE_MAX);

    ESP_LOGI(TAG, "PICC (type=%s, uid=%s, sak=%02x) detected", rc522_picc_type_name(picc->type), uid_str, picc->sak);
}

static void on_picc_disappeared(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    // the UID is available here in same way as
    // in the `on_picc_activated` handler above

    ESP_LOGI(TAG, "PICC has been removed");
}

void app_main()
{
    rc522_spi_create(&driver_config, &driver);
    rc522_driver_install(driver);

    rc522_config_t config = {
        .driver = driver,
    };

    rc522_create(&config, &rc522);
    rc522_register_events(rc522, RC522_EVENT_PICC_ACTIVATED, on_picc_activated, NULL);
    rc522_register_events(rc522, RC522_EVENT_PICC_DISAPPEARED, on_picc_disappeared, NULL);
    rc522_start(rc522);
}
