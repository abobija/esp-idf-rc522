#include <esp_log.h>
#include <esp_check.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include "rc522.h"
#include "driver/rc522_spi.h"
#include "picc/rc522_mifare.h"

static const char *TAG = "rc522-read-write-example";

#define RC522_SPI_BUS_GPIO_MISO    (25)
#define RC522_SPI_BUS_GPIO_MOSI    (23)
#define RC522_SPI_BUS_GPIO_SCLK    (19)
#define RC522_SPI_SCANNER_GPIO_SDA (22)
#define RC522_SCANNER_GPIO_RST     (-1) // soft-reset

static rc522_spi_config_t driver_config = {
    .host_id = VSPI_HOST,
    .bus_config = &(spi_bus_config_t){
        .miso_io_num = RC522_SPI_BUS_GPIO_MISO,
        .mosi_io_num = RC522_SPI_BUS_GPIO_MOSI,
        .sclk_io_num = RC522_SPI_BUS_GPIO_SCLK,
    },
    .dev_config = {
        .spics_io_num = RC522_SPI_SCANNER_GPIO_SDA,
    },
    .rst_io_num = RC522_SCANNER_GPIO_RST,
};

static rc522_driver_handle_t driver;
static rc522_handle_t scanner;

static void dump_block(uint8_t buffer[RC522_MIFARE_BLOCK_SIZE])
{
    for (uint8_t i = 0; i < RC522_MIFARE_BLOCK_SIZE; i++) {
        esp_log_write(ESP_LOG_INFO, TAG, "%02" RC522_X " ", buffer[i]);
    }

    esp_log_write(ESP_LOG_INFO, TAG, "\n");
}

static esp_err_t read_write(rc522_handle_t scanner, rc522_picc_t *picc)
{
    rc522_mifare_key_t key = {
        .value = { RC522_MIFARE_KEY_VALUE_DEFAULT },
        .type = RC522_MIFARE_KEY_A,
    };

    ESP_RETURN_ON_ERROR(rc522_mifare_auth(scanner, picc, 3, &key), TAG, "auth fail");

    uint8_t buffer[RC522_MIFARE_BLOCK_SIZE];
    ESP_RETURN_ON_ERROR(rc522_mifare_read(scanner, picc, 3, buffer), TAG, "");

    ESP_LOGI(TAG, "Block 3 init:");
    dump_block(buffer);

    memcpy(buffer, key.value, sizeof(key.value));         // set key a
    memcpy(buffer + 6 + 4, key.value, sizeof(key.value)); // set key b

    ESP_LOGI(TAG, "Block 3 after keys are set:");
    dump_block(buffer);

    /**
     * [6]: 1111 1111
     * [7]: 0000 0111
     * [8]: 1000 0000
     */

    buffer[7] |= 0x40;    // C12 = 1
    buffer[8] |= 0x04;    // C22 = 1
    buffer[8] &= (~0x40); // C32 = 0

    buffer[6] &= (~0x04); // ~C12 = 0
    buffer[6] &= (~0x40); // ~C22 = 0
    buffer[7] |= 0x04;    // ~C32 = 1

    /**
     * [6]: 1011 1011
     * [7]: 0100 0111
     * [8]: 1000 0100
     */

    ESP_LOGI(TAG, "Block 3 before write:");
    dump_block(buffer);

    ESP_RETURN_ON_ERROR(rc522_mifare_write(scanner, picc, 3, buffer), TAG, "");

    ESP_RETURN_ON_ERROR(rc522_mifare_read(scanner, picc, 3, buffer), TAG, "");

    ESP_LOGI(TAG, "Block 3 after write:");
    dump_block(buffer);

    return ESP_OK;
}

static void on_picc_state_changed(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    rc522_picc_state_changed_event_t *event = (rc522_picc_state_changed_event_t *)data;
    rc522_picc_t *picc = event->picc;

    if (picc->state != RC522_PICC_STATE_ACTIVE) {
        return;
    }

    rc522_picc_print(picc);

    if (!rc522_mifare_type_is_classic_compatible(picc->type)) {
        ESP_LOGW(TAG, "Card is not supported by this example");
        return;
    }

    if (read_write(scanner, picc) == ESP_OK) {
        ESP_LOGI(TAG, "Read/Write success");
    }
    else {
        ESP_LOGE(TAG, "Read/Write failed");
    }

    if (rc522_mifare_deauth(scanner, picc) != ESP_OK) {
        ESP_LOGW(TAG, "Deauth failed");
    }
}

void app_main()
{
    srand(time(NULL)); // Initialize random generator

    rc522_spi_create(&driver_config, &driver);
    rc522_driver_install(driver);

    rc522_config_t scanner_config = {
        .driver = driver,
    };

    rc522_create(&scanner_config, &scanner);
    rc522_register_events(scanner, RC522_EVENT_PICC_STATE_CHANGED, on_picc_state_changed, NULL);
    rc522_start(scanner);
}
