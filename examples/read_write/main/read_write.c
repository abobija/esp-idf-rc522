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
    .host_id = SPI3_HOST,
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
    const char *data_to_write = "rc522 is dope";
    const uint8_t block_address = 4;
    rc522_mifare_key_t key = {
        .value = { RC522_MIFARE_KEY_VALUE_DEFAULT },
    };

    if (strlen(data_to_write) > 14) {
        ESP_LOGW(TAG, "Please make sure that data length is no more than 14 characters");
        ESP_LOGW(TAG, "since we are going to use random values for last two bytes");

        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(rc522_mifare_auth(scanner, picc, block_address, &key), TAG, "auth fail");

    uint8_t read_buffer[RC522_MIFARE_BLOCK_SIZE];
    uint8_t write_buffer[RC522_MIFARE_BLOCK_SIZE];

    // Read
    ESP_LOGI(TAG, "Reading data from the block %d", block_address);
    ESP_RETURN_ON_ERROR(rc522_mifare_read(scanner, picc, block_address, read_buffer), TAG, "read fail");
    ESP_LOGI(TAG, "Current data:");
    dump_block(read_buffer);
    // ~Read

    // Write
    strncpy((char *)write_buffer, data_to_write, RC522_MIFARE_BLOCK_SIZE);

    // Set random values for the last two bytes in the write buffer
    // so we are using new data on each call of read_write function
    int r = rand();
    write_buffer[RC522_MIFARE_BLOCK_SIZE - 2] = ((r >> 8) & 0xFF);
    write_buffer[RC522_MIFARE_BLOCK_SIZE - 1] = ((r >> 0) & 0xFF);

    ESP_LOGI(TAG, "Writing data (%s) to the block %d:", data_to_write, block_address);
    dump_block(write_buffer);
    ESP_RETURN_ON_ERROR(rc522_mifare_write(scanner, picc, block_address, write_buffer), TAG, "write fail");
    // ~Write

    // Read again
    ESP_LOGI(TAG, "Write done. Verifying...");
    ESP_RETURN_ON_ERROR(rc522_mifare_read(scanner, picc, block_address, read_buffer), TAG, "read fail");
    ESP_LOGI(TAG, "New data in the block %d:", block_address);
    dump_block(read_buffer);
    // ~Read again

    // Validate
    bool rw_missmatch = false;
    uint8_t i;
    for (i = 0; i < RC522_MIFARE_BLOCK_SIZE; i++) {
        if (write_buffer[i] != read_buffer[i]) {
            rw_missmatch = true;
            break;
        }
    }
    // ~Validate

    // Feedback
    if (!rw_missmatch) {
        ESP_LOGI(TAG, "Verified.");
    }
    else {
        ESP_LOGE(TAG,
            "Write failed. RW missmatch on the byte %d (w:%02" RC522_X ", r:%02" RC522_X ")",
            i,
            write_buffer[i],
            read_buffer[i]);

        dump_block(write_buffer);
        dump_block(read_buffer);
    }
    // ~Feedback

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
