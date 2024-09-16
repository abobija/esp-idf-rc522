#include <esp_log.h>
#include <esp_check.h>
#include <string.h>
#include "rc522.h"
#include "driver/rc522_spi.h"
#include "picc/rc522_mifare.h"

static const char *TAG = "rc522-memory-dump-example";

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

#define COLUMN_SECTOR_WIDTH (6)
#define COLUMN_BLOCK_WIDTH  (7)

#define DUMP(format, ...) esp_log_write(ESP_LOG_INFO, TAG, format, ##__VA_ARGS__)

static rc522_driver_handle_t driver;
static rc522_handle_t rc522;

static void rc522_mifare_dump_header()
{
    DUMP("%*s%*s                 Bytes                 AccessBits\n",
        COLUMN_SECTOR_WIDTH,
        "Sector",
        COLUMN_BLOCK_WIDTH,
        "Block");

    DUMP("%*s%*s   0 1 2 3  4 5 6 7  8 9  11 12    15    c1 c2 c3\n",
        COLUMN_SECTOR_WIDTH,
        " ",
        COLUMN_BLOCK_WIDTH,
        " ");
}

static esp_err_t dump_block(rc522_mifare_sector_block_t *block)
{
    uint8_t c1 = (block->access_bits >> 2) & 1;
    uint8_t c2 = (block->access_bits >> 1) & 1;
    uint8_t c3 = (block->access_bits >> 0) & 1;

    // Sector number only on first line
    if (block->type == RC522_MIFARE_BLOCK_TRAILER) {
        DUMP("%*d", COLUMN_SECTOR_WIDTH, block->sector_index);
    }
    else {
        DUMP("%*s", COLUMN_SECTOR_WIDTH, " ");
    }

    // Block address
    DUMP("%*d", COLUMN_BLOCK_WIDTH, block->address);
    DUMP("  ");

    // Data
    for (uint8_t i = 0; i < 16; i++) {
        DUMP("%02x", block->bytes[i]);

        if ((i % 4) == 3) {
            DUMP(" ");
        }
    }

    // Access bits
    DUMP("    %d  %d  %d", c1, c2, c3);

    // Value (if it's value block)
    if (block->type == RC522_MIFARE_BLOCK_VALUE) {
        DUMP(" (val=%ld (0x%04lx), adr=0x%02x)", block->value->value, block->value->value, block->value->address);
    }

    // Errors and warnings
    if (block->access_bits_err == RC522_ERR_MIFARE_ACCESS_BITS_INTEGRITY_VIOLATION) {
        DUMP(" AB_VIOL");
    }

    if (block->value_err == RC522_ERR_MIFARE_VALUE_BLOCK_INTEGRITY_VIOLATION) {
        DUMP(" VB_VIOL");
    }

    // Termination
    DUMP("\n");

    return ESP_OK;
}

static esp_err_t dump_memory(rc522_handle_t rc522, rc522_picc_t *picc, rc522_mifare_key_t *key)
{
    rc522_mifare_t mifare;
    ESP_RETURN_ON_ERROR(rc522_mifare_info(picc, &mifare), TAG, "");

    rc522_mifare_dump_header();

    esp_err_t ret = ESP_OK;

    // Start from the highest sector
    for (int8_t sector = mifare.number_of_sectors - 1; sector >= 0; sector--) {
        ret = rc522_mifare_iterate_sector_blocks(rc522, picc, sector, key, dump_block);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "sector iteration failed");
            break;
        }
    }

    ESP_RETURN_ON_ERROR(rc522_mifare_transactions_end(rc522, picc), TAG, "");

    return ret;
}

static void on_picc_selected(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    rc522_picc_t *picc = (rc522_picc_t *)data;

    ESP_LOGI(TAG, "PICC detected (sak=%02x, type=%s)", picc->sak, rc522_picc_type_name(picc->type));
    ESP_LOGI(TAG, "UID:");
    ESP_LOG_BUFFER_HEX(TAG, picc->uid.value, picc->uid.length);

    if (rc522_mifare_type_is_classic_compatible(picc->type)) {
        if (dump_memory(rc522,
                picc,
                &(rc522_mifare_key_t) {
                    .value = RC522_MIFARE_DEFAULT_KEY_VALUE,
                })) {
            ESP_LOGE(TAG, "memory dump failed");
        }

        return;
    }

    ESP_LOGW(TAG, "Memory dump not implemented for PICC of type %02x", picc->type);

    return;
}

void app_main()
{
    rc522_spi_create(&driver_config, &driver);
    rc522_driver_install(driver);

    rc522_config_t config = {
        .driver = driver,
    };

    rc522_create(&config, &rc522);
    rc522_register_events(rc522, RC522_EVENT_PICC_SELECTED, on_picc_selected, NULL);
    rc522_start(rc522);
}
