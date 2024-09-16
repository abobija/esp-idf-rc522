#include <esp_log.h>
#include <esp_check.h>
#include <string.h>
#include "rc522.h"
#include "rc522_driver_spi.h"
#include "rc522_pcd.h"
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

#define COLUMN_SECTOR_WIDTH (6)
#define COLUMN_BLOCK_WIDTH  (7)
#define DUMP(format, ...)   esp_log_write(ESP_LOG_INFO, TAG, format, ##__VA_ARGS__)

static rc522_driver_handle_t driver;
static rc522_handle_t rc522;

static esp_err_t rc522_mifare_sector_dump(
    rc522_handle_t rc522, rc522_picc_t *picc, rc522_mifare_key_t *key, uint8_t sector_index)
{
    esp_err_t ret = ESP_OK;

    rc522_mifare_sector_t sector;
    ESP_RETURN_ON_ERROR(rc522_mifare_sector_info(sector_index, &sector), TAG, "");

    // Establish encrypted communications before reading the first block
    ESP_RETURN_ON_ERROR(rc522_mifare_autha(rc522, picc, sector.block_0_address, key), TAG, "auth failed");

    rc522_mifare_sector_trailer_t trailer;
    bool is_trailer = true;

    for (int8_t block_offset = sector.number_of_blocks - 1; block_offset >= 0; block_offset--) {
        uint8_t block_addr = sector.block_0_address + block_offset;

        // Sector number - only on first line
        if (is_trailer) {
            DUMP("%*d", COLUMN_SECTOR_WIDTH, sector_index);
        }
        else {
            DUMP("%*s", COLUMN_SECTOR_WIDTH, " ");
        }

        DUMP("%*d", COLUMN_BLOCK_WIDTH, block_addr);
        DUMP("  ");

        // Read block
        uint8_t buffer[18];
        uint8_t byte_count = sizeof(buffer);
        memset(buffer, 0x00, byte_count);

        ESP_RETURN_ON_ERROR(rc522_mifare_read(rc522, picc, block_addr, buffer, &byte_count), TAG, "read failed");

        // Dump data
        for (uint8_t index = 0; index < 16; index++) {
            DUMP("%02x", buffer[index]);

            if ((index % 4) == 3) {
                DUMP(" ");
            }
        }

        // Parse sector trailer data
        if (is_trailer) {
            ret = rc522_mifare_parse_sector_trailer(buffer, &trailer);
        }

        // Which access group is this block in?
        uint8_t group;       // 0-3 - active group for access bits
        bool first_in_group; // True for the first block dumped in the group

        if (sector.number_of_blocks == 4) {
            group = block_offset;
            first_in_group = true;
        }
        else {
            group = block_offset / 5;
            first_in_group = (group == 3) || (group != (block_offset + 1) / 5);
        }

        if (first_in_group) {
            // Print access bits
            DUMP("    %d  %d  %d",
                (trailer.access_bits[group] >> 2) & 1,
                (trailer.access_bits[group] >> 1) & 1,
                (trailer.access_bits[group] >> 0) & 1);

            if (ret == RC522_ERR_MIFARE_ACCESS_BITS_INTEGRITY_VIOLATION) {
                DUMP(" (access bits integrity violation)");
            }
        }

        if (ret == ESP_OK && group != 3 && rc522_mifare_block_is_value(trailer.access_bits[group])) {
            rc522_mifare_value_block_t block;
            ret = rc522_mifare_parse_value_block(buffer, &block);

            DUMP(" (value=%ld (0x%04lx), adr=0x%02x)", block.value, block.value, block.address);

            if (ret == RC522_ERR_MIFARE_VALUE_BLOCK_INTEGRITY_VIOLATION) {
                DUMP(" (value block integrity violation)");
            }
        }

        DUMP("\n");

        is_trailer = false;
    }

    return ret;
}

static esp_err_t rc522_mifare_memory_dump(rc522_handle_t rc522, rc522_picc_t *picc, rc522_mifare_key_t *key)
{
    rc522_mifare_t mifare;
    ESP_RETURN_ON_ERROR(rc522_mifare_info(picc, &mifare), TAG, "");

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

    // Dump sectors, highest address first.
    for (int8_t i = mifare.number_of_sectors - 1; i >= 0; i--) {
        if (rc522_mifare_sector_dump(rc522, picc, key, i) != ESP_OK) {
            break;
        }
    }

    ESP_RETURN_ON_ERROR(rc522_mifare_transactions_end(rc522, picc), TAG, "");

    return ESP_OK;
}

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

    rc522_mifare_memory_dump(rc522,
        picc,
        &(rc522_mifare_key_t) {
            .value = RC522_MIFARE_DEFAULT_KEY_VALUE,
        });
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
    rc522_start(rc522);
}
