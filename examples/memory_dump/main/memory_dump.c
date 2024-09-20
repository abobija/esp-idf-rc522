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
#define RC522_GPIO_RST            (-1) // soft-reset

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
    .rst_io_num = RC522_GPIO_RST,
};

static rc522_driver_handle_t driver;
static rc522_handle_t rc522;

#define DUMP(format, ...) esp_log_write(ESP_LOG_INFO, TAG, format, ##__VA_ARGS__)

static void dump_header()
{
    DUMP("Sector  Block                 Bytes                 AccessBits\n");
    DUMP("                0 1 2 3  4 5 6 7  8 9  11 12    15    c1 c2 c3\n");
}

static void dump_block(rc522_mifare_sector_block_t *block, uint8_t sector_index)
{
    if (block->type == RC522_MIFARE_BLOCK_TRAILER) {
        DUMP("%*d", 6, sector_index);
    }
    else {
        DUMP("%*s", 6, "");
    }

    // Block address
    DUMP("  %*d", 5, block->address);

    // Data
    DUMP("  ");
    for (uint8_t i = 0; i < 16; i++) {
        DUMP("%02" RC522_X "", block->bytes[i]);

        if ((i % 4) == 3) {
            DUMP(" ");
        }
    }

    // Access bits
    DUMP("    %d  %d  %d", block->access_bits.c1, block->access_bits.c2, block->access_bits.c3);

    // String representation (if it's data block)
    if (block->type == RC522_MIFARE_BLOCK_DATA) {
        DUMP("  |");

        for (uint8_t i = 0; i < 16; i++) {
            if (block->bytes[i] >= 32 && block->bytes[i] <= 126) { // standard ascii codes
                DUMP(LOG_COLOR(LOG_COLOR_GREEN) "%c" LOG_RESET_COLOR, block->bytes[i]);
            }
            else {
                DUMP("%c", '.');
            }
        }

        DUMP("|");
    }

    // Value (if it's value block)
    else if (block->type == RC522_MIFARE_BLOCK_VALUE) {
        DUMP("  (val=%ld (0x%04l" RC522_X "), adr=0x%02" RC522_X ")",
            block->value_data.value,
            block->value_data.value,
            block->value_data.addr);
    }

    // Errors and warnings
    if (block->trailer_data.err == RC522_ERR_MIFARE_ACCESS_BITS_INTEGRITY_VIOLATION) {
        DUMP("  ABITS_ERR");
    }

    if (block->value_data.err == RC522_ERR_MIFARE_VALUE_BLOCK_INTEGRITY_VIOLATION) {
        DUMP("  VAL_ERR");
    }

    // Termination
    DUMP("\n");
}

static esp_err_t dump_memory(rc522_handle_t rc522, rc522_picc_t *picc)
{
    rc522_mifare_key_t key = {
        .value = { RC522_MIFARE_KEY_VALUE_DEFAULT },
    };

    rc522_mifare_desc_t mifare;
    ESP_RETURN_ON_ERROR(rc522_mifare_get_desc(picc, &mifare), TAG, "");

    DUMP("\n");
    dump_header();

    // Start from the highest sector
    uint8_t sector_index = mifare.number_of_sectors - 1;

    do {
        rc522_mifare_sector_desc_t sector;
        ESP_RETURN_ON_ERROR(rc522_mifare_get_sector_desc(sector_index, &sector), TAG, "");
        ESP_RETURN_ON_ERROR(rc522_mifare_auth_sector(rc522, picc, &sector, &key), TAG, "");

        uint8_t block_offset = sector.number_of_blocks - 1;

        do {
            rc522_mifare_sector_block_t block;
            ESP_RETURN_ON_ERROR(rc522_mifare_read_sector_block(rc522, picc, &sector, block_offset, &block), TAG, "");

            dump_block(&block, sector_index);
        }
        while (block_offset--);
    }
    while (sector_index--);

    DUMP("\n");

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

    if (dump_memory(rc522, picc) == ESP_OK) {
        ESP_LOGI(TAG, "Memory dump success");
    }
    else {
        ESP_LOGE(TAG, "Memory dump failed");
    }

    if (rc522_mifare_deauth(rc522, picc) != ESP_OK) {
        ESP_LOGW(TAG, "Deauth failed");
    }
}

void app_main()
{
    rc522_spi_create(&driver_config, &driver);
    rc522_driver_install(driver);

    rc522_config_t config = {
        .driver = driver,
    };

    rc522_create(&config, &rc522);
    rc522_register_events(rc522, RC522_EVENT_PICC_STATE_CHANGED, on_picc_state_changed, NULL);
    rc522_start(rc522);
}
