#include <esp_log.h>
#include <esp_check.h>
#include <string.h>
#include "driver/spi_master.h"
#include "rc522.h"
#include "rc522_pcd.h"
#include "driver/rc522_spi.h"
#include "picc/rc522_mifare.h"
#include "picc/rc522_nxp.h"

#include "rc522_picc.h"

static const char *TAG = "rc522-picc-nxp-example";

#define RC522_SPI_BUS_GPIO_MISO    (16)
#define RC522_SPI_BUS_GPIO_MOSI    (15)
#define RC522_SPI_BUS_GPIO_SCLK    (17)
#define RC522_SPI_SCANNER_GPIO_SDA (1)
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
static char str_buf[128];

#define DUMP(format, ...) esp_log_write(ESP_LOG_INFO, TAG, format, ##__VA_ARGS__)

static void dump_header()
{
    DUMP("Page | Offset:  +0           +1           +2           +3\n");
    //     00  | 10 20 30 40  50 60 70 80  90 A0 B0 C0  D0 E0 F0 11
}

// Dump four pages (size of a single READ) to output, annotating the start
// and end pages of user memory if present
static void dump_pages(uint8_t address, uint8_t page[RC522_NXP_READ_SIZE], rc522_picc_type_t picc_type)
{
    // Get the start and end pages of user memory so we can mark them in the dump
    uint8_t start_page = rc522_nxp_get_user_mem_start(picc_type);
    uint8_t end_page = rc522_nxp_get_user_mem_end(picc_type);
    // 0: nothing of interest, 1: start page, 2: end page
    uint8_t found = 0;
    DUMP("  %02X |", address);
    for (uint8_t i = 0; i < RC522_NXP_READ_SIZE; i++) {
        DUMP(" %02X", page[i]);
        if (i / 4 + address == start_page && i % 4 == 3) {
            found = 1;
            DUMP("*");
        }
        else if (i / 4 + address == end_page && i % 4 == 3) {
            found = 2;
            DUMP("*");
        }
        else if (i % 4 == 3) {
            DUMP(" ");
        }
    }
    if (found == 1) {
        DUMP(" * Start of user memory (0x%02X)", start_page);
    }
    else if (found == 2) {
        DUMP(" * End of user memory (0x%02X)", end_page);
    }
    DUMP("\n");
}

// Dump all memory to output
static esp_err_t dump_memory(rc522_handle_t scanner, rc522_picc_t *picc)
{
    DUMP("\n");
    dump_header();

    uint8_t page_count = rc522_nxp_get_page_count(picc->type);
    // Standard read is 4 pages; allocate an appropriate buffer and increment by
    // 4 pages on each loop
    uint8_t recv[RC522_NXP_PAGE_SIZE * 4];
    for (uint8_t i = 0; i < page_count; i += 4) {
        ESP_RETURN_ON_ERROR(rc522_nxp_read(scanner, picc, i, recv), TAG, "");
        dump_pages(i, recv, picc->type);
    }
    DUMP("\n");
    return ESP_OK;
}

static void buf_to_hex(const uint8_t *buffer, uint8_t buflen, char *strbuf, uint8_t strbuflen)
{
    const char *format = "%02X ";

    uint16_t len = 0;
    for (uint16_t i = 0; i < buflen; i++) {
        len += sprintf(strbuf + len, format, buffer[i]);
    }

    strbuf[len - 1] = 0x00;
}

// Example: Get the total and user memory page count of an NXP PICC
static void example_page_counts(rc522_handle_t rc522, rc522_picc_t *picc)
{
    uint8_t page_count = rc522_nxp_get_page_count(picc->type);
    uint8_t user_page_count = rc522_nxp_get_user_page_count(picc->type);
    ESP_LOGI(TAG, "User pages: %d/%d", user_page_count, page_count);
}

// Example: Write to and read from user memory, verifying write works
static esp_err_t example_read_write(rc522_handle_t rc522, rc522_picc_t *picc)
{
    uint8_t page = 5;
    uint8_t write_buf[4] = { 0x10, 0x20, 0x30, 0x40 };
    esp_err_t ret = rc522_nxp_write(scanner, picc, page, write_buf);
    ESP_LOGI(TAG, "Writing to address %02X: R=%02X", page, ret);
    ESP_RETURN_ON_ERROR(ret, TAG, "Error in WRITE");

    uint8_t read_buf[16] = { 0 };
    ret = rc522_nxp_read(scanner, picc, page, read_buf);
    ESP_LOGI(TAG, "Reading from address %02X: R=%02X", page, ret);
    ESP_RETURN_ON_ERROR(ret, TAG, "Error in READ");

    if (memcmp(write_buf, read_buf, 4) == 0) {
        ESP_LOGI(TAG, "Data read/write match");
    }
    else {
        ESP_LOGW(TAG, "Data read/write mismatch");
    }

    buf_to_hex(read_buf, 16, str_buf, sizeof(str_buf));
    ESP_LOGI(TAG, "Pages %d to %d: %s", page, page + 3, str_buf);
    return ESP_OK;
}

// Example: Use of FAST_READ command for variable-size read
static esp_err_t example_fast_read(rc522_handle_t rc522, rc522_picc_t *picc)
{
    uint8_t page_count = 3;
    uint8_t start_page = 4;
    // Need to allocate 4 bytes per page
    uint8_t *mem_buf = calloc(page_count, 4);
    ESP_RETURN_ON_FALSE(mem_buf != NULL, ESP_FAIL, TAG, "Out of memory");

    rc522_nxp_fast_read_data_t mem_data = {
        .bytes = mem_buf,
        .buffer_size = page_count * 4,
    };

    esp_err_t ret = rc522_nxp_fast_read(scanner,
        picc,
        start_page,
        start_page + page_count - 1, // end is inclusive, so subtract 1
        &mem_data);
    ESP_LOGI(TAG, "Dumping %d pages: R=%02X", page_count, ret);
    ESP_RETURN_ON_ERROR(ret, TAG, "Error in FAST_READ");

    for (uint8_t i = 0; i < page_count; i++) {
        buf_to_hex(&mem_buf[i * 4], 4, str_buf, 128);
        ESP_LOGI(TAG, "Page %03d: %s", start_page + i, str_buf);
    }
    free(mem_buf);
    return ESP_OK;
}

// Example: Password authentication with PWD/PACK
static esp_err_t example_pwd_auth(rc522_handle_t rc522, rc522_picc_t *picc)
{
    // Note both PWD and PACK can and should be programmed during initial
    // PICC setup
    esp_err_t ret = rc522_nxp_pwd_auth(scanner,
        picc,
        RC522_NXP_DEFAULT_PWD,  // 4-byte password, default is 0xFFFFFFFF
        RC522_NXP_DEFAULT_PACK, // 2-byte acknowledge, default is 0x0000
        &picc->state            // Ensure the card state is updated if successful
    );
    ESP_LOGI(TAG, "Password auth: R=%02X", ret);
    return ret;
}

// Example: Counter reading
static esp_err_t example_read_cnt(rc522_handle_t rc522, rc522_picc_t *picc)
{
    // Counters can only be read if they're enabled, otherwise the PICC will
    // just respond with a NAK. On the NTAG21x, the single counter can be
    // enabled in the config section, so read it and check if it's enabled.
    // 0x83 for NTAG215; address will vary on other PICCs. Check your datasheet
    uint8_t read_buf[16];
    esp_err_t ret = rc522_nxp_read(scanner, picc, 0x83, read_buf);
    ESP_LOGI(TAG, "Read config bytes: R=%02X", ret);
    ESP_RETURN_ON_ERROR(ret, TAG, "Error in READ");

    uint8_t count_en = (read_buf[4] >> 4) & 0x01;
    ESP_LOGI(TAG, "NFC_CNT_EN bit: %hhu", count_en);

    if (count_en) { // Avoid a NAK since then we need to wake the chip back up
        uint32_t counter = 0;
        // Counter argument is always 0x02 on NTAG21x; Ultralight PICCs with
        // counters usually have 3 and the argument selects which to read
        ret = rc522_nxp_read_cnt(scanner, picc, 0x02, &counter);
        ESP_LOGI(TAG, "Read counter: R=%02X", ret);
        ESP_RETURN_ON_ERROR(ret, TAG, "Error in READ_CNT");
        ESP_LOGI(TAG, "Counter value: %lu", counter);
    }
    else {
        ESP_LOGI(TAG, "Counter disabled; skipping");
    }
    return ESP_OK;
}

// Example: Reading the originality signature from the PICC
static esp_err_t example_get_signature(rc522_handle_t rc522, rc522_picc_t *picc)
{
    // Signatures can be either 32 or 48 bytes - here we default to 48 and let
    // the library fill as appropriate. The sig_size field is updated with the
    // correct size
    uint8_t sig_buf[48];
    rc522_nxp_sig_t sig = { .bytes = sig_buf, .buffer_size = sizeof(sig_buf) };

    esp_err_t ret = rc522_nxp_read_sig(scanner, picc, &sig);
    ESP_LOGI(TAG, "Read signature: R=%02X", ret);
    ESP_RETURN_ON_ERROR(ret, TAG, "Error in READ_SIG");

    buf_to_hex(sig_buf, sig.sig_size, str_buf, 128);
    ESP_LOGI(TAG, "Signature: %s", str_buf);
    return ESP_OK;
}

static void on_picc_state_changed(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    rc522_picc_state_changed_event_t *event = (rc522_picc_state_changed_event_t *)data;
    rc522_picc_t *picc = event->picc;

    if (picc->state != RC522_PICC_STATE_ACTIVE) {
        return;
    }

    if (picc->type != RC522_PICC_TYPE_MIFARE_UL) {
        return;
    }

    // get_type MUST be called before rc522_nxp_* functions to ensure the PICC
    // has been correctly identified
    rc522_nxp_get_type(scanner, picc, &picc->type);
    rc522_picc_print(picc);

    example_page_counts(scanner, picc);
    example_read_write(scanner, picc);
    example_fast_read(scanner, picc);
    example_pwd_auth(scanner, picc);
    example_read_cnt(scanner, picc);
    example_get_signature(scanner, picc);

    esp_err_t ret = dump_memory(scanner, picc);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Memory dump failed: E=%02X", ret);
        return;
    }

    ESP_LOGI(TAG, "Memory dump success");
}

void app_main()
{
    rc522_spi_create(&driver_config, &driver);
    rc522_driver_install(driver);

    rc522_config_t scanner_config = {
        .driver = driver,
    };

    rc522_create(&scanner_config, &scanner);
    rc522_register_events(scanner, RC522_EVENT_PICC_STATE_CHANGED, on_picc_state_changed, NULL);
    rc522_start(scanner);
}
