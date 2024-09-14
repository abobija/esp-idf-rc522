#include <string.h>
#include "rc522_types_private.h"
#include "rc522_pcd_private.h"
#include "rc522_picc_private.h"
#include "picc/rc522_mifare.h"

RC522_LOG_DEFINE_BASE();

#define COLUMN_SECTOR_WIDTH (6)
#define COLUMN_BLOCK_WIDTH  (7)

static esp_err_t rc522_mifare_autha(
    rc522_handle_t rc522, rc522_picc_t *picc, uint8_t block_addr, uint8_t *key, uint8_t key_length)
{
    uint8_t wait_irq = 0x10; // IdleIRq

    // Build command buffer
    uint8_t send_data[12];
    send_data[0] = RC522_PICC_CMD_MF_AUTH_KEY_A;
    send_data[1] = block_addr;
    for (uint8_t i = 0; i < key_length; i++) {
        send_data[2 + i] = key[i];
    }
    // Use the last uid bytes as specified in http://cache.nxp.com/documents/application_note/AN10927.pdf
    // section 3.2.5 "MIFARE Classic Authentication".
    // The only missed case is the MF1Sxxxx shortcut activation,
    // but it requires cascade tag (CT) byte, that is not part of uid.
    for (uint8_t i = 0; i < 4; i++) { // The last 4 bytes of the UID
        send_data[8 + i] = picc->uid.bytes[i + picc->uid.bytes_length - 4];
    }

    // Start the authentication.
    return rc522_picc_comm(rc522,
        RC522_PCD_MF_AUTH_CMD,
        wait_irq,
        send_data,
        sizeof(send_data),
        NULL,
        NULL,
        NULL,
        0,
        false);
}

static esp_err_t rc522_mifare_read(
    rc522_handle_t rc522, rc522_picc_t *picc, uint8_t block_addr, uint8_t *buffer, uint8_t *buffer_length)
{
    ESP_RETURN_ON_FALSE(buffer != NULL, ESP_ERR_INVALID_ARG, TAG, "buffer is null");
    ESP_RETURN_ON_FALSE(*buffer_length >= 18, ESP_ERR_NO_MEM, TAG, "buffer too small");

    // Build command buffer
    buffer[0] = RC522_PICC_CMD_MF_READ;
    buffer[1] = block_addr;

    // Calculate CRC_A
    RC522_RETURN_ON_ERROR(rc522_pcd_calculate_crc(rc522, buffer, 2, &buffer[2]));

    // Transmit the buffer and receive the response, validate CRC_A.
    return rc522_picc_transceive(rc522, buffer, 4, buffer, buffer_length, NULL, 0, true);
}

inline static void rc522_mifare_dump_memory_header_to_log()
{
    RC522_LOG_WRITE("%*s%*s  0 1 2 3  4 5 6 7  8 ... 11 12 .. 15  AccessBits\n",
        COLUMN_SECTOR_WIDTH,
        "Sector",
        COLUMN_BLOCK_WIDTH,
        "Block");
}

static esp_err_t rc522_mifare_dump_sector_to_log(
    rc522_handle_t rc522, rc522_picc_t *picc, uint8_t *key, uint8_t key_length, uint8_t sector)
{
    uint8_t first_block;  // Address of lowest address to dump actually last block dumped)
    uint8_t no_of_blocks; // Number of blocks in sector

    // The access bits are stored in a peculiar fashion.
    // There are four groups:
    //		g[3]	Access bits for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
    //		g[2]	Access bits for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
    //		g[1]	Access bits for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
    //		g[0]	Access bits for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
    // Each group has access bits [C1 C2 C3]. In this code C1 is MSB and C3 is LSB.
    // The four CX bits are stored together in a nible cx and an inverted nible cx_.
    uint8_t c1, c2, c3;    // Nibbles
    uint8_t c1_, c2_, c3_; // Inverted nibbles
    bool inverted_error;   // True if one of the inverted nibbles did not match
    uint8_t g[4];          // Access bits for each of the four groups.
    uint8_t group;         // 0-3 - active group for access bits
    bool first_in_group;   // True for the first block dumped in the group

    // Determine position and size of sector.
    if (sector < 32) { // Sectors 0..31 has 4 blocks each
        no_of_blocks = 4;
        first_block = sector * no_of_blocks;
    }
    else if (sector < 40) { // Sectors 32-39 has 16 blocks each
        no_of_blocks = 16;
        first_block = 128 + (sector - 32) * no_of_blocks;
    }
    else {
        ESP_LOGE(TAG, "Illegal input, no MIFARE Classic PICC has more than 40 sectors");
        return ESP_ERR_INVALID_STATE;
    }

    // Dump blocks, highest address first.
    uint8_t byte_count;
    uint8_t buffer[18];
    memset(buffer, 0x00, sizeof(buffer));

    uint8_t block_addr;
    bool is_sector_trailer = true;
    inverted_error = false; // Avoid "unused variable" warning.

    // Establish encrypted communications before reading the first block
    RC522_RETURN_ON_ERROR(rc522_mifare_autha(rc522, picc, first_block, key, key_length));

    for (int8_t blockOffset = no_of_blocks - 1; blockOffset >= 0; blockOffset--) {
        block_addr = first_block + blockOffset;

        // Sector number - only on first line
        if (is_sector_trailer) {
            RC522_LOG_WRITE("%*d", COLUMN_SECTOR_WIDTH, sector);
        }
        else {
            RC522_LOG_WRITE("%*s", COLUMN_SECTOR_WIDTH, " ");
        }

        RC522_LOG_WRITE("%*d", COLUMN_BLOCK_WIDTH, block_addr % 4);
        RC522_LOG_WRITE("  ");

        // Read block
        byte_count = sizeof(buffer);
        RC522_RETURN_ON_ERROR(rc522_mifare_read(rc522, picc, block_addr, buffer, &byte_count));

        // TODO: what if byte_count != 16 ???

        // Dump data
        for (uint8_t index = 0; index < 16; index++) {
            RC522_LOG_WRITE("%02x", buffer[index]);

            if ((index % 4) == 3) {
                RC522_LOG_WRITE(" ");
            }
        }

        // Parse sector trailer data
        if (is_sector_trailer) {
            c1 = buffer[7] >> 4;
            c2 = buffer[8] & 0xF;
            c3 = buffer[8] >> 4;
            c1_ = buffer[6] & 0xF;
            c2_ = buffer[6] >> 4;
            c3_ = buffer[7] & 0xF;
            inverted_error = (c1 != (~c1_ & 0xF)) || (c2 != (~c2_ & 0xF)) || (c3 != (~c3_ & 0xF));
            g[0] = ((c1 & 1) << 2) | ((c2 & 1) << 1) | ((c3 & 1) << 0);
            g[1] = ((c1 & 2) << 1) | ((c2 & 2) << 0) | ((c3 & 2) >> 1);
            g[2] = ((c1 & 4) << 0) | ((c2 & 4) >> 1) | ((c3 & 4) >> 2);
            g[3] = ((c1 & 8) >> 1) | ((c2 & 8) >> 2) | ((c3 & 8) >> 3);
        }

        // Which access group is this block in?
        if (no_of_blocks == 4) {
            group = blockOffset;
            first_in_group = true;
        }
        else {
            group = blockOffset / 5;
            first_in_group = (group == 3) || (group != (blockOffset + 1) / 5);
        }

        if (first_in_group) {
            // Print access bits
            RC522_LOG_WRITE(" [ ");
            RC522_LOG_WRITE("%d", (g[group] >> 2) & 1);
            RC522_LOG_WRITE(" ");
            RC522_LOG_WRITE("%d", (g[group] >> 1) & 1);
            RC522_LOG_WRITE(" ");
            RC522_LOG_WRITE("%d", (g[group] >> 0) & 1);
            RC522_LOG_WRITE(" ] ");
            if (inverted_error) {
                RC522_LOG_WRITE(" Inverted access bits did not match! ");
            }
        }

        if (group != 3 && (g[group] == 1 || g[group] == 6)) { // Not a sector trailer, a value block
            int32_t value = ((int32_t)(buffer[3]) << 24) | ((int32_t)(buffer[2]) << 16) | ((int32_t)(buffer[1]) << 8) |
                            (int32_t)(buffer[0]);
            RC522_LOG_WRITE(" Value=0x%02lx", value);
            RC522_LOG_WRITE(" Adr=0x%02x", buffer[12]);
        }

        RC522_LOG_WRITE("\n");

        is_sector_trailer = false;
    }

    return ESP_OK;
}

esp_err_t rc522_mifare_dump_data_to_log(rc522_handle_t rc522, rc522_picc_t *picc, uint8_t *key, uint8_t key_length)
{
    uint8_t sectors_length = 0;

    switch (picc->type) {
        case RC522_PICC_TYPE_MIFARE_MINI:
            // Has 5 sectors * 4 blocks/sector * 16 bytes/block = 320 bytes.
            sectors_length = 5;
            break;

        case RC522_PICC_TYPE_MIFARE_1K:
            // Has 16 sectors * 4 blocks/sector * 16 bytes/block = 1024 bytes.
            sectors_length = 16;
            break;

        case RC522_PICC_TYPE_MIFARE_4K:
            // Has (32 sectors * 4 blocks/sector + 8 sectors * 16 blocks/sector) * 16 bytes/block = 4096 bytes.
            sectors_length = 40;
            break;

        default: // Should not happen. Ignore.
            break;
    }

    esp_err_t ret = ESP_OK;

    // Dump sectors, highest address first.
    if (sectors_length) {
        rc522_mifare_dump_memory_header_to_log();

        for (int8_t i = sectors_length - 1; i >= 0; i--) {
            ret = rc522_mifare_dump_sector_to_log(rc522, picc, key, key_length, i);

            if (ret != ESP_OK) {
                break;
            }
        }
    }

    if (ret == ESP_OK) {
        RC522_RETURN_ON_ERROR(rc522_picc_halta(rc522, picc));
    }

    RC522_RETURN_ON_ERROR(rc522_pcd_stop_crypto1(rc522));

    return ret;
}
