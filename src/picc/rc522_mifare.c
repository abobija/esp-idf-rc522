#include <string.h>
#include "rc522_types_private.h"
#include "rc522_pcd_private.h"
#include "rc522_picc_private.h"
#include "picc/rc522_mifare.h"

RC522_LOG_DEFINE_BASE();

#define COLUMN_SECTOR_WIDTH (6)
#define COLUMN_BLOCK_WIDTH  (7)

typedef struct
{
    uint8_t index;            // Zero-based index of Sector
    uint8_t number_of_blocks; // Total number of blocks inside of Sector
    uint8_t block_0_address;  // Zero-based index of the first Block inside of MIFARE memory
} rc522_mifare_sector_info_t;

typedef struct
{
    /**
     * [3] Access bits for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
     * [2] Access bits for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
     * [1] Access bits for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
     * [0] Access bits for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
     *
     * Bits of each element are in next format:
     *
     * | 7 | 6 | 5 | 4 | 3 |  2 |  1 |  0 |
     * | - | - | - | - | - | -- | -- | -- |
     * | 0 | 0 | 0 | 0 | 0 | C1 | C2 | C3 |
     *
     */
    uint8_t access_bits[4];
} rc522_mifare_sector_trailer_t;

/**
 * Checks if PICC type is compatible with MIFARE Classic protocol
 */
bool rc522_mifare_type_is_classic_compatible(rc522_picc_type_t type)
{
    return type == RC522_PICC_TYPE_MIFARE_MINI || type == RC522_PICC_TYPE_MIFARE_1K
           || type == RC522_PICC_TYPE_MIFARE_4K;
}

static esp_err_t rc522_mifare_auth(rc522_handle_t rc522, rc522_picc_t *picc, rc522_mifare_key_type_t key_type,
    uint8_t block_addr, rc522_mifare_key_t *key)
{
    uint8_t auth_cmd;

    switch (key_type) {
        case RC522_MIFARE_KEY_A:
            auth_cmd = RC522_MIFARE_AUTH_KEY_A_CMD;
            break;
        case RC522_MIFARE_KEY_B:
            auth_cmd = RC522_MIFARE_AUTH_KEY_B_CMD;
            break;
        default:
            RC522_LOGE("Invalid key type");
            return ESP_ERR_INVALID_ARG;
    }

    uint8_t wait_irq = 0x10; // IdleIRq

    // Build command buffer
    uint8_t send_data[12];
    send_data[0] = auth_cmd;
    send_data[1] = block_addr;
    memcpy(send_data + 2, key->value, RC522_MIFARE_KEY_SIZE);

    // Use the last uid bytes as specified in http://cache.nxp.com/documents/application_note/AN10927.pdf
    // section 3.2.5 "MIFARE Classic Authentication".
    // The only missed case is the MF1Sxxxx shortcut activation,
    // but it requires cascade tag (CT) byte, that is not part of uid.
    memcpy(send_data + 8, picc->uid.value + picc->uid.length - 4, 4);

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
    buffer[0] = RC522_MIFARE_READ_CMD;
    buffer[1] = block_addr;

    // Calculate CRC_A
    RC522_RETURN_ON_ERROR(rc522_pcd_calculate_crc(rc522, buffer, 2, &buffer[2]));

    // Transmit the buffer and receive the response, validate CRC_A.
    return rc522_picc_transceive(rc522, buffer, 4, buffer, buffer_length, NULL, 0, true);
}

static esp_err_t rc522_mifare_number_of_sectors(rc522_picc_type_t type, uint8_t *result)
{
    ESP_RETURN_ON_FALSE(rc522_mifare_type_is_classic_compatible(type), ESP_ERR_INVALID_ARG, TAG, "invalid type");

    switch (type) {
        case RC522_PICC_TYPE_MIFARE_MINI:
            // Has 5 sectors * 4 blocks/sector * 16 bytes/block = 320 bytes.
            *result = 5;
            return ESP_OK;
        case RC522_PICC_TYPE_MIFARE_1K:
            // Has 16 sectors * 4 blocks/sector * 16 bytes/block = 1024 bytes.
            *result = 16;
            return ESP_OK;
        case RC522_PICC_TYPE_MIFARE_4K:
            // Has (32 sectors * 4 blocks/sector + 8 sectors * 16 blocks/sector) * 16 bytes/block = 4096 bytes.
            *result = 40;
            return ESP_OK;
        default:
            return ESP_FAIL;
    }
}

static esp_err_t rc522_mifare_sector_info(uint8_t sector_index, rc522_mifare_sector_info_t *result)
{
    // No MIFARE Classic has more than 40 sectors
    ESP_RETURN_ON_FALSE(sector_index < 40, ESP_ERR_INVALID_ARG, TAG, "invalid sector_index");

    result->index = sector_index;

    // Determine position and size of sector.
    if (result->index < 32) { // Sectors 0..31 has 4 blocks each
        result->number_of_blocks = 4;
        result->block_0_address = result->index * result->number_of_blocks;

        return ESP_OK;
    }

    if (result->index < 40) { // Sectors 32-39 has 16 blocks each
        result->number_of_blocks = 16;
        result->block_0_address = 128 + (result->index - 32) * result->number_of_blocks;

        return ESP_OK;
    }

    return ESP_FAIL;
}

/**
 * Bytes 6, 7 and 8 of sector trailer block are bytes that holds access bits.
 * Next table shows bit descriptions for each of those bytes.
 *
 * |      |   7  |   6  |   5  |   4  |   3  |   2  |   1  |   0  |
 * | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
 * | [6]  | C23_ | C22_ | C21_ | C20_ | C13_ | C12_ | C11_ | C10_ |
 * | [7]  | C13  | C12  | C11  | C10  | C33_ | C32_ | C31_ | C30_ |
 * | [8]  | C33  | C32  | C31  | C30  | C23  | C22  | C21  | C20  |
 *
 */
static esp_err_t rc522_mifare_parse_sector_trailer(uint8_t *block, /** Sector trailer block data */
    rc522_mifare_sector_trailer_t *trailer)
{
    uint8_t c1 = block[7] >> 4;
    uint8_t c2 = block[8] & 0x0F;
    uint8_t c3 = block[8] >> 4;
    uint8_t c1_ = block[6] & 0x0F;
    uint8_t c2_ = block[6] >> 4;
    uint8_t c3_ = block[7] & 0x0F;

    if ((c1 != (~c1_ & 0xF)) || (c2 != (~c2_ & 0xF)) || (c3 != (~c3_ & 0xF))) {
        return RC522_ERR_MIFARE_ACCESS_BITS_INTEGRITY_VIOLATION;
    }

    trailer->access_bits[0] = ((c1 & 1) << 2) | ((c2 & 1) << 1) | ((c3 & 1) << 0);
    trailer->access_bits[1] = ((c1 & 2) << 1) | ((c2 & 2) << 0) | ((c3 & 2) >> 1);
    trailer->access_bits[2] = ((c1 & 4) << 0) | ((c2 & 4) >> 1) | ((c3 & 4) >> 2);
    trailer->access_bits[3] = ((c1 & 8) >> 1) | ((c2 & 8) >> 2) | ((c3 & 8) >> 3);

    return ESP_OK;
}

inline static void rc522_mifare_dump_memory_header_to_log()
{
    RC522_LOG_WRITE("%*s%*s                 Bytes                 AccessBits\n",
        COLUMN_SECTOR_WIDTH,
        "Sector",
        COLUMN_BLOCK_WIDTH,
        "Block");

    RC522_LOG_WRITE("%*s%*s   0 1 2 3  4 5 6 7  8 9  11 12    15    c1 c2 c3\n",
        COLUMN_SECTOR_WIDTH,
        " ",
        COLUMN_BLOCK_WIDTH,
        " ");
}

static esp_err_t rc522_mifare_dump_sector_to_log(
    rc522_handle_t rc522, rc522_picc_t *picc, rc522_mifare_key_t *key, uint8_t sector_index)
{
    esp_err_t ret = ESP_OK;

    rc522_mifare_sector_info_t sector;
    RC522_RETURN_ON_ERROR(rc522_mifare_sector_info(sector_index, &sector));

    // Establish encrypted communications before reading the first block
    RC522_RETURN_ON_ERROR(rc522_mifare_auth(rc522, picc, RC522_MIFARE_KEY_A, sector.block_0_address, key));

    rc522_mifare_sector_trailer_t trailer;
    bool is_trailer = true;

    for (int8_t block_offset = sector.number_of_blocks - 1; block_offset >= 0; block_offset--) {
        uint8_t block_addr = sector.block_0_address + block_offset;

        // Sector number - only on first line
        if (is_trailer) {
            RC522_LOG_WRITE("%*d", COLUMN_SECTOR_WIDTH, sector_index);
        }
        else {
            RC522_LOG_WRITE("%*s", COLUMN_SECTOR_WIDTH, " ");
        }

        RC522_LOG_WRITE("%*d", COLUMN_BLOCK_WIDTH, block_addr);
        RC522_LOG_WRITE("  ");

        // Read block
        uint8_t buffer[18];
        uint8_t byte_count = sizeof(buffer);
        memset(buffer, 0x00, byte_count);
        RC522_RETURN_ON_ERROR(rc522_mifare_read(rc522, picc, block_addr, buffer, &byte_count));

        // Dump data
        for (uint8_t index = 0; index < 16; index++) {
            RC522_LOG_WRITE("%02x", buffer[index]);

            if ((index % 4) == 3) {
                RC522_LOG_WRITE(" ");
            }
        }

        RC522_LOG_WRITE(" ");

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
            RC522_LOG_WRITE("   %d  %d  %d",
                (trailer.access_bits[group] >> 2) & 1,
                (trailer.access_bits[group] >> 1) & 1,
                (trailer.access_bits[group] >> 0) & 1);

            if (ret == RC522_ERR_MIFARE_ACCESS_BITS_INTEGRITY_VIOLATION) {
                RC522_LOG_WRITE(" (Access bits integrity violation!) ");
            }
        }

        if (group != 3
            && (trailer.access_bits[group] == 1
                || trailer.access_bits[group] == 6)) { // Not a sector trailer, a value block
            int32_t value = ((int32_t)(buffer[3]) << 24) | ((int32_t)(buffer[2]) << 16) | ((int32_t)(buffer[1]) << 8)
                            | (int32_t)(buffer[0]);

            RC522_LOG_WRITE(" Value=0x%02lx, Adr=0x%02x", value, buffer[12]);

            // TODO: We can check here for integrity violation of value block
        }

        RC522_LOG_WRITE("\n");

        is_trailer = false;
    }

    return ret;
}

esp_err_t rc522_mifare_dump(rc522_handle_t rc522, rc522_picc_t *picc, rc522_mifare_key_t *key)
{
    ESP_RETURN_ON_FALSE(rc522_mifare_type_is_classic_compatible(picc->type),
        ESP_ERR_INVALID_ARG,
        TAG,
        "invalid picc type");

    uint8_t number_of_sectors;
    RC522_RETURN_ON_ERROR(rc522_mifare_number_of_sectors(picc->type, &number_of_sectors));

    rc522_mifare_dump_memory_header_to_log();

    // Dump sectors, highest address first.
    esp_err_t ret = ESP_OK;
    for (int8_t i = number_of_sectors - 1; i >= 0; i--) {
        ret = rc522_mifare_dump_sector_to_log(rc522, picc, key, i);

        if (ret != ESP_OK) {
            break;
        }
    }

    if (ret == ESP_OK) {
        RC522_RETURN_ON_ERROR(rc522_picc_halta(rc522, picc));
    }

    RC522_RETURN_ON_ERROR(rc522_pcd_stop_crypto1(rc522));

    return ret;
}
