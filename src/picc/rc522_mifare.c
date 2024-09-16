#include <string.h>
#include "rc522_types_private.h"
#include "rc522_private.h"
#include "rc522_pcd_private.h"
#include "rc522_picc_private.h"
#include "picc/rc522_mifare.h"

RC522_LOG_DEFINE_BASE();

/**
 * The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
 * Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on
 * the sector.
 */
typedef enum
{
    /**
     * Perform authentication with Key A
     */
    RC522_MIFARE_AUTH_KEY_A_CMD = 0x60,

    /**
     * Perform authentication with Key B
     */
    RC522_MIFARE_AUTH_KEY_B_CMD = 0x61,

    /**
     * Reads one 16 byte block from the authenticated sector of the PICC
     */
    RC522_MIFARE_READ_CMD = 0x30,

    /**
     * Writes one 16 byte block to the authenticated sector of the PICC
     */
    RC522_MIFARE_WRITE_CMD = 0xA0,

    /**
     * Decrements the contents of a block and stores the result in the internal data register
     */
    RC522_MIFARE_DECREMENT_CMD = 0xC0,

    /**
     * Increments the contents of a block and stores the result in the internal data register
     */
    RC522_MIFARE_INCREMENT_CMD = 0xC1,

    /**
     * Reads the contents of a block into the internal data register
     */
    RC522_MIFARE_RESTORE_CMD = 0xC2,

    /**
     * Writes the contents of the internal data register to a block
     */
    RC522_MIFARE_TRANSFER_CMD = 0xB0,
} rc522_mifare_command_t;

/**
 * Checks if PICC type is compatible with MIFARE Classic protocol
 */
bool rc522_mifare_type_is_classic_compatible(rc522_picc_type_t type)
{
    return type == RC522_PICC_TYPE_MIFARE_MINI || type == RC522_PICC_TYPE_MIFARE_1K
           || type == RC522_PICC_TYPE_MIFARE_4K;
}

inline esp_err_t rc522_mifare_autha(
    rc522_handle_t rc522, rc522_picc_t *picc, uint8_t block_addr, rc522_mifare_key_t *key)
{
    return rc522_mifare_auth(rc522, picc, RC522_MIFARE_KEY_A, block_addr, key);
}

inline esp_err_t rc522_mifare_authb(
    rc522_handle_t rc522, rc522_picc_t *picc, uint8_t block_addr, rc522_mifare_key_t *key)
{
    return rc522_mifare_auth(rc522, picc, RC522_MIFARE_KEY_B, block_addr, key);
}

esp_err_t rc522_mifare_auth(rc522_handle_t rc522, rc522_picc_t *picc, rc522_mifare_key_type_t key_type,
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

    // Use the last uid bytes
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

esp_err_t rc522_mifare_read(
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

esp_err_t rc522_mifare_info(rc522_picc_t *picc, rc522_mifare_t *mifare)
{
    RC522_RETURN_ON_ERROR(rc522_mifare_number_of_sectors(picc->type, &mifare->number_of_sectors));

    return ESP_OK;
}

esp_err_t rc522_mifare_sector_info(uint8_t sector_index, rc522_mifare_sector_t *result)
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
esp_err_t rc522_mifare_parse_sector_trailer(uint8_t *bytes, /** Sector trailer block data */
    rc522_mifare_sector_trailer_t *trailer)
{
    uint8_t c1 = bytes[7] >> 4;
    uint8_t c2 = bytes[8] & 0x0F;
    uint8_t c3 = bytes[8] >> 4;
    uint8_t c1_ = bytes[6] & 0x0F;
    uint8_t c2_ = bytes[6] >> 4;
    uint8_t c3_ = bytes[7] & 0x0F;

    if ((c1 != (~c1_ & 0xF)) || (c2 != (~c2_ & 0xF)) || (c3 != (~c3_ & 0xF))) {
        return RC522_ERR_MIFARE_ACCESS_BITS_INTEGRITY_VIOLATION;
    }

    trailer->access_bits[0] = ((c1 & 1) << 2) | ((c2 & 1) << 1) | ((c3 & 1) << 0);
    trailer->access_bits[1] = ((c1 & 2) << 1) | ((c2 & 2) << 0) | ((c3 & 2) >> 1);
    trailer->access_bits[2] = ((c1 & 4) << 0) | ((c2 & 4) >> 1) | ((c3 & 4) >> 2);
    trailer->access_bits[3] = ((c1 & 8) >> 1) | ((c2 & 8) >> 2) | ((c3 & 8) >> 3);

    return ESP_OK;
}

/**
 * Checks if block is Value block based on access bits
 */
inline bool rc522_mifare_block_is_value(uint8_t access_bits)
{
    return access_bits == 0b110 || access_bits == 0b001;
}

esp_err_t rc522_mifare_parse_value_block(uint8_t *bytes, /** Value block data */ rc522_mifare_value_block_t *block)
{
    block->value = 0;

    block->value |= ((int32_t)(bytes[3]) << (8 * 3));
    block->value |= ((int32_t)(bytes[2]) << (8 * 2));
    block->value |= ((int32_t)(bytes[1]) << (8 * 1));
    block->value |= ((int32_t)(bytes[0]) << (8 * 0));

    block->address = bytes[12];

    // TODO: Check for integrity violation
    //       Use RC522_ERR_MIFARE_VALUE_BLOCK_INTEGRITY_VIOLATION

    return ESP_OK;
}

esp_err_t rc522_mifare_transactions_end(rc522_handle_t rc522, rc522_picc_t *picc)
{
    if (rc522_picc_halta(rc522, picc) != ESP_OK) {
        RC522_LOGW("halta failed");
    }

    return rc522_pcd_stop_crypto1(rc522);
}
