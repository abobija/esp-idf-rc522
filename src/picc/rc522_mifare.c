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
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(key == NULL);

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
    rc522_handle_t rc522, rc522_picc_t *picc, uint8_t block_addr, uint8_t buffer[RC522_MIFARE_BLOCK_SIZE])
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(buffer == NULL);

    // Build command buffer
    buffer[0] = RC522_MIFARE_READ_CMD;
    buffer[1] = block_addr;

    // Calculate CRC_A
    RC522_RETURN_ON_ERROR(rc522_pcd_calculate_crc(rc522, buffer, 2, &buffer[2]));

    // FIXME: Cloning the buffer since picc_transceive uses +2 bytes buffer
    //        to store some extra data (crc i think). Refactor picc_transceive
    //        and picc_comm functions!
    uint8_t buffer_clone[RC522_MIFARE_BLOCK_SIZE + 2];

    // Transmit the buffer and receive the response, validate CRC_A.
    uint8_t _ = sizeof(buffer_clone);
    RC522_RETURN_ON_ERROR(rc522_picc_transceive(rc522, buffer, 4, buffer_clone, &_, NULL, 0, true));

    // FIXME: Move data back to the original buffer
    memcpy(buffer, buffer_clone, RC522_MIFARE_BLOCK_SIZE);

    return ESP_OK;
}

static esp_err_t rc522_mifare_transceive(
    rc522_handle_t rc522, uint8_t *send_data, uint8_t send_length, bool accept_timeout)
{
    RC522_CHECK(send_data == NULL);
    RC522_CHECK(send_length > RC522_MIFARE_BLOCK_SIZE);

    uint8_t cmd_buffer[RC522_MIFARE_BLOCK_SIZE + 2]; // We need room for data + 2 bytes CRC_A

    // Copy sendData[] to cmdBuffer[] and add CRC_A
    memcpy(cmd_buffer, send_data, send_length);

    RC522_RETURN_ON_ERROR(rc522_pcd_calculate_crc(rc522, cmd_buffer, send_length, cmd_buffer + send_length));

    send_length += 2;

    // Transceive the data, store the reply in cmdBuffer[]
    uint8_t wait_irq = 0x30; // RxIRq and IdleIRq
    uint8_t cmd_buffer_size = sizeof(cmd_buffer);
    uint8_t valid_bits = 0;

    esp_err_t ret = rc522_picc_comm(rc522,
        RC522_PCD_TRANSCEIVE_CMD,
        wait_irq,
        cmd_buffer,
        send_length,
        cmd_buffer,
        &cmd_buffer_size,
        &valid_bits,
        0,
        false);

    if (accept_timeout && ret == ESP_ERR_TIMEOUT) {
        return ESP_OK;
    }

    if (ret != ESP_OK) {
        return ret;
    }

    // The PICC must reply with a 4 bit ACK
    if (cmd_buffer_size != 1 || valid_bits != 4) {
        return ESP_FAIL;
    }

    if (cmd_buffer[0] != RC522_MIFARE_ACK) {
        return RC522_ERR_MIFARE_NACK;
    }

    return ret;
}

esp_err_t rc522_mifare_write(
    rc522_handle_t rc522, rc522_picc_t *picc, uint8_t block_addr, uint8_t buffer[RC522_MIFARE_BLOCK_SIZE])
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(buffer == NULL);

    // Step 1: Tell the PICC we want to write to block blockAddr.
    uint8_t cmd_buffer[] = { RC522_MIFARE_WRITE_CMD, block_addr };
    RC522_RETURN_ON_ERROR(
        rc522_mifare_transceive(rc522, cmd_buffer, 2, false)); // Adds CRC_A and checks that the response is MF_ACK.

    RC522_RETURN_ON_ERROR(rc522_mifare_transceive(rc522,
        buffer,
        RC522_MIFARE_BLOCK_SIZE,
        false)); // Adds CRC_A and checks that the response is MF_ACK.

    return ESP_OK;
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
    RC522_CHECK(picc == NULL);
    RC522_CHECK(mifare == NULL);

    RC522_RETURN_ON_ERROR(rc522_mifare_number_of_sectors(picc->type, &mifare->number_of_sectors));

    return ESP_OK;
}

esp_err_t rc522_mifare_sector_info(uint8_t sector_index, rc522_mifare_sector_t *result)
{
    RC522_CHECK(sector_index > RC522_MIFARE_MAX_SECTOR_INDEX);
    RC522_CHECK(result == NULL);

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
 *
 * Resulting access bits:
 *
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
esp_err_t rc522_mifare_parse_access_bits(uint8_t *trailer_bytes, /** Sector trailer block data */
    uint8_t access_bits[4] /* Pointer to 4 bytes array */)
{
    RC522_CHECK(trailer_bytes == NULL);
    RC522_CHECK(access_bits == NULL);

    uint8_t c1 = trailer_bytes[7] >> 4;
    uint8_t c2 = trailer_bytes[8] & 0x0F;
    uint8_t c3 = trailer_bytes[8] >> 4;
    uint8_t c1_ = trailer_bytes[6] & 0x0F;
    uint8_t c2_ = trailer_bytes[6] >> 4;
    uint8_t c3_ = trailer_bytes[7] & 0x0F;

    if ((c1 != (~c1_ & 0xF)) || (c2 != (~c2_ & 0xF)) || (c3 != (~c3_ & 0xF))) {
        return RC522_ERR_MIFARE_ACCESS_BITS_INTEGRITY_VIOLATION;
    }

    access_bits[0] = ((c1 & 1) << 2) | ((c2 & 1) << 1) | ((c3 & 1) << 0);
    access_bits[1] = ((c1 & 2) << 1) | ((c2 & 2) << 0) | ((c3 & 2) >> 1);
    access_bits[2] = ((c1 & 4) << 0) | ((c2 & 4) >> 1) | ((c3 & 4) >> 2);
    access_bits[3] = ((c1 & 8) >> 1) | ((c2 & 8) >> 2) | ((c3 & 8) >> 3);

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
    RC522_CHECK(bytes == NULL);
    RC522_CHECK(block == NULL);

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
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);

    if (rc522_picc_halta(rc522, picc) != ESP_OK) {
        RC522_LOGW("halta failed");
    }

    return rc522_pcd_stop_crypto1(rc522);
}

/**
 * Ensures that communication with PICC is ended correctly by calling rc522_mifare_transactions_end
 */
esp_err_t rc522_mifare_handle_as_transaction(
    rc522_handle_t rc522, rc522_picc_t *picc, rc522_mifare_transaction_handler transaction_handler)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(transaction_handler == NULL);

    esp_err_t ret = transaction_handler(rc522, picc);

    if (rc522_mifare_transactions_end(rc522, picc) != ESP_OK) {
        RC522_LOGW("ending transaction failed");
    }

    return ret;
}

esp_err_t rc522_mifare_iterate_sector_blocks(rc522_handle_t rc522, rc522_picc_t *picc, uint8_t sector_index,
    rc522_mifare_key_t *key, rc522_mifare_sector_block_iterator iterator)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(sector_index > RC522_MIFARE_MAX_SECTOR_INDEX);
    RC522_CHECK(key == NULL);
    RC522_CHECK(iterator == NULL);

    esp_err_t ret = ESP_OK;

    rc522_mifare_sector_t sector;
    ESP_RETURN_ON_ERROR(rc522_mifare_sector_info(sector_index, &sector), TAG, "");

    // Establish encrypted communications before reading the first block
    ESP_RETURN_ON_ERROR(rc522_mifare_autha(rc522, picc, sector.block_0_address, key), TAG, "auth failed");

    int8_t trailer_block_offset = sector.number_of_blocks - 1;
    uint8_t access_bits[4];

    for (int8_t block_offset = trailer_block_offset; block_offset >= 0; block_offset--) {
        rc522_mifare_sector_block_t block;
        memset(&block, 0x00, sizeof(rc522_mifare_sector_block_t));

        block.sector_index = sector_index;
        block.address = (sector.block_0_address + block_offset);

        if (block_offset == trailer_block_offset) {
            block.type = RC522_MIFARE_BLOCK_TRAILER;
        }

        ESP_RETURN_ON_ERROR(rc522_mifare_read(rc522, picc, block.address, block.bytes), TAG, "read failed");

        if (block.type == RC522_MIFARE_BLOCK_TRAILER) {
            block.access_bits_err = rc522_mifare_parse_access_bits(block.bytes, access_bits);
        }

        uint8_t group = block_offset;
        bool is_first_in_group = true;

        if (sector.number_of_blocks != 4) {
            group = block_offset / 5;
            is_first_in_group = (group == 3) || (group != (block_offset + 1) / 5);
        }

        if (is_first_in_group) {
            block.access_bits = access_bits[group];
        }

        if (block.type != RC522_MIFARE_BLOCK_TRAILER) {
            block.type =
                rc522_mifare_block_is_value(block.access_bits) ? RC522_MIFARE_BLOCK_VALUE : RC522_MIFARE_BLOCK_DATA;
        }

        if (block.type == RC522_MIFARE_BLOCK_VALUE) {
            rc522_mifare_value_block_t value;
            block.value_err = rc522_mifare_parse_value_block(block.bytes, &value);
            block.value = &value;
        }

        if ((ret = iterator(&block)) != ESP_OK) {
            break;
        }
    }

    return ret;
}
