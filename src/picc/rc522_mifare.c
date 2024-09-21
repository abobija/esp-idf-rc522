#include <string.h>
#include "rc522_types_private.h"
#include "rc522_private.h"
#include "rc522_pcd_private.h"
#include "rc522_picc_private.h"
#include "picc/rc522_mifare.h"

RC522_LOG_DEFINE_BASE();

/**
 * The commands used for MIFARE Classic
 */
enum
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
};

/**
 * Checks if PICC type is compatible with MIFARE Classic protocol
 */
bool rc522_mifare_type_is_classic_compatible(rc522_picc_type_t type)
{
    return type == RC522_PICC_TYPE_MIFARE_MINI || type == RC522_PICC_TYPE_MIFARE_1K
           || type == RC522_PICC_TYPE_MIFARE_4K;
}

inline esp_err_t rc522_mifare_auth_sector(
    rc522_handle_t rc522, rc522_picc_t *picc, rc522_mifare_sector_desc_t *sector_desc, rc522_mifare_key_t *key)
{
    RC522_CHECK(sector_desc == NULL);

    RC522_RETURN_ON_ERROR(rc522_mifare_auth(rc522, picc, sector_desc->block_0_address, key));

    return ESP_OK;
}

esp_err_t rc522_mifare_auth(rc522_handle_t rc522, rc522_picc_t *picc, uint8_t block_addr, rc522_mifare_key_t *key)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(key == NULL);

    // TODO: Validate block_addr

    RC522_LOGD("MIFARE AUTH (block_addr=%02" RC522_X ")", block_addr);

    uint8_t auth_cmd;

    switch (key->type) {
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
        RC522_PCD_IDLE_IRQ_BIT,
        send_data,
        sizeof(send_data),
        NULL,
        NULL,
        NULL,
        0,
        false);
}

esp_err_t rc522_mifare_read(
    rc522_handle_t rc522, rc522_picc_t *picc, uint8_t block_addr, uint8_t *buffer, uint8_t buffer_size)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(buffer == NULL);
    RC522_CHECK(buffer_size < RC522_MIFARE_BLOCK_SIZE);

    RC522_LOGD("MIFARE READ (block_addr=%02" RC522_X ")", block_addr);

    // FIXME: Cloning the buffer since picc_transceive uses +2 bytes buffer
    //        to store some extra data (crc i think). Refactor picc_transceive
    //        and picc_comm functions!
    uint8_t buffer_clone[RC522_MIFARE_BLOCK_SIZE + 2];

    // Build command buffer
    buffer_clone[0] = RC522_MIFARE_READ_CMD;
    buffer_clone[1] = block_addr;

    // Calculate CRC_A
    RC522_RETURN_ON_ERROR(rc522_pcd_calculate_crc(rc522, buffer_clone, 2, &buffer_clone[2]));

    // Transmit the buffer and receive the response, validate CRC_A.
    uint8_t _ = sizeof(buffer_clone);
    RC522_RETURN_ON_ERROR(rc522_picc_transceive(rc522, buffer_clone, 4, buffer_clone, &_, NULL, 0, true));

    RC522_CHECK(_ != (RC522_MIFARE_BLOCK_SIZE + 2));

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
    uint8_t cmd_buffer_size = sizeof(cmd_buffer);
    uint8_t valid_bits = 0;

    esp_err_t ret = rc522_picc_comm(rc522,
        RC522_PCD_TRANSCEIVE_CMD,
        RC522_PCD_RX_IRQ_BIT | RC522_PCD_IDLE_IRQ_BIT,
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
        return ESP_FAIL; // TODO: use custom err
    }

    if (cmd_buffer[0] != RC522_MIFARE_ACK) {
        return RC522_ERR_MIFARE_NACK;
    }

    return ret;
}

static esp_err_t rc522_mifare_get_number_of_sectors(rc522_picc_type_t type, uint8_t *out_result)
{
    RC522_CHECK(rc522_mifare_type_is_classic_compatible(type) == false);
    RC522_CHECK(out_result == NULL);

    switch (type) {
        case RC522_PICC_TYPE_MIFARE_MINI:
            // Has 5 sectors * 4 blocks/sector * 16 bytes/block = 320 bytes.
            *out_result = 5;
            return ESP_OK;
        case RC522_PICC_TYPE_MIFARE_1K:
            // Has 16 sectors * 4 blocks/sector * 16 bytes/block = 1024 bytes.
            *out_result = 16;
            return ESP_OK;
        case RC522_PICC_TYPE_MIFARE_4K:
            // Has (32 sectors * 4 blocks/sector + 8 sectors * 16 blocks/sector) * 16 bytes/block = 4096 bytes.
            *out_result = 40;
            return ESP_OK;
        default:
            return ESP_FAIL;
    }
}

inline static uint8_t rc522_mifare_get_sector_index_by_block_address(uint8_t block_address)
{
    if (block_address < 128) {
        return block_address / 4;
    }
    else {
        return 32 + ((block_address - 128) / 16);
    }
}

inline static esp_err_t rc522_mifare_get_sector_number_of_blocks(uint8_t sector_index, uint8_t *out_result)
{
    RC522_CHECK(sector_index > RC522_MIFARE_SECTOR_INDEX_MAX);
    RC522_CHECK(out_result == NULL);

    if (sector_index < 32) {
        *out_result = 4;
        return ESP_OK;
    }

    *out_result = 16;

    return ESP_OK;
}

inline static esp_err_t rc522_mifare_get_sector_block_0_address(uint8_t sector_index, uint8_t *out_result)
{
    RC522_CHECK(sector_index > RC522_MIFARE_SECTOR_INDEX_MAX);
    RC522_CHECK(out_result == NULL);

    if (sector_index < 32) {
        *out_result = sector_index * 4;
        return ESP_OK;
    }

    *out_result = 128 + (sector_index - 32) * 16;

    return ESP_OK;
}

esp_err_t rc522_mifare_get_sector_desc(uint8_t sector_index, rc522_mifare_sector_desc_t *out_sector_desc)
{
    RC522_CHECK(sector_index > RC522_MIFARE_SECTOR_INDEX_MAX);
    RC522_CHECK(out_sector_desc == NULL);

    rc522_mifare_sector_desc_t desc;
    memset(&desc, 0, sizeof(desc));

    desc.index = sector_index;
    RC522_RETURN_ON_ERROR(rc522_mifare_get_sector_number_of_blocks(sector_index, &desc.number_of_blocks));
    RC522_RETURN_ON_ERROR(rc522_mifare_get_sector_block_0_address(sector_index, &desc.block_0_address));

    memcpy(out_sector_desc, &desc, sizeof(rc522_mifare_sector_desc_t));

    return ESP_OK;
}

/**
 * Returns ESP_OK if writing to Sector Trailer block is allowed by the configuration
 */
static esp_err_t rc522_mifare_confirm_sector_trailer_write_permission_config(uint8_t block_address)
{
#ifdef CONFIG_RC522_PREVENT_SECTOR_TRAILER_WRITE
    uint8_t sector_index = rc522_mifare_get_sector_index_by_block_address(block_address);

    rc522_mifare_sector_desc_t sector;
    RC522_RETURN_ON_ERROR(rc522_mifare_get_sector_desc(sector_index, &sector));

    uint8_t trailer_address = sector.block_0_address + sector.number_of_blocks - 1;

    if (block_address == trailer_address) {
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG,
            "The block at address %d that you are trying to update is a Sector Trailer block.",
            block_address);
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "Writing to Sector Trailer blocks is prevented by default in the component configuration");
        ESP_LOGE(TAG, "to protect inexperienced users from accidentally overwriting keys");
        ESP_LOGE(TAG, "or writing incorrect access bits, which could make the sector unusable.");
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "If you know what you are doing, please use menuconfig to enable this option.");
        ESP_LOGE(TAG, "");

        return ESP_FAIL;
    }
#endif

    return ESP_OK;
}

esp_err_t rc522_mifare_write(
    rc522_handle_t rc522, rc522_picc_t *picc, uint8_t block_addr, uint8_t *buffer, uint8_t buffer_size)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(buffer == NULL);
    RC522_CHECK(buffer_size < RC522_MIFARE_BLOCK_SIZE);
    RC522_CHECK(!rc522_mifare_type_is_classic_compatible(picc->type));
    RC522_CHECK(rc522_mifare_confirm_sector_trailer_write_permission_config(block_addr) != ESP_OK);

    RC522_LOGD("MIFARE WRITE (block_addr=%02" RC522_X ")", block_addr);

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

inline esp_err_t rc522_mifare_deauth(rc522_handle_t rc522, rc522_picc_t *picc)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);

    RC522_RETURN_ON_ERROR(rc522_pcd_stop_crypto1(rc522));

    return ESP_OK;
}

esp_err_t rc522_mifare_get_desc(rc522_picc_t *picc, rc522_mifare_desc_t *out_mifare_desc)
{
    RC522_CHECK(picc == NULL);
    RC522_CHECK(out_mifare_desc == NULL);

    rc522_mifare_desc_t desc;
    memset(&desc, 0, sizeof(desc));

    RC522_RETURN_ON_ERROR(rc522_mifare_get_number_of_sectors(picc->type, &desc.number_of_sectors));

    memcpy(out_mifare_desc, &desc, sizeof(rc522_mifare_desc_t));

    return ESP_OK;
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
 * Resulting groups:
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
static esp_err_t rc522_mifare_parse_sector_trailer(
    uint8_t *trailer_bytes, uint8_t trailer_bytes_size, rc522_mifare_sector_trailer_data_t *out_sector_trailer_data)
{
    RC522_CHECK(trailer_bytes == NULL);
    RC522_CHECK(trailer_bytes_size != RC522_MIFARE_BLOCK_SIZE);
    RC522_CHECK(out_sector_trailer_data == NULL);

    rc522_mifare_sector_trailer_data_t data;
    memset(&data, 0, sizeof(data));

    uint8_t c1 = trailer_bytes[7] >> 4;
    uint8_t c2 = trailer_bytes[8] & 0x0F;
    uint8_t c3 = trailer_bytes[8] >> 4;
    uint8_t c1_ = trailer_bytes[6] & 0x0F;
    uint8_t c2_ = trailer_bytes[6] >> 4;
    uint8_t c3_ = trailer_bytes[7] & 0x0F;

    if ((c1 != (~c1_ & 0xF)) || (c2 != (~c2_ & 0xF)) || (c3 != (~c3_ & 0xF))) {
        data.err = RC522_ERR_MIFARE_ACCESS_BITS_INTEGRITY_VIOLATION;
    }

    data.access_bit_groups[0] = ((c1 & 1) << 2) | ((c2 & 1) << 1) | ((c3 & 1) << 0);
    data.access_bit_groups[1] = ((c1 & 2) << 1) | ((c2 & 2) << 0) | ((c3 & 2) >> 1);
    data.access_bit_groups[2] = ((c1 & 4) << 0) | ((c2 & 4) >> 1) | ((c3 & 4) >> 2);
    data.access_bit_groups[3] = ((c1 & 8) >> 1) | ((c2 & 8) >> 2) | ((c3 & 8) >> 3);

    memcpy(out_sector_trailer_data, &data, sizeof(rc522_mifare_sector_trailer_data_t));

    return ESP_OK;
}

static esp_err_t rc522_mifare_parse_access_bits_group(uint8_t access_bits_group, rc522_mifare_access_bits_t *out_bits)
{
    RC522_CHECK(out_bits == NULL);

    out_bits->group = access_bits_group;
    out_bits->c1 = (access_bits_group >> 2) & 1;
    out_bits->c2 = (access_bits_group >> 1) & 1;
    out_bits->c3 = (access_bits_group >> 0) & 1;

    return ESP_OK;
}

/**
 * Checks if block is Value block based on access bits
 */
inline static bool rc522_mifare_block_is_value(uint8_t access_bits_group)
{
    return access_bits_group == 0b110 || access_bits_group == 0b001;
}

static esp_err_t rc522_mifare_parse_value_block(
    uint8_t *bytes, uint8_t bytes_size, rc522_mifare_value_block_data_t *out_value_block_data)
{
    RC522_CHECK(bytes == NULL);
    RC522_CHECK(bytes_size != RC522_MIFARE_BLOCK_SIZE);
    RC522_CHECK(out_value_block_data == NULL);

    rc522_mifare_value_block_data_t data;
    memset(&data, 0, sizeof(data));

    data.value = 0;

    data.value |= ((int32_t)(bytes[3]) << (8 * 3));
    data.value |= ((int32_t)(bytes[2]) << (8 * 2));
    data.value |= ((int32_t)(bytes[1]) << (8 * 1));
    data.value |= ((int32_t)(bytes[0]) << (8 * 0));

    data.addr = bytes[12];

    // TODO: Check for integrity violation
    //       Use RC522_ERR_MIFARE_VALUE_BLOCK_INTEGRITY_VIOLATION
    data.err = ESP_OK;

    memcpy(out_value_block_data, &data, sizeof(rc522_mifare_value_block_data_t));

    return ESP_OK;
}

esp_err_t rc522_mifare_read_sector_block(rc522_handle_t rc522, rc522_picc_t *picc,
    rc522_mifare_sector_desc_t *sector_desc, uint8_t block_offset, rc522_mifare_sector_block_t *out_block)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(sector_desc == NULL);
    RC522_CHECK(out_block == NULL);
    RC522_CHECK(block_offset >= sector_desc->number_of_blocks);

    rc522_mifare_sector_block_t block;
    memset(&block, 0x00, sizeof(rc522_mifare_sector_block_t));

    block.address = (sector_desc->block_0_address + block_offset);

    if (block_offset == sector_desc->number_of_blocks - 1) {
        block.type = RC522_MIFARE_BLOCK_TRAILER;
    }

    RC522_RETURN_ON_ERROR(rc522_mifare_read(rc522, picc, block.address, block.bytes, sizeof(block.bytes)));

    if (block.type == RC522_MIFARE_BLOCK_TRAILER) {
        RC522_RETURN_ON_ERROR(rc522_mifare_parse_sector_trailer(block.bytes, sizeof(block.bytes), &block.trailer_data));
    }

    uint8_t group = block_offset;
    bool is_first_in_group = true;

    if (sector_desc->number_of_blocks != 4) {
        group = block_offset / 5;
        is_first_in_group = (group == 3) || (group != (block_offset + 1) / 5);
    }

    if (is_first_in_group) {
        RC522_RETURN_ON_ERROR(
            rc522_mifare_parse_access_bits_group(block.trailer_data.access_bit_groups[group], &block.access_bits));
    }

    if (block.type != RC522_MIFARE_BLOCK_TRAILER) {
        if (block.address == 0x00) {
            block.type = RC522_MIFARE_BLOCK_MANUFACTURER_DATA;
        }
        else if (rc522_mifare_block_is_value(block.trailer_data.access_bit_groups[group])) {
            block.type = RC522_MIFARE_BLOCK_VALUE;
        }
        else {
            block.type = RC522_MIFARE_BLOCK_DATA;
        }
    }

    if (block.type == RC522_MIFARE_BLOCK_VALUE) {
        RC522_RETURN_ON_ERROR(rc522_mifare_parse_value_block(block.bytes, sizeof(block.bytes), &block.value_data));
    }

    memcpy(out_block, &block, sizeof(rc522_mifare_sector_block_t));

    return ESP_OK;
}
