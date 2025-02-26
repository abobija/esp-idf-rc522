/**
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * # Memory access management via access bits
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * The access conditions for every data block and sector trailer are defined by 3 bits, which
 * are stored non-inverted and inverted in the sector trailer of the specified sector.
 *
 * The access bits control the rights of memory access using the secret keys A and B. The
 * access conditions may be altered, provided one knows the relevant key and the current
 * access condition allows this operation.
 *
 * @warning
 * With each memory access the internal logic verifies the format of the access
 * conditions. If it detects a format violation the whole sector is irreversibly blocked.
 *
 * +-----------------------------------------------------------------------------------+
 * |                                   Sector Trailer                                  |
 * +=============+===+===+===+===+===+===+===+===+===+===+====+====+====+====+====+====+
 * | Byte number | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 |
 * +-------------+---+---+---+---+---+---+---+---+---+---+----+----+----+----+----+----+
 * | Description |         Key A         |  Access Bits  |       Key B (optional)      |
 * +-------------+-----------------------+---------------+-----------------------------+
 *
 * Bytes 6, 7 and 8 of sector trailer block are bytes that holds access bits.
 * Next table shows descriptions for each bit of those bytes.
 *
 * +-----+------+------+------+------+------+------+------+------+
 * |     |   7  |   6  |   5  |   4  |   3  |   2  |   1  |   0  |
 * +-----+------+------+------+------+------+------+------+------+
 * | [6] | ~C23 | ~C22 | ~C21 | ~C20 | ~C13 | ~C12 | ~C11 | ~C10 |
 * +-----+------+------+------+------+------+------+------+------+
 * | [7] |  C13 |  C12 |  C11 |  C10 | ~C33 | ~C32 | ~C31 | ~C30 |
 * +-----+------+------+------+------+------+------+------+------+
 * | [8] |  C33 |  C32 |  C31 |  C30 |  C23 |  C22 |  C21 |  C20 |
 * +-----+------+------+------+------+------+------+------+------+
 *
 * Where Cxy is access bit Cx for the block (group) y, for example:
 *  - C10 is the access bit C1 for the block (group) 0,
 *  - C20 is the access bit C2 for the block (group) 0,
 *  - ...,
 *  - C33 is the access bit C3 for the sector trailer,
 *  - ~C10 is the inverted bit C10.
 */

#include <string.h>
#include "rc522_types_internal.h"
#include "rc522_internal.h"
#include "rc522_helpers_internal.h"
#include "rc522_pcd_internal.h"
#include "rc522_picc_internal.h"
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

inline esp_err_t rc522_mifare_auth_sector(const rc522_handle_t rc522, const rc522_picc_t *picc,
    const rc522_mifare_sector_desc_t *sector_desc, const rc522_mifare_key_t *key)
{
    RC522_CHECK(sector_desc == NULL);

    RC522_RETURN_ON_ERROR(rc522_mifare_auth(rc522, picc, sector_desc->block_0_address, key));

    return ESP_OK;
}

esp_err_t rc522_mifare_auth(
    const rc522_handle_t rc522, const rc522_picc_t *picc, uint8_t block_address, const rc522_mifare_key_t *key)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(key == NULL);

    // TODO: Validate block_address

    RC522_LOGD("MIFARE AUTH (block_address=%02" RC522_X ")", block_address);

    uint8_t send_data[12];

    switch (key->type) {
        case RC522_MIFARE_KEY_A:
            send_data[0] = RC522_MIFARE_AUTH_KEY_A_CMD;
            break;
        case RC522_MIFARE_KEY_B:
            send_data[0] = RC522_MIFARE_AUTH_KEY_B_CMD;
            break;
        default:
            RC522_LOGE("Invalid key type");
            return ESP_ERR_INVALID_ARG;
    }

    send_data[1] = block_address;
    memcpy(send_data + 2, key->value, RC522_MIFARE_KEY_SIZE);

    // Use the last uid bytes
    // section 3.2.5 "MIFARE Classic Authentication".
    // The only missed case is the MF1Sxxxx shortcut activation,
    // but it requires cascade tag (CT) byte, that is not part of uid.
    memcpy(send_data + 8, picc->uid.value + picc->uid.length - 4, 4);

    // Start the authentication.

    rc522_picc_transaction_t transaction = {
        .pcd_command = RC522_PCD_MF_AUTH_CMD,
        .expected_interrupts = RC522_PCD_IDLE_IRQ_BIT,
        .bytes = { .ptr = send_data, .length = sizeof(send_data) },
    };

    esp_err_t ret = rc522_picc_send(rc522, &transaction, NULL);

    // 10.3.1.9
    //
    // "If an error occurs during authentication,
    // the ErrorReg register’s ProtocolErr bit is set to logic 1
    // and the Status2Reg register’s Crypto1On bit is set to logic 0"

    // Timer interrupt fires before ProtocolErr bit is set to 1
    // so we will only check for Crypto1On bit (even if ret == ESP_OK)
    uint8_t status2;
    RC522_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_PCD_STATUS_2_REG, &status2));

    if (!(status2 & RC522_PCD_MF_CRYPTO1_ON_BIT)) {
        return RC522_ERR_MIFARE_AUTHENTICATION_FAILED;
    }

    return ret;
}

esp_err_t rc522_mifare_read(const rc522_handle_t rc522, const rc522_picc_t *picc, uint8_t block_address,
    uint8_t out_buffer[RC522_MIFARE_BLOCK_SIZE])
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(out_buffer == NULL);

    RC522_LOGD("MIFARE READ (block_address=%02" RC522_X ")", block_address);

    uint8_t cmd_buffer[4] = { 0 };

    // Build command buffer
    cmd_buffer[0] = RC522_MIFARE_READ_CMD;
    cmd_buffer[1] = block_address;

    // Calculate CRC_A
    rc522_pcd_crc_t crc = { 0 };
    RC522_RETURN_ON_ERROR(rc522_pcd_calculate_crc(rc522, &(rc522_bytes_t) { .ptr = cmd_buffer, .length = 2 }, &crc));

    cmd_buffer[2] = crc.lsb;
    cmd_buffer[3] = crc.msb;

    uint8_t block_buffer[RC522_MIFARE_BLOCK_SIZE + 2] = { 0 }; // +2 for CRC_A

    rc522_picc_transaction_t transaction = {
        .bytes = { .ptr = cmd_buffer, .length = sizeof(cmd_buffer) },
        .check_crc = true,
    };

    rc522_picc_transaction_result_t result = {
        .bytes = { .ptr = block_buffer, .length = sizeof(block_buffer) },
    };

    RC522_RETURN_ON_ERROR(rc522_picc_transceive(rc522, &transaction, &result));
    RC522_CHECK_AND_RETURN((result.bytes.length - 2) != RC522_MIFARE_BLOCK_SIZE, ESP_FAIL);

    memcpy(out_buffer, block_buffer, sizeof(block_buffer) - 2); // -2 cuz of CRC_A

    return ESP_OK;
}

static esp_err_t rc522_mifare_send(const rc522_handle_t rc522, const uint8_t *send_data, uint8_t send_length)
{
    RC522_CHECK(send_data == NULL);
    RC522_CHECK(send_length > RC522_MIFARE_BLOCK_SIZE);

#pragma GCC diagnostic ignored "-Wcast-qual"
    uint8_t *sdata = (uint8_t *)send_data;
#pragma GCC diagnostic pop

    rc522_pcd_crc_t crc = { 0 };
    RC522_RETURN_ON_ERROR(
        rc522_pcd_calculate_crc(rc522, &(rc522_bytes_t) { .ptr = sdata, .length = send_length }, &crc));

    uint8_t buffer[RC522_MIFARE_BLOCK_SIZE + 2]; // +2 for CRC_A

    memcpy(buffer, send_data, send_length);

    buffer[send_length] = crc.lsb;
    buffer[send_length + 1] = crc.msb;

    send_length += 2;

    rc522_picc_transaction_t transaction = {
        .bytes = { .ptr = buffer, .length = send_length },
    };

    rc522_picc_transaction_result_t result = {
        .bytes = { .ptr = buffer, .length = sizeof(buffer) },
    };

    RC522_RETURN_ON_ERROR(rc522_picc_transceive(rc522, &transaction, &result));

    // The PICC must reply with a 4 bit ACK
    if (result.bytes.length != 1 || result.valid_bits != 4) {
        return ESP_FAIL; // TODO: use custom err
    }

    if (result.bytes.ptr[0] != RC522_MIFARE_ACK) {
        return RC522_ERR_MIFARE_NACK;
    }

    return ESP_OK;
}

esp_err_t rc522_mifare_get_number_of_sectors(rc522_picc_type_t type, uint8_t *out_result)
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

inline uint8_t rc522_mifare_get_sector_index_by_block_address(uint8_t block_address)
{
    if (block_address < 128) {
        return block_address / 4;
    }
    else {
        return 32 + ((block_address - 128) / 16);
    }
}

inline esp_err_t rc522_mifare_get_number_of_blocks_in_sector(uint8_t sector_index, uint8_t *out_result)
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

inline esp_err_t rc522_mifare_get_sector_block_0_address(uint8_t sector_index, uint8_t *out_result)
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
    RC522_RETURN_ON_ERROR(rc522_mifare_get_number_of_blocks_in_sector(sector_index, &desc.number_of_blocks));
    RC522_RETURN_ON_ERROR(rc522_mifare_get_sector_block_0_address(sector_index, &desc.block_0_address));

    memcpy(out_sector_desc, &desc, sizeof(rc522_mifare_sector_desc_t));

    return ESP_OK;
}

static esp_err_t rc522_mifare_block_at_address_is_sector_trailer(uint8_t block_address, bool *result)
{
    RC522_CHECK(result == NULL);

    uint8_t sector_index = rc522_mifare_get_sector_index_by_block_address(block_address);

    rc522_mifare_sector_desc_t sector = { 0 };
    RC522_RETURN_ON_ERROR(rc522_mifare_get_sector_desc(sector_index, &sector));

    *result = (block_address == (sector.block_0_address + sector.number_of_blocks - 1));

    return ESP_OK;
}

static esp_err_t rc522_mifare_verify_access_bits_integrity(const uint8_t trailer_bytes[RC522_MIFARE_BLOCK_SIZE])
{
    RC522_CHECK(trailer_bytes == NULL);

    uint8_t c1 = 0, c2 = 0, c3 = 0;
    uint8_t c1_ = 0, c2_ = 0, c3_ = 0;

    RC522_RETURN_ON_ERROR(rc522_nibbles(trailer_bytes[6], &c2_, &c1_));
    RC522_RETURN_ON_ERROR(rc522_nibbles(trailer_bytes[7], &c1, &c3_));
    RC522_RETURN_ON_ERROR(rc522_nibbles(trailer_bytes[8], &c3, &c2));

    if ((c1 != (~c1_ & 0x0F)) || (c2 != (~c2_ & 0x0F)) || (c3 != (~c3_ & 0x0F))) {
        return RC522_ERR_MIFARE_ACCESS_BITS_INTEGRITY_VIOLATION;
    }

    return ESP_OK;
}

esp_err_t rc522_mifare_write(const rc522_handle_t rc522, const rc522_picc_t *picc, uint8_t block_address,
    const uint8_t buffer[RC522_MIFARE_BLOCK_SIZE])
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(buffer == NULL);
    RC522_CHECK(!rc522_mifare_type_is_classic_compatible(picc->type));

    bool is_trailer = false;
    RC522_RETURN_ON_ERROR(rc522_mifare_block_at_address_is_sector_trailer(block_address, &is_trailer));

    if (is_trailer) {
#ifdef CONFIG_RC522_PREVENT_SECTOR_TRAILER_WRITE
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG,
            "The block at address %d that you are trying to update is a Sector Trailer block.",
            block_address);
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "Writing to Sector Trailer blocks is prevented by default in the component configuration");
        ESP_LOGE(TAG, "to protect inexperienced users from accidentally overwriting keys");
        ESP_LOGE(TAG, "or writing incorrect access bits, which could make the sector unusable.");
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "If you know what you are doing, please use menuconfig to disable this protection.");
        ESP_LOGE(TAG, "");

        return RC522_ERR_MIFARE_SECTOR_TRAILER_WRITE_NOT_ALLOWED;
#endif

        RC522_RETURN_ON_ERROR(rc522_mifare_verify_access_bits_integrity(buffer));
    }

    RC522_LOGD("MIFARE WRITE (block_address=%02" RC522_X ")", block_address);

    uint8_t cmd_buffer[] = { RC522_MIFARE_WRITE_CMD, block_address };

    RC522_RETURN_ON_ERROR(rc522_mifare_send(rc522, cmd_buffer, sizeof(cmd_buffer)));
    RC522_RETURN_ON_ERROR(rc522_mifare_send(rc522, buffer, RC522_MIFARE_BLOCK_SIZE));

    return ESP_OK;
}

inline esp_err_t rc522_mifare_deauth(const rc522_handle_t rc522, const rc522_picc_t *picc)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);

    RC522_RETURN_ON_ERROR(rc522_pcd_stop_crypto1(rc522));

    return ESP_OK;
}

esp_err_t rc522_mifare_get_desc(const rc522_picc_t *picc, rc522_mifare_desc_t *out_mifare_desc)
{
    RC522_CHECK(picc == NULL);
    RC522_CHECK(out_mifare_desc == NULL);

    rc522_mifare_desc_t desc;
    memset(&desc, 0, sizeof(desc));

    RC522_RETURN_ON_ERROR(rc522_mifare_get_number_of_sectors(picc->type, &desc.number_of_sectors));

    memcpy(out_mifare_desc, &desc, sizeof(rc522_mifare_desc_t));

    return ESP_OK;
}

static esp_err_t rc522_mifare_nibbles_to_access_bits(
    uint8_t c1, uint8_t c2, uint8_t c3, uint8_t offset, rc522_mifare_access_bits_t *access_bits)
{
    RC522_CHECK(offset > 3);
    RC522_CHECK(access_bits == NULL);

    rc522_mifare_access_bits_t bits = { 0 };

    bits.c1 = (c1 & (1 << offset)) >> offset;
    bits.c2 = (c2 & (1 << offset)) >> offset;
    bits.c3 = (c3 & (1 << offset)) >> offset;

    memcpy(access_bits, &bits, sizeof(bits));

    return ESP_OK;
}

static esp_err_t rc522_mifare_parse_sector_trailer(
    const uint8_t *trailer_bytes, uint8_t trailer_bytes_size, rc522_mifare_sector_trailer_info_t *out_trailer_info)
{
    RC522_CHECK(trailer_bytes == NULL);
    RC522_CHECK(trailer_bytes_size != RC522_MIFARE_BLOCK_SIZE);
    RC522_CHECK(out_trailer_info == NULL);

    RC522_RETURN_ON_ERROR(rc522_mifare_verify_access_bits_integrity(trailer_bytes));

    uint8_t c1 = 0, c2 = 0, c3 = 0;

    RC522_RETURN_ON_ERROR(rc522_nibbles(trailer_bytes[7], &c1, NULL));
    RC522_RETURN_ON_ERROR(rc522_nibbles(trailer_bytes[8], &c3, &c2));

    rc522_mifare_sector_trailer_info_t info = { 0 };

    for (uint8_t i = 0; i < 4; i++) {
        RC522_RETURN_ON_ERROR(rc522_mifare_nibbles_to_access_bits(c1, c2, c3, i, &info.access_bits[i]));
    }

    memcpy(out_trailer_info, &info, sizeof(info));

    return ESP_OK;
}

/**
 * Checks if block is Value block based on access bits
 */
inline static bool rc522_mifare_block_is_value(rc522_mifare_access_bits_t access_bits)
{
    uint8_t bits = (access_bits.c1 << 2) | (access_bits.c2 << 1) | (access_bits.c3 << 0);

    return bits == 0b110 || bits == 0b001;
}

static esp_err_t rc522_mifare_parse_value_block(
    const uint8_t *bytes, uint8_t bytes_size, rc522_mifare_sector_value_block_info_t *out_value_block_info)
{
    RC522_CHECK(bytes == NULL);
    RC522_CHECK(bytes_size != RC522_MIFARE_BLOCK_SIZE);
    RC522_CHECK(out_value_block_info == NULL);

    rc522_mifare_sector_value_block_info_t value_block_info;
    memset(&value_block_info, 0, sizeof(value_block_info));

    value_block_info.value = 0;

    value_block_info.value |= ((int32_t)(bytes[3]) << (8 * 3));
    value_block_info.value |= ((int32_t)(bytes[2]) << (8 * 2));
    value_block_info.value |= ((int32_t)(bytes[1]) << (8 * 1));
    value_block_info.value |= ((int32_t)(bytes[0]) << (8 * 0));

    value_block_info.addr = bytes[12];

    // TODO: Check for integrity violation
    //       Use RC522_ERR_MIFARE_VALUE_BLOCK_INTEGRITY_VIOLATION

    memcpy(out_value_block_info, &value_block_info, sizeof(value_block_info));

    return ESP_OK;
}

inline static esp_err_t rc522_mifare_get_sector_block_group_index(
    const rc522_mifare_sector_desc_t *sector, uint8_t block_offset, uint8_t *out_group)
{
    RC522_CHECK(sector == NULL);
    RC522_CHECK(out_group == NULL);
    RC522_CHECK(block_offset >= sector->number_of_blocks);

    if (sector->number_of_blocks > 4) {
        *out_group = block_offset / 5;
        return ESP_OK;
    }

    *out_group = block_offset;
    return ESP_OK;
}

esp_err_t rc522_mifare_read_sector_trailer_block(const rc522_handle_t rc522, const rc522_picc_t *picc,
    const rc522_mifare_sector_desc_t *sector_desc, rc522_mifare_sector_block_t *out_trailer)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(sector_desc == NULL);
    RC522_CHECK(out_trailer == NULL);

    rc522_mifare_sector_block_t block;
    memset(&block, 0x00, sizeof(block));

    block.address = (sector_desc->block_0_address + sector_desc->number_of_blocks - 1);
    block.type = RC522_MIFARE_BLOCK_TRAILER;

    RC522_RETURN_ON_ERROR(rc522_mifare_read(rc522, picc, block.address, block.bytes));
    block.error = rc522_mifare_parse_sector_trailer(block.bytes, sizeof(block.bytes), &block.trailer_info);
    memcpy(&block.access_bits, &block.trailer_info.access_bits[3], sizeof(rc522_mifare_access_bits_t));

    memcpy(out_trailer, &block, sizeof(block));

    return ESP_OK;
}

esp_err_t rc522_mifare_read_sector_block(const rc522_handle_t rc522, const rc522_picc_t *picc,
    const rc522_mifare_sector_desc_t *sector_desc, const rc522_mifare_sector_block_t *trailer, uint8_t block_offset,
    rc522_mifare_sector_block_t *out_block)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(sector_desc == NULL);
    RC522_CHECK(trailer == NULL);
    RC522_CHECK(block_offset >= sector_desc->number_of_blocks);
    RC522_CHECK(out_block == NULL);
    RC522_CHECK_WITH_MESSAGE(block_offset == sector_desc->number_of_blocks - 1,
        "use rc522_mifare_read_sector_trailer_block() to read the sector trailer block");
    RC522_CHECK(trailer->type != RC522_MIFARE_BLOCK_TRAILER);

    rc522_mifare_sector_block_t block;
    memset(&block, 0, sizeof(block));

    block.address = (sector_desc->block_0_address + block_offset);

    RC522_RETURN_ON_ERROR(rc522_mifare_read(rc522, picc, block.address, block.bytes));

    uint8_t group = 0;
    RC522_RETURN_ON_ERROR(rc522_mifare_get_sector_block_group_index(sector_desc, block_offset, &group));

    memcpy(&block.access_bits, &trailer->trailer_info.access_bits[group], sizeof(rc522_mifare_access_bits_t));

    if (block.address == 0) {
        block.type = RC522_MIFARE_BLOCK_MANUFACTURER_DATA;
    }
    else if (rc522_mifare_block_is_value(block.access_bits)) {
        block.type = RC522_MIFARE_BLOCK_VALUE;
        block.error = rc522_mifare_parse_value_block(block.bytes, sizeof(block.bytes), &block.value_info);
    }
    else {
        block.type = RC522_MIFARE_BLOCK_DATA;
    }

    memcpy(out_block, &block, sizeof(block));

    return ESP_OK;
}
