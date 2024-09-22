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
    send_data[1] = block_address;
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
    const rc522_handle_t rc522, const rc522_picc_t *picc, uint8_t block_address, uint8_t *buffer, uint8_t buffer_size)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(buffer == NULL);
    RC522_CHECK(buffer_size < RC522_MIFARE_BLOCK_SIZE);

    RC522_LOGD("MIFARE READ (block_address=%02" RC522_X ")", block_address);

    // FIXME: Cloning the buffer since picc_transceive uses +2 bytes buffer
    //        to store some extra data (crc i think). Refactor picc_transceive
    //        and picc_comm functions!
    uint8_t buffer_clone[RC522_MIFARE_BLOCK_SIZE + 2];

    // Build command buffer
    buffer_clone[0] = RC522_MIFARE_READ_CMD;
    buffer_clone[1] = block_address;

    // Calculate CRC_A
    uint16_t crc = 0;
    RC522_RETURN_ON_ERROR(rc522_pcd_calculate_crc(rc522, &(rc522_bytes_t) { .ptr = buffer_clone, .length = 2 }, &crc));

    buffer_clone[2] = crc & 0xFF;
    buffer_clone[3] = crc >> 8;

    // Transmit the buffer and receive the response, validate CRC_A.
    uint8_t _ = sizeof(buffer_clone);
    RC522_RETURN_ON_ERROR(rc522_picc_transceive(rc522, buffer_clone, 4, buffer_clone, &_, NULL, 0, true));

    RC522_CHECK(_ != (RC522_MIFARE_BLOCK_SIZE + 2));

    // FIXME: Move data back to the original buffer
    memcpy(buffer, buffer_clone, RC522_MIFARE_BLOCK_SIZE);

    return ESP_OK;
}

static esp_err_t rc522_mifare_send(
    const rc522_handle_t rc522, const uint8_t *send_data, uint8_t send_length, bool accept_timeout)
{
    RC522_CHECK(send_data == NULL);
    RC522_CHECK(send_length > RC522_MIFARE_BLOCK_SIZE);

    uint8_t cmd_buffer[RC522_MIFARE_BLOCK_SIZE + 2]; // We need room for data + 2 bytes CRC_A

    // Copy sendData[] to cmdBuffer[] and add CRC_A
    memcpy(cmd_buffer, send_data, send_length);

    uint16_t crc = 0;
    RC522_RETURN_ON_ERROR(
        rc522_pcd_calculate_crc(rc522, &(rc522_bytes_t) { .ptr = cmd_buffer, .length = send_length }, &crc));

    cmd_buffer[send_length] = crc & 0xFF;
    cmd_buffer[send_length + 1] = crc >> 8;

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

esp_err_t rc522_mifare_write(const rc522_handle_t rc522, const rc522_picc_t *picc, uint8_t block_address,
    const uint8_t *buffer, uint8_t buffer_size)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(buffer == NULL);
    RC522_CHECK(buffer_size < RC522_MIFARE_BLOCK_SIZE);
    RC522_CHECK(!rc522_mifare_type_is_classic_compatible(picc->type));
    RC522_CHECK(rc522_mifare_confirm_sector_trailer_write_permission_config(block_address) != ESP_OK);

    RC522_LOGD("MIFARE WRITE (block_address=%02" RC522_X ")", block_address);

    // Step 1: Tell the PICC we want to write to block blockAddr.
    uint8_t cmd_buffer[] = { RC522_MIFARE_WRITE_CMD, block_address };
    RC522_RETURN_ON_ERROR(
        rc522_mifare_send(rc522, cmd_buffer, 2, false)); // Adds CRC_A and checks that the response is MF_ACK.

    RC522_RETURN_ON_ERROR(rc522_mifare_send(rc522,
        buffer,
        RC522_MIFARE_BLOCK_SIZE,
        false)); // Adds CRC_A and checks that the response is MF_ACK.

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
 * Where Cx is access bit for the block group y, for example:
 *  - C10 is the access bit C1 for the block group 0,
 *  - C20 is the access bit C2 for the block group 0,
 *  - ...,
 *  - C33 is the access bit C3 for the sector trailer.
 */
static esp_err_t rc522_mifare_parse_sector_trailer(
    const uint8_t *trailer_bytes, uint8_t trailer_bytes_size, rc522_mifare_sector_trailer_info_t *out_trailer_info)
{
    RC522_CHECK(trailer_bytes == NULL);
    RC522_CHECK(trailer_bytes_size != RC522_MIFARE_BLOCK_SIZE);
    RC522_CHECK(out_trailer_info == NULL);

    rc522_mifare_sector_trailer_info_t trailer_info;
    memset(&trailer_info, 0, sizeof(trailer_info));

    uint8_t c3 = trailer_bytes[8] >> 4;
    uint8_t c2 = trailer_bytes[8] & 0x0F;
    uint8_t c1 = trailer_bytes[7] >> 4;
    uint8_t c3_ = trailer_bytes[7] & 0x0F;
    uint8_t c2_ = trailer_bytes[6] >> 4;
    uint8_t c1_ = trailer_bytes[6] & 0x0F;

    if ((c1 != (~c1_ & 0xF)) || (c2 != (~c2_ & 0xF)) || (c3 != (~c3_ & 0xF))) {
        return RC522_ERR_MIFARE_ACCESS_BITS_INTEGRITY_VIOLATION;
    }

    // trailer
    trailer_info.access_bits[3].c1 = (c1 & 8) >> 3;
    trailer_info.access_bits[3].c2 = (c2 & 8) >> 3;
    trailer_info.access_bits[3].c3 = (c3 & 8) >> 3;

    // block 2
    trailer_info.access_bits[2].c1 = (c1 & 4) >> 2;
    trailer_info.access_bits[2].c2 = (c2 & 4) >> 2;
    trailer_info.access_bits[2].c3 = (c3 & 4) >> 2;

    // block 1
    trailer_info.access_bits[1].c1 = (c1 & 2) >> 1;
    trailer_info.access_bits[1].c2 = (c2 & 2) >> 1;
    trailer_info.access_bits[1].c3 = (c3 & 2) >> 1;

    // block 0
    trailer_info.access_bits[0].c1 = (c1 & 1) >> 0;
    trailer_info.access_bits[0].c2 = (c2 & 1) >> 0;
    trailer_info.access_bits[0].c3 = (c3 & 1) >> 0;

    memcpy(out_trailer_info, &trailer_info, sizeof(trailer_info));

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

    RC522_RETURN_ON_ERROR(rc522_mifare_read(rc522, picc, block.address, block.bytes, sizeof(block.bytes)));
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

    RC522_RETURN_ON_ERROR(rc522_mifare_read(rc522, picc, block.address, block.bytes, sizeof(block.bytes)));

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
