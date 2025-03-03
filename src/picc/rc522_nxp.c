#include <esp_system.h>
#include <esp_check.h>
#include <string.h>

#include "rc522_picc.h"
#include "rc522_types_internal.h"
#include "rc522_pcd_internal.h"
#include "rc522_picc_internal.h"

#include "picc/rc522_nxp.h"
#include "picc/rc522_mifare.h"

RC522_LOG_DEFINE_BASE();

const uint8_t RC522_NXP_DEFAULT_PWD[RC522_NXP_PWD_SIZE] = { 0xFF, 0xFF, 0xFF, 0xFF };
const uint8_t RC522_NXP_DEFAULT_PACK[RC522_NXP_PACK_SIZE] = { 0x00 };

// Helper to check if the card responded with a NACK
inline esp_err_t rc522_nxp_check_for_nak(rc522_picc_transaction_result_t *result)
{
    if (result->bytes.length == 1 && result->valid_bits == 4 && result->bytes.ptr[0] != RC522_MIFARE_ACK) {
        return RC522_ERR_NXP_NACK;
    }
    return ESP_OK;
}

// WRITE supported? (not classic write - see COMPAT_WRITE)
inline bool rc522_nxp_type_has_write(rc522_picc_type_t type)
{
    switch (type) {
        case RC522_PICC_TYPE_MIFARE_UL_:
        case RC522_PICC_TYPE_MIFARE_UL_C:
        case RC522_PICC_TYPE_MIFARE_UL_EV1_1:
        case RC522_PICC_TYPE_MIFARE_UL_EV1_2:
        case RC522_PICC_TYPE_MIFARE_UL_NANO:
        case RC522_PICC_TYPE_MIFARE_UL_AES:
        case RC522_PICC_TYPE_NTAG2xx:
        case RC522_PICC_TYPE_NTAG213:
        case RC522_PICC_TYPE_NTAG215:
        case RC522_PICC_TYPE_NTAG216:
            return true;
        default:
            return false;
    }
}

// FAST_READ supported?
inline bool rc522_nxp_type_has_fast_read(rc522_picc_type_t type)
{
    switch (type) {
        case RC522_PICC_TYPE_MIFARE_UL_EV1_1:
        case RC522_PICC_TYPE_MIFARE_UL_EV1_2:
        case RC522_PICC_TYPE_MIFARE_UL_AES:
        case RC522_PICC_TYPE_NTAG213:
        case RC522_PICC_TYPE_NTAG215:
        case RC522_PICC_TYPE_NTAG216:
            return true;
        default:
            return false;
    }
}

// READ_CNT supported?
inline bool rc522_nxp_type_has_read_cnt(rc522_picc_type_t type)
{
    switch (type) {
        case RC522_PICC_TYPE_MIFARE_UL_EV1_1:
        case RC522_PICC_TYPE_MIFARE_UL_EV1_2:
        case RC522_PICC_TYPE_MIFARE_UL_AES:
        case RC522_PICC_TYPE_NTAG213:
        case RC522_PICC_TYPE_NTAG215:
        case RC522_PICC_TYPE_NTAG216:
            return true;
        default:
            return false;
    }
}

// PWD_AUTH supported?
inline bool rc522_nxp_type_has_pwd_auth(rc522_picc_type_t type)
{
    switch (type) {
        case RC522_PICC_TYPE_MIFARE_UL_EV1_1:
        case RC522_PICC_TYPE_MIFARE_UL_EV1_2:
        case RC522_PICC_TYPE_NTAG213:
        case RC522_PICC_TYPE_NTAG215:
        case RC522_PICC_TYPE_NTAG216:
            return true;
        default:
            return false;
    }
}

// Get originality signature size, or 0 if not supported
inline uint8_t rc522_nxp_type_sig_size(rc522_picc_type_t type)
{
    switch (type) {
        case RC522_PICC_TYPE_MIFARE_UL_AES:
            return 48;
        case RC522_PICC_TYPE_MIFARE_UL_EV1_1:
        case RC522_PICC_TYPE_MIFARE_UL_EV1_2:
        case RC522_PICC_TYPE_MIFARE_UL_NANO:
        case RC522_PICC_TYPE_NTAG213:
        case RC522_PICC_TYPE_NTAG215:
        case RC522_PICC_TYPE_NTAG216:
            return 32;
        default:
            return 0;
    }
}

// Convert GET_VERSION response to type
static rc522_picc_type_t rc522_nxp_type_from_version(rc522_nxp_picc_version_t *version)
{
    rc522_nxp_product_type_t type = version->product_type;
    rc522_nxp_major_version_t major_version = version->major_version;
    uint8_t size = version->storage_size;
    if (type == RC522_NXP_PRODUCT_TYPE_NTAG) {
        if (major_version != RC522_NXP_MAJ_VER_NTAG21) { // Not an NTAG21x
            RC522_LOGW("Unknown NTAG product version: %02" RC522_X, major_version);
        }
        else if (size == 0x0F) { // 0x0F => between 128 and 256 user bytes
            return RC522_PICC_TYPE_NTAG213;
        }
        else if (size == 0x11) { // 0x11 => between 256 and 512 user bytes
            return RC522_PICC_TYPE_NTAG215;
        }
        else if (size == 0x13) { // 0x13 => between 512 and 1024 user byets
            return RC522_PICC_TYPE_NTAG216;
        }
        else { // Unknown size?
            RC522_LOGW("Unknown NTAG size: %02" RC522_X, size);
        }
    }
    else if (type == RC522_NXP_PRODUCT_TYPE_UL) {
        if (major_version == RC522_NXP_MAJ_VER_UL_EV1) {
            if (size == 0x0B) { // 0x0B => between 32 and 64 user bytes
                return RC522_PICC_TYPE_MIFARE_UL_EV1_1;
            }
            else if (size == 0x0E) { // 0x0E => 128 user bytes
                return RC522_PICC_TYPE_MIFARE_UL_EV1_2;
            }
        }
        else if (major_version == RC522_NXP_MAJ_VER_UL_NANO) {
            return RC522_PICC_TYPE_MIFARE_UL_NANO;
        }
        else if (major_version == RC522_NXP_MAJ_VER_UL_AES) {
            return RC522_PICC_TYPE_MIFARE_UL_AES;
        }
        else {
            RC522_LOGW("Unknown UL product version: %02" RC522_X, major_version);
        }
    }
    else {
        RC522_LOGE("Unknown product type: %02" RC522_X, type);
    }
    return RC522_PICC_TYPE_UNKNOWN;
}

uint8_t rc522_nxp_get_user_page_count(rc522_picc_type_t type)
{
    switch (type) {
        case RC522_PICC_TYPE_MIFARE_UL_:
            return 12;
        case RC522_PICC_TYPE_MIFARE_UL_C:
            return 36;
        case RC522_PICC_TYPE_MIFARE_UL_EV1_1:
            return 12;
        case RC522_PICC_TYPE_MIFARE_UL_EV1_2:
            return 32;
        case RC522_PICC_TYPE_MIFARE_UL_NANO:
            return 10;
        case RC522_PICC_TYPE_MIFARE_UL_AES:
            return 36;
        case RC522_PICC_TYPE_NTAG213:
            return 36;
        case RC522_PICC_TYPE_NTAG215:
            return 126;
        case RC522_PICC_TYPE_NTAG216:
            return 222;
        default:
            return 0;
    }
}

uint8_t rc522_nxp_get_user_mem_start(rc522_picc_type_t type)
{
    switch (type) {
        case RC522_PICC_TYPE_MIFARE_UL_:
        case RC522_PICC_TYPE_MIFARE_UL_C:
        case RC522_PICC_TYPE_MIFARE_UL_EV1_1:
        case RC522_PICC_TYPE_MIFARE_UL_EV1_2:
        case RC522_PICC_TYPE_MIFARE_UL_NANO:
        case RC522_PICC_TYPE_MIFARE_UL_AES:
        case RC522_PICC_TYPE_NTAG2xx:
        case RC522_PICC_TYPE_NTAG213:
        case RC522_PICC_TYPE_NTAG215:
        case RC522_PICC_TYPE_NTAG216:
            return 0x04;
        default:
            return 0;
    }
}

uint8_t rc522_nxp_get_user_mem_end(rc522_picc_type_t type)
{
    switch (type) {
        case RC522_PICC_TYPE_MIFARE_UL_:
            return 0x0F;
        case RC522_PICC_TYPE_MIFARE_UL_C:
            return 0x27;
        case RC522_PICC_TYPE_MIFARE_UL_EV1_1:
            return 0x0F;
        case RC522_PICC_TYPE_MIFARE_UL_EV1_2:
            return 0x23;
        case RC522_PICC_TYPE_MIFARE_UL_NANO:
            return 0x0D;
        case RC522_PICC_TYPE_MIFARE_UL_AES:
            return 0x27;
        case RC522_PICC_TYPE_NTAG213:
            return 0x27;
        case RC522_PICC_TYPE_NTAG215:
            return 0x81;
        case RC522_PICC_TYPE_NTAG216:
            return 0xE1;
        default:
            return 0;
    }
}

uint8_t rc522_nxp_get_page_count(rc522_picc_type_t type)
{
    switch (type) {
        case RC522_PICC_TYPE_MIFARE_UL_:
            return 16;
        case RC522_PICC_TYPE_MIFARE_UL_C:
            return 48;
        case RC522_PICC_TYPE_MIFARE_UL_EV1_1:
            return 20;
        case RC522_PICC_TYPE_MIFARE_UL_EV1_2:
            return 41;
        case RC522_PICC_TYPE_MIFARE_UL_NANO:
            return 14;
        case RC522_PICC_TYPE_MIFARE_UL_AES:
            return 60;
        case RC522_PICC_TYPE_NTAG213:
            return 45;
        case RC522_PICC_TYPE_NTAG215:
            return 135;
        case RC522_PICC_TYPE_NTAG216:
            return 231;
        default:
            return 0;
    }
}

esp_err_t rc522_nxp_keyauth_supported(const rc522_handle_t rc522, const rc522_picc_t *picc)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_LOGD("AUTHENTICATE (support check)");

    uint8_t buffer[11] = { 0 }; // combined send(4)/recv(11) buffer
    buffer[0] = RC522_PICC_CMD_UL_AUTH;
    // buffer[1] is argument; 0x00 is fine for support check

    rc522_pcd_crc_t crc = { 0 };
    RC522_RETURN_ON_ERROR(rc522_pcd_calculate_crc(rc522, &(rc522_bytes_t) { .ptr = buffer, .length = 2 }, &crc));
    buffer[2] = crc.lsb;
    buffer[3] = crc.msb;

    rc522_picc_transaction_t transaction = {
        .bytes = { .ptr = buffer, .length = 4 },
    };

    rc522_picc_transaction_result_t result = {
        .bytes = { .ptr = buffer, .length = sizeof(buffer) },
    };

    esp_err_t ret = rc522_picc_transceive(rc522, &transaction, &result);
    if (ret == ESP_OK && buffer[0] != 0xAF) {
        return RC522_ERR_NXP_NACK;
    }
    return ret;
}

esp_err_t rc522_nxp_get_type(const rc522_handle_t rc522, const rc522_picc_t *picc, rc522_picc_type_t *out_type)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(out_type == NULL);
    RC522_CHECK(picc->type != RC522_PICC_TYPE_MIFARE_UL);
    RC522_LOGD("Attempting to get NXP PICC type");
    RC522_LOGD("Checking for GET_VERSION support...");

    // In case of early exit due to errors
    rc522_picc_type_t type = RC522_PICC_TYPE_MIFARE_UL;
    rc522_nxp_picc_version_t version_info = { 0 };
    esp_err_t ret = rc522_nxp_get_version(rc522, picc, &version_info);

    if (ret == ESP_OK) { // GET_VERSION supported
        type = rc522_nxp_type_from_version(&version_info);
    }
    else if (ret == RC522_ERR_RX_TIMER_TIMEOUT) { // No GET_VERSION
        // Failure probably resulted in a HALT, so make sure we can still connect
        RC522_LOGD("No L3 GET_VERSION; reselecting");
        rc522_picc_uid_t uid;
        uint8_t sak;
        rc522_picc_heartbeat(rc522, picc, &uid, &sak);

        // Fall back to checking for authentication availability (C)
        RC522_LOGD("Checking for authentication support...");
        ret = rc522_nxp_keyauth_supported(rc522, picc);
        if (ret == ESP_OK) {
            // Chip supports authentication
            // TODO: Do we need to complete authentication, or can we leave it here?
            // Note datasheet also mentions a "MIFARE Hospitality" here, but there
            // doesn't seem to be any more information about it
            type = RC522_PICC_TYPE_MIFARE_UL_C;
        }
        else if (ret == RC522_ERR_RX_TIMER_TIMEOUT || ret == RC522_ERR_MIFARE_NACK) { // Not supported
            // Only Ultralight without auth is the original
            type = RC522_PICC_TYPE_MIFARE_UL_;
        }
        else { // Some other error in AUTHENTICATE
            RC522_RETURN_ON_ERROR(ret);
        }
        rc522_picc_heartbeat(rc522, picc, &uid, &sak);
    }
    else { // A different error occured in GET_VERSION
        RC522_RETURN_ON_ERROR(ret);
    }
    RC522_LOGD("Found %s", rc522_picc_type_name(type));
    *out_type = type;
    return ESP_OK;
}

esp_err_t rc522_nxp_get_version(
    const rc522_handle_t rc522, const rc522_picc_t *picc, rc522_nxp_picc_version_t *out_version)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(out_version == NULL);

    RC522_LOGD("GET_VERSION");

    uint8_t buffer[10]; // Combined buffer; response is 8-byte data + 2-byte CRC
    buffer[0] = RC522_PICC_CMD_GETV;
    rc522_pcd_crc_t crc = { 0 };
    RC522_RETURN_ON_ERROR(rc522_pcd_calculate_crc(rc522, &(rc522_bytes_t) { .ptr = buffer, .length = 1 }, &crc));

    buffer[1] = crc.lsb;
    buffer[2] = crc.msb;

    rc522_picc_transaction_t transaction = {
        .bytes = { .ptr = buffer, .length = 3 },
    };

    rc522_picc_transaction_result_t result = {
        .bytes = { .ptr = buffer, .length = sizeof(buffer) },
    };

    RC522_RETURN_ON_ERROR(rc522_picc_transceive(rc522, &transaction, &result));
    RC522_RETURN_ON_ERROR(rc522_nxp_check_for_nak(&result));

    memcpy(out_version, buffer, sizeof(rc522_nxp_picc_version_t));
    return ESP_OK;
}

esp_err_t rc522_nxp_read(
    const rc522_handle_t rc522, const rc522_picc_t *picc, uint8_t address, uint8_t out_buffer[RC522_NXP_PAGE_SIZE * 4])
{
    // Same protocol as MIFARE cards, so defer
    return rc522_mifare_read(rc522, picc, address, out_buffer);
}

esp_err_t rc522_nxp_fast_read(const rc522_handle_t rc522, const rc522_picc_t *picc, uint8_t start_page,
    uint8_t end_page, rc522_nxp_fast_read_data_t *out_buffer)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(out_buffer == NULL);
    RC522_CHECK(!rc522_nxp_type_has_fast_read(picc->type));
    // some sanity checks - valid range, output buffer sufficiently large
    RC522_CHECK(start_page > end_page);
    RC522_CHECK(out_buffer->buffer_size < (end_page - start_page + 1) * 4);

    RC522_LOGD("NXP FAST_READ (start=%02" RC522_X ", end=%02" RC522_X ")", start_page, end_page);

    uint8_t cmd_buffer[5] = { RC522_NXP_FAST_READ, start_page, end_page, 0, 0 };
    uint8_t byte_count = (end_page - start_page + 1) * 4;

    rc522_pcd_crc_t crc = { 0 };
    RC522_RETURN_ON_ERROR(rc522_pcd_calculate_crc(rc522,
        &(rc522_bytes_t) {
            .ptr = cmd_buffer,
            .length = 3,
        },
        &crc));
    cmd_buffer[3] = crc.lsb;
    cmd_buffer[4] = crc.msb;

    // I'd rather not malloc() here, but we need extra space for the CRC which
    // we're not guaranteed to have in out_buffer
    // TODO: Allow for separate CRC fetch and processing to avoid this?
    uint8_t *recv_buffer = malloc(byte_count + 2); // Allow for CRC_A
    RC522_CHECK_AND_RETURN(recv_buffer == NULL, ESP_ERR_NO_MEM);

    rc522_picc_transaction_t transaction = {
        .bytes = { .ptr = cmd_buffer, .length = sizeof(cmd_buffer) },
        .check_crc = true,
    };
    rc522_picc_transaction_result_t result = {
        .bytes = { .ptr = recv_buffer, .length = byte_count + 2 },
    };

    // Bunch of manual handling here to make sure free() is called
    esp_err_t ret = rc522_picc_transceive(rc522, &transaction, &result);
    if (ret != ESP_OK) {
        free(recv_buffer);
        return ret;
    }
    else if ((result.bytes.length - 2) != byte_count) {
        free(recv_buffer);
        return ESP_FAIL;
    }

    memcpy(out_buffer->bytes, recv_buffer, byte_count);
    out_buffer->read_size = byte_count;
    free(recv_buffer);
    return ESP_OK;
}

esp_err_t rc522_nxp_write(
    const rc522_handle_t rc522, const rc522_picc_t *picc, uint8_t address, const uint8_t buffer[RC522_NXP_PAGE_SIZE])
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(buffer == NULL);
    RC522_CHECK(!rc522_nxp_type_has_write(picc->type));

    RC522_LOGD("NXP WRITE (address=%02" RC522_X ")", address);

    uint8_t cmd_buffer[RC522_NXP_PAGE_SIZE + 4] = { 0 };
    cmd_buffer[0] = RC522_NXP_WRITE;
    cmd_buffer[1] = address;

    memcpy(&cmd_buffer[2], buffer, RC522_NXP_PAGE_SIZE);

    rc522_pcd_crc_t crc = { 0 };
    RC522_RETURN_ON_ERROR(rc522_pcd_calculate_crc(rc522,
        &(rc522_bytes_t) {
            .ptr = cmd_buffer,
            .length = sizeof(cmd_buffer) - 2,
        },
        &crc));

    cmd_buffer[2 + RC522_NXP_PAGE_SIZE] = crc.lsb;
    cmd_buffer[2 + RC522_NXP_PAGE_SIZE + 1] = crc.msb;

    rc522_picc_transaction_t transaction = {
        .bytes = { .ptr = cmd_buffer, .length = sizeof(cmd_buffer) },
    };
    rc522_picc_transaction_result_t result = {
        .bytes = { .ptr = cmd_buffer, .length = sizeof(cmd_buffer) },
    };

    RC522_RETURN_ON_ERROR(rc522_picc_transceive(rc522, &transaction, &result));
    return rc522_nxp_check_for_nak(&result);
}

esp_err_t rc522_nxp_read_cnt(
    const rc522_handle_t rc522, const rc522_picc_t *picc, uint8_t counter_no, uint32_t *out_count)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(out_count == NULL);
    RC522_CHECK(!rc522_nxp_type_has_read_cnt(picc->type));

    RC522_LOGD("NXP READ_CNT (counter=%02" RC522_X ")", counter_no);

    uint8_t cmd_buffer[5] = {
        // send(4)/recv(5) buffer
        RC522_NXP_READ_CNT,
        counter_no,
        0,
        0, // CRC
        0  // Space for reply (3 bytes + CRC)
    };

    rc522_pcd_crc_t crc = { 0 };
    RC522_RETURN_ON_ERROR(rc522_pcd_calculate_crc(rc522,
        &(rc522_bytes_t) {
            .ptr = cmd_buffer,
            .length = sizeof(cmd_buffer) - 2,
        },
        &crc));
    cmd_buffer[2] = crc.lsb;
    cmd_buffer[3] = crc.msb;

    rc522_picc_transaction_t transaction = {
        .bytes = { .ptr = cmd_buffer, .length = 4 },
    };
    rc522_picc_transaction_result_t result = {
        .bytes = { .ptr = cmd_buffer, .length = sizeof(cmd_buffer) },
    };

    RC522_RETURN_ON_ERROR(rc522_picc_transceive(rc522, &transaction, &result));
    RC522_RETURN_ON_ERROR(rc522_nxp_check_for_nak(&result));

    // TODO: Verify this counter is correct
    *out_count = (cmd_buffer[2] << 16) | (cmd_buffer[1] << 8) | cmd_buffer[0];
    return ESP_OK;
}

esp_err_t rc522_nxp_pwd_auth(const rc522_handle_t rc522, const rc522_picc_t *picc,
    const uint8_t pwd[RC522_NXP_PWD_SIZE], const uint8_t pack[RC522_NXP_PACK_SIZE], rc522_picc_state_t *out_state)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(pwd == NULL);
    RC522_CHECK(pack == NULL);
    RC522_CHECK(!rc522_nxp_type_has_pwd_auth(picc->type));

    RC522_LOGD("NXP PWD_AUTH");

    uint8_t cmd_buffer[RC522_NXP_PWD_SIZE + 3] = { 0 }; // send(11)/recv(4) buffer
    cmd_buffer[0] = RC522_NXP_PWD_AUTH;
    memcpy(&cmd_buffer[1], pwd, RC522_NXP_PWD_SIZE);

    rc522_pcd_crc_t crc = { 0 };
    RC522_RETURN_ON_ERROR(rc522_pcd_calculate_crc(rc522,
        &(rc522_bytes_t) {
            .ptr = cmd_buffer,
            .length = sizeof(cmd_buffer) - 2,
        },
        &crc));
    cmd_buffer[1 + RC522_NXP_PWD_SIZE] = crc.lsb;
    cmd_buffer[1 + RC522_NXP_PWD_SIZE + 1] = crc.msb;

    rc522_picc_transaction_t transaction = {
        .bytes = { .ptr = cmd_buffer, .length = sizeof(cmd_buffer) },
    };
    rc522_picc_transaction_result_t result = {
        .bytes = { .ptr = cmd_buffer, .length = sizeof(cmd_buffer) },
    };

    // If authentication fails (timeout or NAK), the card will HALT, so assume
    // it has until we know otherwise
    *out_state = RC522_PICC_STATE_HALT;
    RC522_RETURN_ON_ERROR(rc522_picc_transceive(rc522, &transaction, &result));
    RC522_RETURN_ON_ERROR(rc522_nxp_check_for_nak(&result));

    // Successful authentication, set state
    *out_state = RC522_PICC_STATE_AUTHENTICATED;
    // Validate the PACK - not critical to authenticate the card, but good
    // practice to verify
    RC522_CHECK_AND_RETURN(memcmp(pack, cmd_buffer, RC522_NXP_PACK_SIZE) != 0, RC522_ERR_NXP_PACK_MISMATCH);
    *out_state = RC522_PICC_STATE_AUTHENTICATED;
    return ESP_OK;
}

esp_err_t rc522_nxp_read_sig(const rc522_handle_t rc522, const rc522_picc_t *picc, rc522_nxp_sig_t *out_sig)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(out_sig == NULL);

    uint8_t sig_size = rc522_nxp_type_sig_size(picc->type);
    RC522_CHECK(sig_size == 0);
    RC522_CHECK(out_sig->buffer_size < sig_size);

    RC522_LOGD("NXP READ_SIG (size=%d)", sig_size);
    uint8_t cmd_buffer[4] = {
        RC522_NXP_READ_SIG,
        0,
        0,
        0 // CRC
    };

    rc522_pcd_crc_t crc = { 0 };
    RC522_RETURN_ON_ERROR(rc522_pcd_calculate_crc(rc522,
        &(rc522_bytes_t) {
            .ptr = cmd_buffer,
            .length = sizeof(cmd_buffer) - 2,
        },
        &crc));
    cmd_buffer[2] = crc.lsb;
    cmd_buffer[3] = crc.msb;

    // TODO: Combine to single malloc() for both buffers?
    // Again, I'd rather not do a large malloc() here, but we need the space for
    // CRC. See TODO in FAST_READ
    uint8_t *recv_buffer = malloc(sig_size + 2); // +2 for CRC
    RC522_CHECK_AND_RETURN(recv_buffer == NULL, ESP_ERR_NO_MEM);

    rc522_picc_transaction_t transaction = {
        .bytes = { .ptr = cmd_buffer, .length = sizeof(cmd_buffer) },
    };
    rc522_picc_transaction_result_t result = {
        .bytes = { .ptr = recv_buffer, .length = sig_size + 2 },
    };
    esp_err_t ret = rc522_picc_transceive(rc522, &transaction, &result);
    if (ret != ESP_OK) {
        free(recv_buffer);
        return ret;
    }
    ret = rc522_nxp_check_for_nak(&result);
    if (ret != ESP_OK) {
        free(recv_buffer);
        return ret;
    }

    memcpy(out_sig->bytes, recv_buffer, sig_size);
    out_sig->sig_size = sig_size;
    free(recv_buffer);
    return ESP_OK;
}
