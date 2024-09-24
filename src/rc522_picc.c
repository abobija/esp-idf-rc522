#include <esp_system.h>
#include <esp_check.h>
#include <string.h>

#include "rc522_internal.h"
#include "rc522_types_internal.h"
#include "rc522_helpers_internal.h"
#include "rc522_pcd_internal.h"
#include "rc522_picc_internal.h"

RC522_LOG_DEFINE_BASE();

struct rc522_picc_transaction_context
{
    const rc522_picc_transaction_t *transaction;
    uint8_t interrupts;
    bool completed;
    uint8_t error_reg;
};

esp_err_t rc522_picc_send(const rc522_handle_t rc522, const rc522_picc_transaction_t *transaction,
    rc522_picc_transaction_context_t *out_context)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(transaction == NULL);
    RC522_CHECK_BYTES(&transaction->bytes);
    RC522_CHECK(
        transaction->pcd_command != RC522_PCD_TRANSCEIVE_CMD && transaction->pcd_command != RC522_PCD_MF_AUTH_CMD);
    RC522_CHECK(transaction->expected_interrupts == 0);

    rc522_picc_transaction_context_t context = {
        .transaction = transaction,
    };

    // Prepare values for bit framing
    uint8_t bit_framing = (transaction->rx_align << 4) + transaction->valid_bits;

    if (RC522_LOG_LEVEL >= ESP_LOG_DEBUG) {
        RC522_LOGD("rx_align=%d,tx_last_bits=%d, bit_framing=%d",
            transaction->rx_align,
            transaction->valid_bits,
            bit_framing);

        char debug_buffer[64];
        rc522_buffer_to_hex_str(transaction->bytes.ptr, transaction->bytes.length, debug_buffer, sizeof(debug_buffer));
        RC522_LOGD("picc << %s", debug_buffer);
    }

    RC522_RETURN_ON_ERROR(rc522_pcd_stop_active_command(rc522));
    RC522_RETURN_ON_ERROR(rc522_pcd_clear_all_com_interrupts(rc522));
    RC522_RETURN_ON_ERROR(rc522_pcd_fifo_flush(rc522));
    RC522_RETURN_ON_ERROR(rc522_pcd_fifo_write(rc522, &transaction->bytes));
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_PCD_BIT_FRAMING_REG, bit_framing));
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_PCD_COMMAND_REG, transaction->pcd_command));

    if (transaction->pcd_command == RC522_PCD_TRANSCEIVE_CMD) {
        RC522_RETURN_ON_ERROR(rc522_pcd_start_data_transmission(rc522));
    }

    // TAuto flag in TModeReg is set.
    // This means the timer automatically starts when the PCD stops transmitting.

    const uint32_t deadline = rc522_millis() + 36;

    do {
        RC522_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_PCD_COM_INT_REQ_REG, &context.interrupts));

        if (context.interrupts & transaction->expected_interrupts) {
            context.completed = true;
            break;
        }

        // Timer interrupt - nothing received
        if (context.interrupts & RC522_PCD_TIMER_IRQ_BIT) {
            RC522_LOGD("timer interrupt (irq=0x%02" RC522_X ")", context.interrupts);

            return RC522_ERR_RX_TIMER_TIMEOUT;
        }

        taskYIELD();
    }
    while (rc522_millis() < deadline);

    // Deadline reached and nothing happened.
    // Communication with the MFRC522 might be down.
    RC522_RETURN_ON_FALSE(context.completed, RC522_ERR_RX_TIMEOUT);

    // Stop now if any errors except collisions were detected.
    RC522_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_PCD_ERROR_REG, &context.error_reg));

    if (context.error_reg & RC522_PCD_BUFFER_OVFL_BIT) {
        return RC522_ERR_PCD_FIFO_BUFFER_OVERFLOW;
    }
    else if (context.error_reg & RC522_PCD_PARITY_ERR_BIT) {
        RC522_LOGD("parity error detected");

        return RC522_ERR_PCD_PARITY_CHECK_FAILED;
    }
    else if (context.error_reg & RC522_PCD_PROTOCOL_ERR_BIT) {
        RC522_LOGD("protocol error detected");

        return RC522_ERR_PCD_PROTOCOL_ERROR;
    }

    if (out_context) {
        memcpy(out_context, &context, sizeof(context));
    }

    return ESP_OK;
}

static esp_err_t rc522_picc_receive(const rc522_handle_t rc522, const rc522_picc_transaction_context_t *context,
    rc522_picc_transaction_result_t *out_result)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(context == NULL);
    RC522_CHECK(context->transaction == NULL);
    RC522_CHECK(out_result == NULL);
    RC522_CHECK_BYTES(&context->transaction->bytes);

    uint8_t fifo_level = 0;
    RC522_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_PCD_FIFO_LEVEL_REG, &fifo_level));

    if (fifo_level < 1) {
        RC522_LOGW("fifo empty (irq=0x%02" RC522_X ")", context->interrupts);

        return RC522_ERR_PCD_FIFO_EMPTY;
    }

    RC522_CHECK(fifo_level > out_result->bytes.length);

    rc522_picc_transaction_result_t result = {
        .bytes = { 
            .ptr = out_result->bytes.ptr, // Use buffer provided by caller
            .length = fifo_level,
        },
    };

    RC522_RETURN_ON_ERROR(rc522_pcd_fifo_read(rc522, &result.bytes));

    if (RC522_LOG_LEVEL >= ESP_LOG_DEBUG) {
        char debug_buffer[64];
        rc522_buffer_to_hex_str(result.bytes.ptr, result.bytes.length, debug_buffer, sizeof(debug_buffer));
        RC522_LOGD("picc >> %s", debug_buffer);
    }

    if (context->transaction->rx_align) {
        RC522_LOGD("applying mask (rx_align=%d)", context->transaction->rx_align);

        // Apply mask for rx_align..7 of the first byte
        result.bytes.ptr[0] &= (0xFF << context->transaction->rx_align);
    }

    // RxLastBits[2:0] indicates the number of valid bits in the last received byte.
    // If this value is 0, the whole byte is valid.
    RC522_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_PCD_CONTROL_REG, &result.valid_bits));
    result.valid_bits &= 0x07;

    if (result.valid_bits) {
        RC522_LOGD("not full byte received, valid_bits=%d", result.valid_bits);
    }

    if (context->error_reg & RC522_PCD_COLL_ERR_BIT) {
        return RC522_ERR_COLLISION;
    }

    // Perform CRC_A validation
    if (context->transaction->check_crc) {
        // We need at least the CRC_A value
        // and all 8 bits of the last byte
        RC522_CHECK_AND_RETURN(result.bytes.length < 3, ESP_ERR_INVALID_STATE);
        RC522_CHECK_AND_RETURN(result.valid_bits != 0, ESP_ERR_INVALID_STATE);

        // Verify CRC_A
        rc522_pcd_crc_t crc = { 0 };
        RC522_RETURN_ON_ERROR(rc522_pcd_calculate_crc(rc522,
            &(rc522_bytes_t) { .ptr = result.bytes.ptr, .length = result.bytes.length - 2 },
            &crc));

        if (memcmp(result.bytes.ptr + result.bytes.length - 2, &crc, sizeof(crc)) != 0) {
            return RC522_ERR_CRC_WRONG;
        }
    }

    memcpy(out_result, &result, sizeof(result));

    return ESP_OK;
}

esp_err_t rc522_picc_transceive(const rc522_handle_t rc522, const rc522_picc_transaction_t *transaction,
    rc522_picc_transaction_result_t *out_result)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(transaction == NULL);

    rc522_picc_transaction_t transaction_clone = { 0 };
    memcpy(&transaction_clone, transaction, sizeof(transaction_clone));

    transaction_clone.pcd_command = RC522_PCD_TRANSCEIVE_CMD;
    transaction_clone.expected_interrupts = RC522_PCD_RX_IRQ_BIT | RC522_PCD_IDLE_IRQ_BIT;

    rc522_picc_transaction_context_t context = { 0 };
    RC522_RETURN_ON_ERROR_SILENTLY(rc522_picc_send(rc522, &transaction_clone, &context));

    if (out_result) {
        RC522_RETURN_ON_ERROR(rc522_picc_receive(rc522, &context, out_result));
    }

    return ESP_OK;
}

inline static esp_err_t rc522_picc_parse_atqa(uint16_t atqa, rc522_picc_atqa_desc_t *out_atqa)
{
    RC522_CHECK(out_atqa == NULL);

    out_atqa->source = atqa;
    out_atqa->rfu4 = (atqa >> 12) & 0x0F;
    out_atqa->prop_coding = (atqa >> 8) & 0x0F;
    out_atqa->uid_size = (atqa >> 6) & 0x03;
    out_atqa->rfu1 = (atqa >> 5) & 0x01;
    out_atqa->anticollision = atqa & 0x1F;

    return ESP_OK;
}

static esp_err_t rc522_picc_reqa_or_wupa(const rc522_handle_t rc522, uint8_t picc_cmd, rc522_picc_atqa_desc_t *out_atqa)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc_cmd != RC522_PICC_CMD_REQA && picc_cmd != RC522_PICC_CMD_WUPA);
    RC522_CHECK(out_atqa == NULL);

    RC522_RETURN_ON_ERROR(rc522_pcd_clear_bits(rc522, RC522_PCD_COLL_REG, RC522_PCD_VALUES_AFTER_COLL_BIT));

    uint8_t buffer[2] = { 0 };

    rc522_picc_transaction_t transaction = {
        .bytes = { .ptr = &picc_cmd, .length = 1 },
        .valid_bits = 7, // REQA and WUPA use short frame format
    };

    rc522_picc_transaction_result_t transaction_result = {
        .bytes = { .ptr = buffer, .length = sizeof(buffer) },
    };

    esp_err_t ret = rc522_picc_transceive(rc522, &transaction, &transaction_result);

    if (ret != ESP_OK) {
        // Timeouts are expected if no PICC are in the field, log other errors
        if (ret != RC522_ERR_RX_TIMER_TIMEOUT && ret != RC522_ERR_RX_TIMEOUT) {
            RC522_LOGD("non-timeout error: %04" RC522_X, ret);
        }

        return ret;
    }

    if (transaction_result.bytes.length != 2 || transaction_result.valid_bits != 0) {
        return RC522_ERR_INVALID_ATQA;
    }

    uint16_t atqa = (buffer[0] << 8) | buffer[1];
    RC522_RETURN_ON_ERROR(rc522_picc_parse_atqa(atqa, out_atqa));

    return ESP_OK;
}

inline esp_err_t rc522_picc_reqa(const rc522_handle_t rc522, rc522_picc_atqa_desc_t *out_atqa)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(out_atqa == NULL);

    RC522_LOGD("REQA");
    return rc522_picc_reqa_or_wupa(rc522, RC522_PICC_CMD_REQA, out_atqa);
}

inline esp_err_t rc522_picc_wupa(const rc522_handle_t rc522, rc522_picc_atqa_desc_t *out_atqa)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(out_atqa == NULL);

    RC522_LOGD("WUPA");
    return rc522_picc_reqa_or_wupa(rc522, RC522_PICC_CMD_WUPA, out_atqa);
}

/**
 * Resolve collision and SELECT a PICC
 */
esp_err_t rc522_picc_select(const rc522_handle_t rc522, rc522_picc_uid_t *out_uid, uint8_t *out_sak, bool skip_anticoll)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(skip_anticoll && (out_uid == NULL || out_uid->length < RC522_PICC_UID_SIZE_MIN));

    bool uid_complete;
    bool select_done;
    bool use_cascade_tag;
    uint8_t cascade_level = 1;
    esp_err_t ret;
    uint8_t count;
    uint8_t check_bit;
    uint8_t index;
    uint8_t uid_index;               // The first index in uid->uidByte[] that is used in the current Cascade Level.
    int8_t current_level_known_bits; // The number of known UID bits in the current Cascade Level.
    uint8_t buffer[9];               // The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
    uint8_t buffer_used;  // The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
    uint8_t rx_align;     // Used in BitFramingReg. Defines the bit position for the first bit received.
    uint8_t tx_last_bits; // Used in BitFramingReg. The number of valid bits in the last transmitted byte.
    uint8_t *response_buffer;
    uint8_t response_length;

    rc522_picc_uid_t uid;

    if (skip_anticoll) {
        memcpy(&uid, out_uid, sizeof(rc522_picc_uid_t));
    }
    else {
        memset(&uid, 0, sizeof(uid));
    }

    uint8_t sak;

    // Description of buffer structure:
    //	 Byte 0: SEL              Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
    //   Byte 1: NVB              Number of Valid Bits (in complete command, not just the UID):
    //                            High nibble: complete bytes, Low nibble: Extra bits.
    //   Byte 2: UID-data or CT	  See explanation below. CT means Cascade Tag.
    //   Byte 3: UID-data
    //   Byte 4: UID-data
    //   Byte 5: UID-data
    //   Byte 6: BCC              Block Check Character - XOR of bytes 2-5
    //   Byte 7: CRC_A
    //   Byte 8: CRC_A
    //
    //   The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
    //
    // Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
    //		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
    //		========	=============	=====	=====	=====	=====
    //		 4 bytes		1			uid0	uid1	uid2	uid3
    //		 7 bytes		1			CT		uid0	uid1	uid2
    //						2			uid3	uid4	uid5	uid6
    //		10 bytes		1			CT		uid0	uid1	uid2
    //						2			CT		uid3	uid4	uid5
    //						3			uid6	uid7	uid8	uid9

    // Prepare MFRC522
    RC522_RETURN_ON_ERROR(rc522_pcd_clear_bits(rc522, RC522_PCD_COLL_REG, RC522_PCD_VALUES_AFTER_COLL_BIT));

    // Repeat Cascade Level loop until we have a complete UID.
    uid_complete = false;
    while (!uid_complete) {
        RC522_LOGD("cascade_level=%d, uid.length=%d", cascade_level, uid.length);

        // Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
        switch (cascade_level) {
            case 1:
                buffer[0] = RC522_PICC_CMD_SEL_CL1;
                uid_index = 0;

                // When we know that the UID has more than 4 bytes
                use_cascade_tag = uid.length > 4;
                break;

            case 2:
                buffer[0] = RC522_PICC_CMD_SEL_CL2;
                uid_index = 3;

                // When we know that the UID has more than 7 bytes
                use_cascade_tag = uid.length > 7;
                break;

            case 3:
                buffer[0] = RC522_PICC_CMD_SEL_CL3;
                uid_index = 6;
                use_cascade_tag = false; // Never used in CL3.
                break;

            default:
                return ESP_FAIL; // TODO: use custom err
                break;
        }

        RC522_LOGD("cl=%d, uid_index=%d, use_cascade_tag=%d", cascade_level, uid_index, use_cascade_tag);

        // How many UID bits are known in this Cascade Level?
        if (skip_anticoll) {
            current_level_known_bits = (4 * 8);
        }
        else {
            current_level_known_bits = (8 * uid_index);
            if (current_level_known_bits < 0) {
                current_level_known_bits = 0;
            }
        }
        // Copy the known bits from uid.uidByte[] to buffer[]
        index = 2; // destination index in buffer[]
        if (use_cascade_tag) {
            buffer[index++] = RC522_PICC_CMD_CT;
        }

        // The number of bytes needed to represent the known bits for this level.
        uint8_t bytes_to_copy = current_level_known_bits / 8 + (current_level_known_bits % 8 ? 1 : 0);

        if (bytes_to_copy) {
            // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
            uint8_t max_bytes = use_cascade_tag ? 3 : 4;
            if (bytes_to_copy > max_bytes) {
                bytes_to_copy = max_bytes;
            }
            for (count = 0; count < bytes_to_copy; count++) {
                buffer[index++] = uid.value[uid_index + count];
            }
        }
        // Now that the data has been copied we need to include the 8 bits in CT in current_level_known_bits
        if (use_cascade_tag) {
            current_level_known_bits += 8;
        }

        // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
        select_done = false;
        while (!select_done) {
            // Find out how many bits and bytes to send and receive.
            if (current_level_known_bits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
                RC522_LOGD("SELECT (cl=%d)", cascade_level);

                // NVB - Number of Valid Bits: Seven whole bytes
                buffer[1] = 0x70;
                // Calculate BCC - Block Check Character
                buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
                // Calculate CRC_A

                rc522_pcd_crc_t crc = { 0 };
                RC522_RETURN_ON_ERROR(
                    rc522_pcd_calculate_crc(rc522, &(rc522_bytes_t) { .ptr = buffer, .length = 7 }, &crc));

                buffer[7] = crc.lsb;
                buffer[8] = crc.msb;

                tx_last_bits = 0; // 0 => All 8 bits are valid.
                buffer_used = 9;
                // Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
                response_buffer = &buffer[6];
                response_length = 3;
            }
            else { // This is an ANTICOLLISION.
                RC522_LOGD("ANTICOLLISION (cl=%d)", cascade_level);

                tx_last_bits = current_level_known_bits % 8;
                count = current_level_known_bits / 8;    // Number of whole bytes in the UID part.
                index = 2 + count;                       // Number of whole bytes: SEL + NVB + UIDs
                buffer[1] = (index << 4) + tx_last_bits; // NVB - Number of Valid Bits
                buffer_used = index + (tx_last_bits ? 1 : 0);
                // Store response in the unused part of buffer
                response_buffer = &buffer[index];
                response_length = sizeof(buffer) - index;
            }

            // Set bit adjustments
            // Having a separate variable is overkill. But it makes the next line easier to read.
            rx_align = tx_last_bits;

            // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
            RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_PCD_BIT_FRAMING_REG, (rx_align << 4) + tx_last_bits));

            // Transmit the buffer and receive the response.
            rc522_picc_transaction_t transaction = {
                .bytes = { .ptr = buffer, .length = buffer_used },
                .rx_align = rx_align,
                .valid_bits = tx_last_bits,
            };

            rc522_picc_transaction_result_t transaction_result = {
                .bytes = { .ptr = response_buffer, .length = response_length },
            };

            ret = rc522_picc_transceive(rc522, &transaction, &transaction_result);

            if (ret == ESP_OK) {
                response_length = transaction_result.bytes.length;
                tx_last_bits = transaction_result.valid_bits;
            }

            if (ret == RC522_ERR_COLLISION) { // More than one PICC in the field => collision.
                RC522_LOGD("collision detected (cl=%d, skip_anticoll=%d)", cascade_level, skip_anticoll);

                if (skip_anticoll) {
                    // If we are skipping anticoll, we should not have collisions
                    RC522_LOGD("unexpected collision detected");

                    return ret;
                }

                // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
                uint8_t value_of_coll_reg;
                rc522_pcd_read(rc522, RC522_PCD_COLL_REG, &value_of_coll_reg);

                if (value_of_coll_reg & RC522_PCD_COLL_POS_NOT_VALID_BIT) {
                    // Without a valid collision position we cannot continue
                    RC522_LOGD("collision position not valid, coll_poss[4:0] out of range");

                    return RC522_ERR_COLLISION_UNSOLVABLE;
                }

                uint8_t collision_pos = value_of_coll_reg & 0x1F; // Values 0-31, 0 means bit 32.
                if (collision_pos == 0) {
                    collision_pos = 32;
                }
                if (collision_pos <= current_level_known_bits) { // No progress - should not happen
                    RC522_LOGD("collision_pos (%d) <= current_level_known_bits (%d)",
                        collision_pos,
                        current_level_known_bits);

                    return RC522_ERR_COLLISION_UNSOLVABLE;
                }
                // Choose the PICC with the bit set.
                current_level_known_bits = collision_pos;
                count = current_level_known_bits % 8; // The bit to modify
                check_bit = (current_level_known_bits - 1) % 8;
                index = 1 + (current_level_known_bits / 8) + (count ? 1 : 0); // First byte is index 0.
                buffer[index] |= (1 << check_bit);
            }
            else if (ret != ESP_OK) {
                RC522_LOGD("transceive failed");

                return ret;
            }
            else {                                    // ESP_OK
                if (current_level_known_bits >= 32) { // This was a SELECT.
                    // No more anticollision
                    // We continue below outside the while.
                    select_done = true;
                }
                else { // This was an ANTICOLLISION.
                    // We now have all 32 bits of the UID in this Cascade Level
                    current_level_known_bits = 32;
                    // Run loop again to do the SELECT.
                }
            }
        } // End of while (!selectDone)

        RC522_LOGD("SELECT (cl=%d) done", cascade_level);

        // We do not check the CBB - it was constructed by us above.

        // Copy the found UID bytes from buffer[] to uid.uidByte[]
        index = (buffer[2] == RC522_PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
        bytes_to_copy = (buffer[2] == RC522_PICC_CMD_CT) ? 3 : 4;
        for (count = 0; count < bytes_to_copy; count++) {
            uid.value[uid_index + count] = buffer[index++];
        }

        // Check response SAK (Select Acknowledge)
        if (response_length != 3 || tx_last_bits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
            RC522_LOGD("invalid sak");
            return RC522_ERR_INVALID_SAK;
        }
        // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed
        // anymore.

// compiler complains about uninitialized response_buffer even is
// no chance that response_buffer is NULL here, so ignore warning here
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
        rc522_pcd_crc_t crc = { 0 };
        RC522_RETURN_ON_ERROR(
            rc522_pcd_calculate_crc(rc522, &(rc522_bytes_t) { .ptr = response_buffer, .length = 1 }, &crc));

        if (memcmp(response_buffer + 1, &crc, sizeof(crc)) != 0) {
            RC522_LOGD("crc wrong");
            return RC522_ERR_CRC_WRONG;
        }

        buffer[2] = crc.lsb;
        buffer[3] = crc.msb;

        if (response_buffer[0] & 0x04) { // Cascade bit set - UID not complete yes
            cascade_level++;
        }
        else {
            uid_complete = true;
            sak = response_buffer[0];
        }
#pragma GCC diagnostic pop
    } // End of while (!uidComplete)

    // Set correct uid.size
    uid.length = 3 * cascade_level + 1;

    RC522_LOGD("sak=0x%02" RC522_X, sak);

    if (out_uid) {
        memcpy(out_uid, &uid, sizeof(uid));
    }

    if (out_sak) {
        *out_sak = sak;
    }

    return ESP_OK;
}

/**
 * Checks if PICC is still in the PCD field
 */
esp_err_t rc522_picc_heartbeat(
    const rc522_handle_t rc522, const rc522_picc_t *picc, rc522_picc_uid_t *out_uid, uint8_t *out_sak)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);
    RC522_CHECK(picc->state != RC522_PICC_STATE_ACTIVE && picc->state != RC522_PICC_STATE_ACTIVE_H);

    esp_err_t ret = ESP_OK;
    const uint8_t retries = 5;
    uint8_t retry = 1;

    do {
        rc522_picc_atqa_desc_t atqa;

        if (retry <= 2) {
            if (picc->state == RC522_PICC_STATE_ACTIVE) {
                ret = rc522_picc_reqa(rc522, &atqa);
            }
            else if (picc->state == RC522_PICC_STATE_ACTIVE_H) {
                ret = rc522_picc_wupa(rc522, &atqa);
            }
        }
        else {
            if ((ret = rc522_picc_reqa(rc522, &atqa)) != ESP_OK) {
                ret = rc522_picc_wupa(rc522, &atqa);
            }
        }

        if (ret == ESP_OK) {
            break;
        }

        rc522_delay_ms(3);
        taskYIELD();
    }
    while (retry++ < retries);

    if (ret != ESP_OK) {
        return ret;
    }

    rc522_picc_uid_t uid;
    uint8_t sak;

    memcpy(&uid, &picc->uid, sizeof(rc522_picc_uid_t));

    ret = rc522_picc_select(rc522, &uid, &sak, true);

    if (ret != ESP_OK) {
        return ret;
    }

    if (picc->sak != sak) {
        return RC522_ERR_PICC_POST_HEARTBEAT_MISSMATCH;
    }

    for (uint8_t i = 0; i < uid.length; i++) {
        if (picc->uid.value[i] != uid.value[i]) {
            return RC522_ERR_PICC_POST_HEARTBEAT_MISSMATCH;
        }
    }

    if (out_uid) {
        memcpy(out_uid, &uid, sizeof(uid));
    }

    if (out_sak) {
        *out_sak = sak;
    }

    return ESP_OK;
}

esp_err_t rc522_picc_uid_to_str(const rc522_picc_uid_t *uid, char *buffer, uint8_t buffer_size)
{
    RC522_CHECK(uid == NULL);
    RC522_CHECK(buffer == NULL);
    RC522_CHECK(buffer_size < RC522_PICC_UID_STR_BUFFER_SIZE_MAX);

    return rc522_buffer_to_hex_str(uid->value, uid->length, buffer, buffer_size);
}

esp_err_t rc522_picc_halta(const rc522_handle_t rc522, rc522_picc_t *picc)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);

    RC522_LOGD("HALTA");

    uint8_t buffer[4] = { 0 };

    buffer[0] = RC522_PICC_CMD_HLTA;
    buffer[1] = 0;

    rc522_pcd_crc_t crc = { 0 };
    RC522_RETURN_ON_ERROR(rc522_pcd_calculate_crc(rc522, &(rc522_bytes_t) { .ptr = buffer, .length = 2 }, &crc));

    buffer[2] = crc.lsb;
    buffer[3] = crc.msb;

    rc522_picc_transaction_t transaction = {
        .bytes = { .ptr = buffer, .length = sizeof(buffer) },
    };

    esp_err_t ret = rc522_picc_transceive(rc522, &transaction, NULL);

    // If the PICC responds with any modulation during a period of 1 ms after the HLTA,
    // response shall be interpreted as 'not acknowledge', so timeout is not an error.
    if (ret == ESP_OK) {
        return RC522_ERR_HLTA_NOT_ACKED;
    }
    else if (ret == RC522_ERR_RX_TIMER_TIMEOUT || ret == RC522_ERR_RX_TIMEOUT) {
        RC522_RETURN_ON_ERROR(rc522_picc_set_state(rc522, picc, RC522_PICC_STATE_HALT, true));

        return ESP_OK;
    }

    return ret;
}

esp_err_t rc522_picc_set_state(
    const rc522_handle_t rc522, rc522_picc_t *picc, rc522_picc_state_t new_state, bool fire_event)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(picc == NULL);

    esp_err_t ret = ESP_OK;

    if (picc->state == new_state) {
        return ESP_OK;
    }

    RC522_LOGD("changing state from %d to %d (fire=%d)", picc->state, new_state, fire_event);

    rc522_picc_state_t old_state = picc->state;

    picc->state = new_state;

    if (fire_event) {
        rc522_picc_state_changed_event_t event_data = {
            .old_state = old_state,
            .picc = picc,
        };

        if ((ret = rc522_dispatch_event(rc522, RC522_EVENT_PICC_STATE_CHANGED, &event_data, sizeof(event_data)))
            != ESP_OK) {
            RC522_LOGW("picc_state_changed event dispatch failed (err=%04" RC522_X ")", ret);
        }
    }

    return ret;
}

rc522_picc_type_t rc522_picc_get_type(const rc522_picc_t *picc)
{
    RC522_CHECK(picc == NULL);

    uint8_t sak = picc->sak;

    // http://www.nxp.com/documents/application_note/AN10833.pdf
    // Section: Coding of Select Acknowledge (SAK)

    // ignore 8th (iso14443 starts with LSBit = bit 1)
    // fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A)
    sak &= 0x7F;

    switch (sak) {
        case 0x09:
            return RC522_PICC_TYPE_MIFARE_MINI;
        case 0x08:
            return RC522_PICC_TYPE_MIFARE_1K;
        case 0x18:
            return RC522_PICC_TYPE_MIFARE_4K;
        case 0x00:
            return RC522_PICC_TYPE_MIFARE_UL;
        case 0x10:
        case 0x11:
            return RC522_PICC_TYPE_MIFARE_PLUS;
        case 0x01:
            return RC522_PICC_TYPE_TNP3XXX;
        case 0x20:
            return picc->atqa.source == 0x4400 ? RC522_PICC_TYPE_MIFARE_DESFIRE : RC522_PICC_TYPE_ISO_14443_4;
        case 0x40:
            return RC522_PICC_TYPE_ISO_18092;
        default:
            return RC522_PICC_TYPE_UNKNOWN;
    }
}

char *rc522_picc_type_name(rc522_picc_type_t type)
{
    switch (type) {
        case RC522_PICC_TYPE_ISO_14443_4:
            return "PICC compliant with ISO/IEC 14443-4";
        case RC522_PICC_TYPE_ISO_18092:
            return "PICC compliant with ISO/IEC 18092 (NFC)";
        case RC522_PICC_TYPE_MIFARE_MINI:
            return "MIFARE Mini, 320 bytes";
        case RC522_PICC_TYPE_MIFARE_1K:
            return "MIFARE 1K";
        case RC522_PICC_TYPE_MIFARE_4K:
            return "MIFARE 4K";
        case RC522_PICC_TYPE_MIFARE_UL:
            return "MIFARE Ultralight or Ultralight C";
        case RC522_PICC_TYPE_MIFARE_PLUS:
            return "MIFARE Plus";
        case RC522_PICC_TYPE_MIFARE_DESFIRE:
            return "MIFARE DESFire";
        case RC522_PICC_TYPE_TNP3XXX:
            return "MIFARE TNP3XXX";
        case RC522_PICC_TYPE_UNDEFINED:
        case RC522_PICC_TYPE_UNKNOWN:
        default:
            return "unknown";
    }
}

esp_err_t rc522_picc_print(const rc522_picc_t *picc)
{
    RC522_CHECK(picc == NULL);

    char uid_str[RC522_PICC_UID_STR_BUFFER_SIZE_MAX];
    RC522_RETURN_ON_ERROR(rc522_picc_uid_to_str(&picc->uid, uid_str, sizeof(uid_str)));

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔══════════════╗");
    ESP_LOGI(TAG, "║              ║ Type: %s", rc522_picc_type_name(picc->type));
    ESP_LOGI(TAG, "║     RFID     ║ UID:  %s", uid_str);
    ESP_LOGI(TAG, "║     CARD     ║ ATQA: 0x%04" RC522_X, picc->atqa.source);
    ESP_LOGI(TAG, "║              ║ SAK:  0x%02" RC522_X, picc->sak);
    ESP_LOGI(TAG, "╚══════════════╝");
    ESP_LOGI(TAG, "");

    return ESP_OK;
}
