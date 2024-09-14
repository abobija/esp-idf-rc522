#include <esp_system.h>
#include <esp_check.h>
#include <string.h>

#include "rc522_types_private.h"
#include "rc522_helpers_private.h"
#include "rc522_pcd_private.h"
#include "rc522_picc_private.h"

RC522_LOG_DEFINE_BASE();

esp_err_t rc522_picc_comm(rc522_handle_t rc522, rc522_pcd_command_t command, uint8_t wait_irq, uint8_t *send_data,
    uint8_t send_data_len, uint8_t *back_data, uint8_t *back_data_len, uint8_t *valid_bits, uint8_t rx_align,
    bool check_crc)
{
    if (RC522_LOG_LEVEL >= ESP_LOG_DEBUG) {
        RC522_LOGD("transceive (rx_align=0x%02x, check_crc=%d)", rx_align, check_crc);
        char debug_buffer[64];
        rc522_buffer_to_hex_str(send_data, send_data_len, debug_buffer, sizeof(debug_buffer));
        RC522_LOGD("picc << %s", debug_buffer);
    }

    // Prepare values for bit framing
    uint8_t tx_last_bits = valid_bits ? *valid_bits : 0;
    uint8_t bit_framing = (rx_align << 4) + tx_last_bits;

    RC522_RETURN_ON_ERROR(rc522_pcd_stop_active_command(rc522));
    RC522_RETURN_ON_ERROR(
        rc522_pcd_write(rc522, RC522_PCD_COMM_INT_REQ_REG, 0x7F)); // Clear all seven interrupt request bits
    RC522_RETURN_ON_ERROR(rc522_pcd_fifo_flush(rc522));
    RC522_RETURN_ON_ERROR(rc522_pcd_fifo_write(rc522, send_data, send_data_len));
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_PCD_BIT_FRAMING_REG, bit_framing)); // Bit adjustments
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_PCD_COMMAND_REG, command));         // Execute the command

    if (command == RC522_PCD_TRANSCEIVE_CMD) {
        RC522_RETURN_ON_ERROR(rc522_pcd_start_data_transmission(rc522));
    }

    // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer
    // automatically starts when the PCD stops transmitting.
    //
    // Wait here for the command to complete. The bits specified in the
    // `waitIRq` parameter define what bits constitute a completed command.
    // When they are set in the ComIrqReg register, then the command is
    // considered complete. If the command is not indicated as complete in
    // ~36ms, then consider the command as timed out.
    const uint32_t deadline = rc522_millis() + 36;
    bool completed = false;

    do {
        uint8_t irq;
        RC522_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_PCD_COMM_INT_REQ_REG, &irq));

        if (irq & wait_irq) { // One of the interrupts that signal success has been set.
            completed = true;
            break;
        }

        // Timer interrupt - nothing received in 25ms
        if (irq & RC522_PCD_TIMER_IRQ_BIT) {
            RC522_LOGD("timer interrupt (irq=0x%02x)", irq);

            return ESP_ERR_TIMEOUT;
        }

        taskYIELD();
    }
    while (rc522_millis() < deadline);

    // 36ms and nothing happened. Communication with the MFRC522 might be down.
    RC522_RETURN_ON_FALSE(completed, ESP_ERR_TIMEOUT);

    // Stop now if any errors except collisions were detected.
    uint8_t error_reg_value;
    RC522_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_PCD_ERROR_REG, &error_reg_value));

    if (error_reg_value & 0x13) { // BufferOvfl ParityErr ProtocolErr
        return ESP_FAIL;
    }

    uint8_t _valid_bits = 0;

    // If the caller wants data back, get it from the MFRC522.
    if (back_data && back_data_len) {
        uint8_t length;
        RC522_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_PCD_FIFO_LEVEL_REG, &length));

        if (length > *back_data_len) {
            return ESP_ERR_NO_MEM;
        }

        *back_data_len = length; // Number of bytes returned

        uint8_t b0_orig = back_data[0];
        RC522_RETURN_ON_ERROR(rc522_pcd_fifo_read(rc522, back_data, length));

        if (rx_align) { // Only update bit positions rxAlign..7 in values[0]
            RC522_LOGD("rx_align=0x%02x, applying mask", rx_align);

            // Create bit mask for bit positions rxAlign..7
            uint8_t mask = (0xFF << rx_align) & 0xFF;
            // Apply mask to both current value of values[0] and the new data in value.
            back_data[0] = (b0_orig & ~mask) | (back_data[0] & mask);
        }

        // RxLastBits[2:0] indicates the number of valid bits in the last
        // received byte. If this value is 000b, the whole byte is valid.
        RC522_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_PCD_CONTROL_REG, &_valid_bits));
        _valid_bits &= 0x07;

        if (valid_bits) {
            *valid_bits = _valid_bits;
        }
    }

    // Tell about collisions
    if (error_reg_value & 0x08) { // CollErr
        return ESP_ERR_RC522_COLLISION;
    }

    // Perform CRC_A validation if requested.
    if (check_crc && back_data && back_data_len) {
        // In this case a MIFARE Classic NAK is not OK.
        if (*back_data_len == 1 && _valid_bits == 4) {
            return ESP_ERR_RC522_MIFARE_NACK;
        }
        // We need at least the CRC_A value and all 8 bits of the last byte must be received.
        if (*back_data_len < 2 || _valid_bits != 0) {
            return ESP_ERR_RC522_CRC_WRONG;
        }
        // Verify CRC_A - do our own calculation and store the control in controlBuffer.
        uint8_t control_buffer[2];

        RC522_RETURN_ON_ERROR(rc522_pcd_calculate_crc(rc522, back_data, *back_data_len - 2, control_buffer));

        if ((back_data[*back_data_len - 2] != control_buffer[0]) ||
            (back_data[*back_data_len - 1] != control_buffer[1])) {
            return ESP_ERR_RC522_CRC_WRONG;
        }
    }

    if (RC522_LOG_LEVEL >= ESP_LOG_DEBUG) {
        if (back_data && back_data_len) {
            char debug_buffer[64];
            rc522_buffer_to_hex_str(back_data, *back_data_len, debug_buffer, sizeof(debug_buffer));
            RC522_LOGD("picc >> %s", debug_buffer);
        }
        else {
            // TODO: Read from PICC above even if back_data is null so we can log.
            //        But then we would need local buffer. Hmm...

            RC522_LOGW("Cannot log rx data since back_data buffer is null");
        }
    }

    return ESP_OK;
}

esp_err_t rc522_picc_transceive(rc522_handle_t rc522, uint8_t *send_data, uint8_t send_data_len, uint8_t *back_data,
    uint8_t *back_data_len, uint8_t *valid_bits, uint8_t rx_align, bool check_crc)
{
    uint8_t wait_irq = 0x30; // RxIRq and IdleIRq

    return rc522_picc_comm(rc522,
        RC522_PCD_TRANSCEIVE_CMD,
        wait_irq,
        send_data,
        send_data_len,
        back_data,
        back_data_len,
        valid_bits,
        rx_align,
        check_crc);
}

static esp_err_t rc522_picc_reqa_or_wupa(
    rc522_handle_t rc522, uint8_t picc_cmd, uint8_t *atqa_buffer, uint8_t *atqa_buffer_size)
{
    uint8_t valid_bits;

    if (atqa_buffer == NULL || *atqa_buffer_size < 2) { // The ATQA response is 2 bytes long.
        return ESP_ERR_NO_MEM;
    }

    // ValuesAfterColl=1 => Bits received after collision are cleared.
    RC522_RETURN_ON_ERROR(rc522_pcd_clear_bits(rc522, RC522_PCD_COLL_REG, 0x80));

    // For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only)
    // byte. TxLastBits = BitFramingReg[2..0]
    valid_bits = 7;

    RC522_RETURN_ON_ERROR_SILENTLY(
        rc522_picc_transceive(rc522, &picc_cmd, 1, atqa_buffer, atqa_buffer_size, &valid_bits, 0, false));

    if (*atqa_buffer_size != 2 || valid_bits != 0) { // ATQA must be exactly 16 bits.
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t rc522_picc_reqa(rc522_handle_t rc522, uint8_t *atqa_buffer, uint8_t *atqa_buffer_size)
{
    return rc522_picc_reqa_or_wupa(rc522, RC522_PICC_CMD_REQA, atqa_buffer, atqa_buffer_size);
}

/**
 * Check if PICC is in the PCD field
 */
esp_err_t rc522_picc_find(rc522_handle_t rc522, rc522_picc_t *picc)
{
    ESP_RETURN_ON_FALSE(picc != NULL, ESP_ERR_INVALID_ARG, TAG, "picc is null");

    uint8_t atqa_buffer[2];
    uint8_t atqa_buffer_size = sizeof(atqa_buffer);

    // Reset baud rates
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_PCD_TX_MODE_REG, RC522_PCD_TX_MODE_RESET_VALUE));
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_PCD_RX_MODE_REG, RC522_PCD_RX_MODE_RESET_VALUE));
    // Reset ModWidthReg
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_PCD_MOD_WIDTH_REG, RC522_PCD_MOD_WIDTH_RESET_VALUE));

    esp_err_t ret = rc522_picc_reqa(rc522, atqa_buffer, &atqa_buffer_size);

    picc->is_present = (ret == ESP_OK || ret == ESP_ERR_RC522_COLLISION);

    return ret;
}

static esp_err_t rc522_picc_select(rc522_handle_t rc522, rc522_picc_t *picc, uint8_t valid_bits)
{
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

    // Sanity checks
    const uint8_t valid_bits_max = 80;
    ESP_RETURN_ON_FALSE(valid_bits <= valid_bits_max, ESP_ERR_INVALID_ARG, TAG, "valid_bits > %d", valid_bits_max);

    // Prepare MFRC522
    // ValuesAfterColl=1 => Bits received after collision are cleared.
    RC522_RETURN_ON_ERROR(rc522_pcd_clear_bits(rc522, RC522_PCD_COLL_REG, 0x80));

    // Repeat Cascade Level loop until we have a complete UID.
    uid_complete = false;
    while (!uid_complete) {
        RC522_LOGD("cascade_level=%d, valid_bits=%d, uid->bytes_length=%d",
            cascade_level,
            valid_bits,
            picc->uid.bytes_length);

        // Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
        switch (cascade_level) {
            case 1:
                buffer[0] = RC522_PICC_CMD_SEL_CL1;
                uid_index = 0;

                // When we know that the UID has more than 4 bytes
                use_cascade_tag = valid_bits && picc->uid.bytes_length > 4;
                break;

            case 2:
                buffer[0] = RC522_PICC_CMD_SEL_CL2;
                uid_index = 3;

                // When we know that the UID has more than 7 bytes
                use_cascade_tag = valid_bits && picc->uid.bytes_length > 7;
                break;

            case 3:
                buffer[0] = RC522_PICC_CMD_SEL_CL3;
                uid_index = 6;
                use_cascade_tag = false; // Never used in CL3.
                break;

            default:
                return ESP_FAIL;
                break;
        }

        RC522_LOGD("uid_index=%d, use_cascade_tag=%d", uid_index, use_cascade_tag);

        // How many UID bits are known in this Cascade Level?
        current_level_known_bits = valid_bits - (8 * uid_index);
        if (current_level_known_bits < 0) {
            current_level_known_bits = 0;
        }
        // Copy the known bits from uid->uidByte[] to buffer[]
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
                buffer[index++] = picc->uid.bytes[uid_index + count];
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
                RC522_LOGD("SELECT");

                // NVB - Number of Valid Bits: Seven whole bytes
                buffer[1] = 0x70;
                // Calculate BCC - Block Check Character
                buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
                // Calculate CRC_A

                RC522_RETURN_ON_ERROR(rc522_pcd_calculate_crc(rc522, buffer, 7, &buffer[7]));

                tx_last_bits = 0; // 0 => All 8 bits are valid.
                buffer_used = 9;
                // Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
                response_buffer = &buffer[6];
                response_length = 3;
            }
            else { // This is an ANTICOLLISION.
                RC522_LOGD("ANTICOLLISION");

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
            ret = rc522_picc_transceive(rc522,
                buffer,
                buffer_used,
                response_buffer,
                &response_length,
                &tx_last_bits,
                rx_align,
                false);

            if (ret == ESP_ERR_RC522_COLLISION) { // More than one PICC in the field => collision.
                RC522_LOGD("collision detected");

                // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
                uint8_t value_of_coll_reg;
                rc522_pcd_read(rc522, RC522_PCD_COLL_REG, &value_of_coll_reg);

                if (value_of_coll_reg & 0x20) { // CollPosNotValid
                    // Without a valid collision position we cannot continue
                    return ESP_ERR_RC522_COLLISION;
                }

                uint8_t collision_pos = value_of_coll_reg & 0x1F; // Values 0-31, 0 means bit 32.
                if (collision_pos == 0) {
                    collision_pos = 32;
                }
                if (collision_pos <= current_level_known_bits) { // No progress - should not happen
                    return ESP_FAIL;
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

        RC522_LOGD("SELECT done");

        // We do not check the CBB - it was constructed by us above.

        // Copy the found UID bytes from buffer[] to uid->uidByte[]
        index = (buffer[2] == RC522_PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
        bytes_to_copy = (buffer[2] == RC522_PICC_CMD_CT) ? 3 : 4;
        for (count = 0; count < bytes_to_copy; count++) {
            picc->uid.bytes[uid_index + count] = buffer[index++];
        }

        // Check response SAK (Select Acknowledge)
        if (response_length != 3 || tx_last_bits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
            return ESP_FAIL;
        }
        // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed
        // anymore.

// compiler complains about uninitialized response_buffer even is
// no chance that response_buffer is NULL here, so ignore warning here
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
        RC522_RETURN_ON_ERROR(rc522_pcd_calculate_crc(rc522, response_buffer, 1, buffer + 2));

        if ((buffer[2] != response_buffer[1]) || (buffer[3] != response_buffer[2])) {
            return ESP_ERR_RC522_CRC_WRONG;
        }
        if (response_buffer[0] & 0x04) { // Cascade bit set - UID not complete yes
            cascade_level++;
        }
        else {
            uid_complete = true;
            picc->sak = response_buffer[0];
        }
#pragma GCC diagnostic pop
    } // End of while (!uidComplete)

    // Set correct uid->size
    picc->uid.bytes_length = 3 * cascade_level + 1;

    return ESP_OK;
}

static rc522_picc_type_t rc522_picc_type(uint8_t sak)
{
    // http://www.nxp.com/documents/application_note/AN10833.pdf
    // 3.2 Coding of Select Acknowledge (SAK)
    // ignore 8-bit (iso14443 starts with LSBit = bit 1)
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
            return RC522_PICC_TYPE_ISO_14443_4;
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
            return "MIFARE 1KB";
        case RC522_PICC_TYPE_MIFARE_4K:
            return "MIFARE 4KB";
        case RC522_PICC_TYPE_MIFARE_UL:
            return "MIFARE Ultralight or Ultralight C";
        case RC522_PICC_TYPE_MIFARE_PLUS:
            return "MIFARE Plus";
        case RC522_PICC_TYPE_MIFARE_DESFIRE:
            return "MIFARE DESFire";
        case RC522_PICC_TYPE_TNP3XXX:
            return "MIFARE TNP3XXX";
        case RC522_PICC_TYPE_UNKNOWN:
        default:
            return "unknown";
    }
}

/**
 * Resolve collision and select PICC
 */
esp_err_t rc522_picc_fetch(rc522_handle_t rc522, rc522_picc_t *picc)
{
    ESP_RETURN_ON_FALSE(picc != NULL, ESP_ERR_INVALID_ARG, TAG, "picc is null");

    RC522_RETURN_ON_ERROR(rc522_picc_select(rc522, picc, 0));

    picc->type = rc522_picc_type(picc->sak);

    return ESP_OK;
}

esp_err_t rc522_picc_halta(rc522_handle_t rc522, rc522_picc_t *picc)
{
    uint8_t buffer[4];

    // Build command buffer
    buffer[0] = RC522_PICC_CMD_HLTA;
    buffer[1] = 0;
    // Calculate CRC_A
    RC522_RETURN_ON_ERROR(rc522_pcd_calculate_crc(rc522, buffer, 2, &buffer[2]));

    // Send the command.
    // The standard says:
    //		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
    //		HLTA command, this response shall be interpreted as 'not acknowledge'.
    // We interpret that this way: Only STATUS_TIMEOUT is a success.
    esp_err_t ret = rc522_picc_transceive(rc522, buffer, sizeof(buffer), NULL, NULL, NULL, 0, false);

    if (ret == ESP_ERR_TIMEOUT) {
        return ESP_OK;
    }

    if (ret == ESP_OK) {
        // That is ironically NOT ok in this case ;-)
        return ESP_FAIL;
    }

    return ret;
}
