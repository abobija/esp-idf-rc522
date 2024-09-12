#include <esp_system.h>
#include <esp_check.h>
#include <string.h>

#include "comm/rc522_comm.h"
#include "comm/rc522_crc.h"
#include "rc522_io.h"

RC522_LOG_DEFINE_BASE();

static esp_err_t rc522_comm(rc522_handle_t rc522, rc522_command_t command, uint8_t wait_irq, uint8_t *send_data,
    uint8_t send_data_len, uint8_t *back_data, uint8_t *back_data_len, uint8_t *valid_bits, uint8_t rx_align,
    bool check_crc)
{
    RC522_LOG_IM_HERE();

    // Prepare values for bit framing
    uint8_t tx_last_bits = valid_bits ? *valid_bits : 0;
    uint8_t bit_framing = (rx_align << 4) + tx_last_bits;

    RC522_RETURN_ON_ERROR(rc522_stop_active_command(rc522));
    RC522_RETURN_ON_ERROR(rc522_write(rc522, RC522_COMM_INT_REQ_REG, 0x7F)); // Clear all seven interrupt request bits
    RC522_RETURN_ON_ERROR(rc522_fifo_flush(rc522));
    RC522_RETURN_ON_ERROR(rc522_fifo_write(rc522, send_data, send_data_len));
    RC522_RETURN_ON_ERROR(rc522_write(rc522, RC522_BIT_FRAMING_REG, bit_framing)); // Bit adjustments
    RC522_RETURN_ON_ERROR(rc522_write(rc522, RC522_COMMAND_REG, command));         // Execute the command

    if (command == RC522_CMD_TRANSCEIVE) {
        RC522_RETURN_ON_ERROR(rc522_start_data_transmission(rc522));
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
        RC522_RETURN_ON_ERROR(rc522_read(rc522, RC522_COMM_INT_REQ_REG, &irq));

        if (irq & wait_irq) { // One of the interrupts that signal success has been set.
            completed = true;
            break;
        }

        // Timer interrupt - nothing received in 25ms
        if (irq & 0x01) {
            RC522_LOGD("Timer interrupt (irq=0x%02x)", irq);

            return ESP_ERR_TIMEOUT;
        }

        taskYIELD();
    }
    while (rc522_millis() < deadline);

    // 36ms and nothing happened. Communication with the MFRC522 might be down.
    RC522_RETURN_ON_FALSE(completed, ESP_ERR_TIMEOUT);

    // Stop now if any errors except collisions were detected.
    uint8_t error_reg_value;
    RC522_RETURN_ON_ERROR(rc522_read(rc522, RC522_ERROR_REG, &error_reg_value));

    if (error_reg_value & 0x13) { // BufferOvfl ParityErr ProtocolErr
        return ESP_FAIL;
    }

    uint8_t _valid_bits = 0;

    // If the caller wants data back, get it from the MFRC522.
    if (back_data && back_data_len) {
        uint8_t length;
        RC522_RETURN_ON_ERROR(rc522_read(rc522, RC522_FIFO_LEVEL_REG, &length));

        if (length > *back_data_len) {
            return ESP_ERR_NO_MEM;
        }

        *back_data_len = length; // Number of bytes returned

        RC522_RETURN_ON_ERROR(rc522_read_n(rc522, RC522_FIFO_DATA_REG, length, back_data));

        // FIXME: Implement and do rx_align here or somehow on read_n like in arduino lib
        //
        // PCD_ReadRegister(FIFODataReg, n, backData, rxAlign); // Get received data from FIFO

        // RxLastBits[2:0] indicates the number of valid bits in the last
        // received byte. If this value is 000b, the whole byte is valid.
        RC522_RETURN_ON_ERROR(rc522_read(rc522, RC522_CONTROL_REG, &_valid_bits));
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
    if (back_data && back_data_len && check_crc) {
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

        RC522_RETURN_ON_ERROR(rc522_calculate_crc(rc522, back_data, *back_data_len, control_buffer));

        if ((back_data[*back_data_len - 2] != control_buffer[0]) ||
            (back_data[*back_data_len - 1] != control_buffer[1])) {
            return ESP_ERR_RC522_CRC_WRONG;
        }
    }

    return ESP_OK;
}

static esp_err_t rc522_transceive_data(rc522_handle_t rc522, uint8_t *send_data, uint8_t send_data_len,
    uint8_t *back_data, uint8_t *back_data_len, uint8_t *valid_bits, uint8_t rx_align, bool check_crc)
{
    RC522_LOG_IM_HERE();

    uint8_t wait_irq = 0x30; // RxIRq and IdleIRq

    return rc522_comm(rc522,
        RC522_CMD_TRANSCEIVE,
        wait_irq,
        send_data,
        send_data_len,
        back_data,
        back_data_len,
        valid_bits,
        rx_align,
        check_crc);
}

static esp_err_t rc522_reqa_or_wupa(
    rc522_handle_t rc522, uint8_t picc_cmd, uint8_t *atqa_buffer, uint8_t *atqa_buffer_size)
{
    RC522_LOG_IM_HERE();

    uint8_t valid_bits;

    if (atqa_buffer == NULL || *atqa_buffer_size < 2) { // The ATQA response is 2 bytes long.
        return ESP_ERR_NO_MEM;
    }

    // ValuesAfterColl=1 => Bits received after collision are cleared.
    RC522_RETURN_ON_ERROR(rc522_clear_bitmask(rc522, RC522_COLL_REG, 0x80));

    // For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only)
    // byte. TxLastBits = BitFramingReg[2..0]
    valid_bits = 7;

    RC522_RETURN_ON_ERROR_SILENTLY(
        rc522_transceive_data(rc522, &picc_cmd, 1, atqa_buffer, atqa_buffer_size, &valid_bits, 0, false));

    if (*atqa_buffer_size != 2 || valid_bits != 0) { // ATQA must be exactly 16 bits.
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t rc522_request_a(rc522_handle_t rc522, uint8_t *atqa_buffer, uint8_t *atqa_buffer_size)
{
    RC522_LOG_IM_HERE();

    const uint8_t picc_cmd_reqa = 0x26; // FIXME: Move to #define

    return rc522_reqa_or_wupa(rc522, picc_cmd_reqa, atqa_buffer, atqa_buffer_size);
}

esp_err_t rc522_picc_presence(rc522_handle_t rc522, rc522_picc_presence_t *result)
{
    RC522_LOG_IM_HERE();

    ESP_RETURN_ON_FALSE(result != NULL, ESP_ERR_INVALID_ARG, TAG, "result is null");

    uint8_t atqa_buffer[2];
    uint8_t atqa_buffer_size = sizeof(atqa_buffer);

    // Reset baud rates
    RC522_RETURN_ON_ERROR(rc522_write(rc522, RC522_TX_MODE_REG, 0x00));
    RC522_RETURN_ON_ERROR(rc522_write(rc522, RC522_RX_MODE_REG, 0x00));
    // Reset ModWidthReg
    RC522_RETURN_ON_ERROR(rc522_write(rc522, RC522_MOD_WIDTH_REG, RC522_MOD_WIDTH_RESET_VALUE));

    esp_err_t ret = rc522_request_a(rc522, atqa_buffer, &atqa_buffer_size);

    result->is_present = (ret == ESP_OK || ret == ESP_ERR_RC522_COLLISION);

    return ret;
}

esp_err_t rc522_firmware(rc522_handle_t rc522, rc522_firmware_t *result)
{
    uint8_t value;
    RC522_RETURN_ON_ERROR(rc522_read(rc522, RC522_VERSION_REG, &value));

    *result = (rc522_firmware_t)value;
    return ESP_OK;
}

char *rc522_firmware_name(rc522_firmware_t firmware)
{
    switch (firmware) {
        case RC522_FW_CLONE:
            return "clone";
        case RC522_FW_00:
            return "v0.0";
        case RC522_FW_10:
            return "v1.0";
        case RC522_FW_20:
            return "v2.0";
        case RC522_FW_COUNTERFEIT:
            return "counterfeit_chip";
    }

    return "unknown";
}

esp_err_t rc522_antenna_on(rc522_handle_t rc522)
{
    RC522_LOG_IM_HERE();

    uint8_t value;
    RC522_RETURN_ON_ERROR(rc522_read(rc522, RC522_TX_CONTROL_REG, &value));

    if ((value & 0x03) != 0x03) {
        RC522_RETURN_ON_ERROR(rc522_write(rc522, RC522_TX_CONTROL_REG, value | 0x03));
    }

    return ESP_OK;
}

inline esp_err_t rc522_stop_active_command(rc522_handle_t rc522)
{
    return rc522_write(rc522, RC522_COMMAND_REG, RC522_CMD_IDLE);
}

inline esp_err_t rc522_fifo_write(rc522_handle_t rc522, uint8_t *data, uint8_t data_length)
{
    return rc522_write_n(rc522, RC522_FIFO_DATA_REG, data_length, data);
}

inline esp_err_t rc522_fifo_flush(rc522_handle_t rc522)
{
    return rc522_write(rc522, RC522_FIFO_LEVEL_REG, RC522_FLUSH_BUFFER);
}

esp_err_t rc522_soft_reset(rc522_handle_t rc522, uint32_t timeout_ms)
{
    RC522_LOG_IM_HERE();

    RC522_RETURN_ON_ERROR(rc522_write(rc522, RC522_COMMAND_REG, RC522_CMD_SOFT_RESET));

    bool power_down_bit = true;
    uint32_t start_ms = rc522_millis();

    // Wait for the PowerDown bit in CommandReg to be cleared
    do {
        rc522_delay_ms(25);
        taskYIELD();

        uint8_t cmd;
        RC522_RETURN_ON_ERROR(rc522_read(rc522, RC522_COMMAND_REG, &cmd));

        if (!(power_down_bit = cmd & RC522_POWER_DOWN)) {
            break;
        }
    }
    while ((rc522_millis() - start_ms) < timeout_ms);

    return power_down_bit ? ESP_ERR_TIMEOUT : ESP_OK;
}

inline esp_err_t rc522_start_data_transmission(rc522_handle_t rc522)
{
    return rc522_set_bitmask(rc522, RC522_BIT_FRAMING_REG, RC522_START_SEND);
}

inline esp_err_t rc522_stop_data_transmission(rc522_handle_t rc522)
{
    return rc522_clear_bitmask(rc522, RC522_BIT_FRAMING_REG, RC522_START_SEND);
}
