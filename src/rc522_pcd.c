#include <esp_system.h>
#include <esp_check.h>
#include <string.h>

#include "rc522_types_internal.h"
#include "rc522_helpers_internal.h"
#include "rc522_driver_internal.h"
#include "rc522_pcd_internal.h"

RC522_LOG_DEFINE_BASE();

/**
 * @see https://stackoverflow.com/a/48705557
 */
esp_err_t rc522_pcd_calculate_crc(const rc522_handle_t rc522, const rc522_bytes_t *bytes, rc522_pcd_crc_t *result)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK_BYTES(bytes);
    RC522_CHECK(result == NULL);

    RC522_RETURN_ON_ERROR(rc522_pcd_stop_active_command(rc522));
    RC522_RETURN_ON_ERROR(rc522_pcd_clear_bits(rc522, RC522_PCD_DIV_INT_REQ_REG, RC522_PCD_CRC_IRQ_BIT));
    RC522_RETURN_ON_ERROR(rc522_pcd_fifo_flush(rc522));
    RC522_RETURN_ON_ERROR(rc522_pcd_fifo_write(rc522, bytes));
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_PCD_COMMAND_REG, RC522_PCD_CALC_CRC_CMD));

    uint32_t deadline_ms = rc522_millis() + 90;
    bool calculation_done = false;

    do {
        uint8_t irq;
        RC522_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_PCD_DIV_INT_REQ_REG, &irq));

        if (irq & RC522_PCD_CRC_IRQ_BIT) {
            calculation_done = true;
            break;
        }

        taskYIELD();
    }
    while (rc522_millis() < deadline_ms);

    if (!calculation_done) { // Deadline reached
        return ESP_ERR_TIMEOUT;
    }

    rc522_pcd_crc_t crc = { 0 };

    RC522_RETURN_ON_ERROR(rc522_pcd_stop_active_command(rc522));
    RC522_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_PCD_CRC_RESULT_MSB_REG, &crc.msb));
    RC522_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_PCD_CRC_RESULT_LSB_REG, &crc.lsb));

    if (RC522_LOG_LEVEL >= ESP_LOG_DEBUG) {
        char debug_buffer[64];
        rc522_buffer_to_hex_str(bytes->ptr, bytes->length, debug_buffer, sizeof(debug_buffer));
        RC522_LOGD("crc(%s) = 0x%04" RC522_X, debug_buffer, crc.value);
    }

    *result = crc;

    return ESP_OK;
}

static esp_err_t rc522_pcd_wait_for_reset(const rc522_handle_t rc522, uint32_t timeout_ms)
{
    RC522_CHECK(rc522 == NULL);

    esp_err_t ret = ESP_OK;
    bool power_down_bit = true;
    uint32_t start_ms = rc522_millis();

    // Wait for the PowerDown bit in CommandReg to be cleared
    do {
        rc522_delay_ms(25);

        uint8_t cmd;
        if ((ret = rc522_pcd_read(rc522, RC522_PCD_COMMAND_REG, &cmd)) != ESP_OK) {
            RC522_LOGD("wait for reset error %04" RC522_X, ret);
            continue;
        }

        if (!(power_down_bit = (cmd & RC522_PCD_POWER_DOWN_BIT))) {
            break;
        }
    }
    while ((rc522_millis() - start_ms) < timeout_ms);

    return power_down_bit ? ESP_ERR_TIMEOUT : ESP_OK;
}

inline static esp_err_t rc522_pcd_hard_reset(const rc522_handle_t rc522, uint32_t timeout_ms)
{
    RC522_CHECK(rc522 == NULL);
    RC522_LOGD("hard reset");

    esp_err_t ret = rc522_driver_reset(rc522->config->driver);

    return ret == ESP_OK ? rc522_pcd_wait_for_reset(rc522, timeout_ms) : ret;
}

inline static esp_err_t rc522_pcd_soft_reset(const rc522_handle_t rc522, uint32_t timeout_ms)
{
    RC522_CHECK(rc522 == NULL);
    RC522_LOGD("soft reset");

    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_PCD_COMMAND_REG, RC522_PCD_SOFT_RESET_CMD));

    return rc522_pcd_wait_for_reset(rc522, timeout_ms);
}

esp_err_t rc522_pcd_reset(const rc522_handle_t rc522, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;

    if ((ret = rc522_pcd_hard_reset(rc522, timeout_ms)) != ESP_OK) {
        if (ret != RC522_ERR_RST_PIN_UNUSED) {
            RC522_LOGW("hard reset failed, trying soft reset");
        }

        ret = rc522_pcd_soft_reset(rc522, timeout_ms);
    }

    return ret;
}

inline static esp_err_t rc522_pcd_tx_enable(const rc522_handle_t rc522)
{
    return rc522_pcd_set_bits(rc522, RC522_PCD_TX_CONTROL_REG, (RC522_PCD_TX2_RF_EN_BIT | RC522_PCD_TX1_RF_EN_BIT));
}

static esp_err_t rc522_pcd_configure_timer(const rc522_handle_t rc522, uint8_t mode, uint16_t prescaler)
{
    uint8_t prescaler_hi = (prescaler >> 8) & 0x0F;
    uint8_t timer_mode = (mode & 0xF0) | prescaler_hi;
    uint8_t prescaler_lo = (prescaler & 0xFF);

    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_PCD_TIMER_MODE_REG, timer_mode));
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_PCD_TIMER_PRESCALER_REG, prescaler_lo));

    return ESP_OK;
}

inline static esp_err_t rc522_pcd_set_timer_reload_value(const rc522_handle_t rc522, uint16_t value)
{
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_PCD_TIMER_RELOAD_MSB_REG, (value >> 8) & 0xFF));
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_PCD_TIMER_RELOAD_LSB_REG, value & 0xFF));

    return ESP_OK;
}

inline static esp_err_t rc522_pcd_set_rx_gain(const rc522_handle_t rc522, rc522_pcd_rx_gain_t gain)
{
    return rc522_pcd_set_bits(rc522, RC522_PCD_RF_CFG_REG, gain);
}

esp_err_t rc522_pcd_init(const rc522_handle_t rc522)
{
    RC522_CHECK(rc522 == NULL);

    // Reset baud rates
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_PCD_TX_MODE_REG, RC522_PCD_TX_MODE_REG_RESET_VALUE));
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_PCD_RX_MODE_REG, RC522_PCD_RX_MODE_REG_RESET_VALUE));

    // Reset modulation width
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_PCD_MOD_WIDTH_REG, RC522_PCD_MOD_WIDTH_REG_RESET_VALUE));

    // When communicating with a PICC we need a timeout if something goes wrong.
    // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
    // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.

    // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
    RC522_RETURN_ON_ERROR(rc522_pcd_configure_timer(rc522, RC522_PCD_T_AUTO_BIT, 169));

    // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
    RC522_RETURN_ON_ERROR(rc522_pcd_set_timer_reload_value(rc522, 1000));

    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522, RC522_PCD_TX_ASK_REG, RC522_PCD_FORCE_100_ASK_BIT));

    // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3
    // part 6.2.4)
    RC522_RETURN_ON_ERROR(rc522_pcd_write(rc522,
        RC522_PCD_MODE_REG,
        (RC522_PCD_TX_WAIT_RF_BIT | RC522_PCD_POL_MFIN_BIT | RC522_PCD_CRC_PRESET_6363H)));

    // Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
    RC522_RETURN_ON_ERROR(rc522_pcd_tx_enable(rc522));

    rc522_pcd_firmware_t fw;
    ESP_RETURN_ON_ERROR(rc522_pcd_firmware(rc522, &fw), TAG, "read fw version failed");

    // When 0x00 or 0xFF is returned, communication probably failed
    if (fw == 0x00 || fw == 0xFF) {
        RC522_LOGE("Communication failure, is the MFRC522 properly connected?");

        return ESP_FAIL; // TODO: use custom err
    }

    ESP_LOGI(TAG, "PCD (fw=%s) initialized", rc522_pcd_firmware_name(fw));

    return ESP_OK;
}

esp_err_t rc522_pcd_firmware(const rc522_handle_t rc522, rc522_pcd_firmware_t *fw)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(fw == NULL);

    uint8_t value;
    RC522_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_PCD_VERSION_REG, &value));

    *fw = (rc522_pcd_firmware_t)value;

    return ESP_OK;
}

char *rc522_pcd_firmware_name(rc522_pcd_firmware_t firmware)
{
    switch (firmware) {
        case RC522_PCD_FIRMWARE_CLONE:
            return "clone";
        case RC522_PCD_FIRMWARE_00:
            return "v0.0";
        case RC522_PCD_FIRMWARE_10:
            return "v1.0";
        case RC522_PCD_FIRMWARE_20:
            return "v2.0";
        case RC522_PCD_FIRMWARE_COUNTERFEIT:
            return "counterfeit_chip";
        default:
            return "unknown";
    }
}

inline esp_err_t rc522_pcd_stop_active_command(const rc522_handle_t rc522)
{
    return rc522_pcd_write(rc522, RC522_PCD_COMMAND_REG, RC522_PCD_IDLE_CMD);
}

inline esp_err_t rc522_pcd_clear_all_com_interrupts(const rc522_handle_t rc522)
{
    return rc522_pcd_write(rc522, RC522_PCD_COM_INT_REQ_REG, (uint8_t)(~RC522_PCD_SET_1_BIT));
}

inline esp_err_t rc522_pcd_fifo_write(const rc522_handle_t rc522, const rc522_bytes_t *bytes)
{
    return rc522_pcd_write_n(rc522, RC522_PCD_FIFO_DATA_REG, bytes);
}

inline esp_err_t rc522_pcd_fifo_read(const rc522_handle_t rc522, rc522_bytes_t *bytes)
{
    return rc522_pcd_read_n(rc522, RC522_PCD_FIFO_DATA_REG, bytes);
}

inline esp_err_t rc522_pcd_fifo_flush(const rc522_handle_t rc522)
{
    return rc522_pcd_write(rc522, RC522_PCD_FIFO_LEVEL_REG, RC522_PCD_FLUSH_BUFFER_BIT);
}

inline esp_err_t rc522_pcd_start_data_transmission(const rc522_handle_t rc522)
{
    return rc522_pcd_set_bits(rc522, RC522_PCD_BIT_FRAMING_REG, RC522_PCD_START_SEND_BIT);
}

inline esp_err_t rc522_pcd_stop_data_transmission(const rc522_handle_t rc522)
{
    return rc522_pcd_clear_bits(rc522, RC522_PCD_BIT_FRAMING_REG, RC522_PCD_START_SEND_BIT);
}

inline esp_err_t rc522_pcd_stop_crypto1(const rc522_handle_t rc522)
{
    return rc522_pcd_clear_bits(rc522, RC522_PCD_STATUS_2_REG, RC522_PCD_MF_CRYPTO1_ON_BIT);
}

esp_err_t rc522_pcd_rw_test(const rc522_handle_t rc522)
{
    RC522_CHECK(rc522 == NULL);

    uint8_t tmp;

    ESP_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_PCD_FIFO_LEVEL_REG, &tmp), TAG, "Cannot read FIFO length");
    ESP_RETURN_ON_ERROR(rc522_pcd_fifo_flush(rc522), TAG, "Cannot flush FIFO");

    uint8_t buffer1[] = { 0x13, 0x33, 0x37 };
    const uint8_t buffer_size = sizeof(buffer1);
    uint8_t buffer2[buffer_size];

    ESP_RETURN_ON_ERROR(rc522_pcd_fifo_write(rc522, &(rc522_bytes_t) { .ptr = buffer1, .length = buffer_size }),
        TAG,
        "Cannot write to FIFO");
    RC522_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_PCD_FIFO_LEVEL_REG, &tmp));
    ESP_RETURN_ON_FALSE(tmp == buffer_size, ESP_FAIL, TAG, "FIFO length missmatch after write");
    RC522_RETURN_ON_ERROR(rc522_pcd_fifo_read(rc522, &(rc522_bytes_t) { .ptr = buffer2, .length = buffer_size }));

    if (memcmp(buffer1, buffer2, buffer_size) != 0) {
        RC522_LOGE("Buffers content missmatch");
        RC522_LOGE("Buffer1: ");
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, buffer1, buffer_size, ESP_LOG_ERROR);
        RC522_LOGE("Buffer2: ");
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, buffer2, buffer_size, ESP_LOG_ERROR);

        return ESP_FAIL;
    }

    return ESP_OK;
}

inline esp_err_t rc522_pcd_write_n(const rc522_handle_t rc522, rc522_pcd_register_t addr, const rc522_bytes_t *bytes)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK_BYTES(bytes);

    if (RC522_LOG_LEVEL >= ESP_LOG_VERBOSE) {
        char debug_buffer[64];
        rc522_buffer_to_hex_str(bytes->ptr, bytes->length, debug_buffer, sizeof(debug_buffer));
        RC522_LOGV("pcd [0x%02" RC522_X "] <<< %s", addr, debug_buffer);
    }

    RC522_RETURN_ON_ERROR(rc522_driver_send(rc522->config->driver, addr, bytes));

    return ESP_OK;
}

inline esp_err_t rc522_pcd_write(const rc522_handle_t rc522, rc522_pcd_register_t addr, uint8_t val)
{
    return rc522_pcd_write_n(rc522, addr, &(rc522_bytes_t) { .ptr = &val, .length = 1 });
}

inline esp_err_t rc522_pcd_read_n(const rc522_handle_t rc522, rc522_pcd_register_t addr, rc522_bytes_t *bytes)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK_BYTES(bytes);

    esp_err_t ret = rc522_driver_receive(rc522->config->driver, addr, bytes);

    if (RC522_LOG_LEVEL >= ESP_LOG_VERBOSE) {
        char debug_buffer[64];
        rc522_buffer_to_hex_str(bytes->ptr, bytes->length, debug_buffer, sizeof(debug_buffer));
        RC522_LOGV("pcd [0x%02" RC522_X "] >>> %s", addr, debug_buffer);
    }

    return ret;
}

inline esp_err_t rc522_pcd_read(const rc522_handle_t rc522, rc522_pcd_register_t addr, uint8_t *value_ref)
{
    return rc522_pcd_read_n(rc522, addr, &(rc522_bytes_t) { .ptr = value_ref, .length = 1 });
}

inline esp_err_t rc522_pcd_set_bits(const rc522_handle_t rc522, rc522_pcd_register_t addr, uint8_t bits)
{
    uint8_t value;
    RC522_RETURN_ON_ERROR(rc522_pcd_read(rc522, addr, &value));

    return rc522_pcd_write(rc522, addr, value | bits);
}

inline esp_err_t rc522_pcd_clear_bits(const rc522_handle_t rc522, rc522_pcd_register_t addr, uint8_t bits)
{
    uint8_t value;
    ESP_RETURN_ON_ERROR(rc522_pcd_read(rc522, addr, &value), TAG, "");

    return rc522_pcd_write(rc522, addr, value & (~bits));
}
