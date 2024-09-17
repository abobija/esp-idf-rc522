#pragma once

#include "rc522_pcd.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC522_PCD_MOD_WIDTH_RESET_VALUE (0x26)
#define RC522_PCD_TX_MODE_RESET_VALUE   (0x00)
#define RC522_PCD_RX_MODE_RESET_VALUE   (RC522_RX_NO_ERR)

typedef enum
{
    // Starts and stops command execution
    RC522_PCD_COMMAND_REG = 0x01,

    // Communication Interrupt Enable Register.
    // Control bits to enable and disable the passing of interrupt requests
    RC522_PCD_COMM_INT_EN_REG = 0x02,

    // Diverted Interrupt Enable Register.
    // Control bits to enable and disable the passing of interrupt requests
    RC522_PCD_DIV_INT_EN_REG = 0x03,

    // Communication Interrupt request bits
    RC522_PCD_COMM_INT_REQ_REG = 0x04,

    // Diverted Interrupt request bits
    RC522_PCD_DIV_INT_REQ_REG = 0x05,

    // Error bits showing the error status of the last command  executed
    RC522_PCD_ERROR_REG = 0x06,

    // Contains status bits of the receiver, transmitter and data mode detector
    RC522_PCD_STATUS_2_REG = 0x08,

    // Input and output of 64 byte FIFO buffer
    RC522_PCD_FIFO_DATA_REG = 0x09,

    // Number of bytes stored in the FIFO buffer
    RC522_PCD_FIFO_LEVEL_REG = 0x0A,

    // Shows the MFRC522 software version
    RC522_PCD_VERSION_REG = 0x37,

    // Controls the logical behavior of the antenna driver pins TX1 and TX2
    RC522_PCD_TX_CONTROL_REG = 0x14,

    // Configures the receiver gain
    RC522_PCD_RF_CFG_REG = 0x26,

    // Miscellaneous control register
    RC522_PCD_CONTROL_REG = 0x0C,

    // Adjustments for bit-oriented frames
    RC522_PCD_BIT_FRAMING_REG = 0x0D,

    // Defines the first bit-collision detected on the RF interface
    RC522_PCD_COLL_REG = 0x0E,

    // MSB (higher bits) values of the CRC calculation
    RC522_PCD_CRC_RESULT_MSB_REG = 0x21,

    // LSB (lower bits) values of the CRC calculation
    RC522_PCD_CRC_RESULT_LSB_REG = 0x22,

    // Sets the modulation width
    RC522_PCD_MOD_WIDTH_REG = 0x24,

    // Defines the mode of the timer
    RC522_PCD_TIMER_MODE_REG = 0x2A,

    // Defines the timer prescaler settings
    RC522_PCD_TIMER_PRESCALER_REG = 0x2B,

    // MSB (higher bits) value of 16-bit timer reload value
    RC522_PCD_TIMER_RELOAD_MSB_REG = 0x2C,

    // LSB (lower bits) value of 16-bit timer reload value
    RC522_PCD_TIMER_RELOAD_LSB_REG = 0x2D,

    // Defines general modes for transmitting and receiving
    RC522_PCD_MODE_REG = 0x11,

    // Controls the setting of the transmission modulation
    RC522_PCD_TX_ASK_REG = 0x15,

    // Defines the data rate during transmission
    RC522_PCD_TX_MODE_REG = 0x12,

    // Defines the data rate during reception
    RC522_PCD_RX_MODE_REG = 0x13,
} rc522_pcd_register_t;

/**
 * Bits of RC522_PCD_COMMAND_REG register
 */
typedef enum
{
    // Soft power-down mode entered
    RC522_PCD_POWER_DOWN_BIT = BIT4,
} rc522_pcd_command_reg_bit_t;

/**
 * Bits of RC522_PCD_TX_CONTROL_REG register
 */
typedef enum
{
    // Output signal on pin TX2 delivers the 13.56 MHz energy carrier modulated by the transmission data
    RC522_PCD_TX2_RF_EN_BIT = BIT1,

    // Output signal on pin TX1 delivers the 13.56 MHz energy carrier modulated by the transmission data
    RC522_PCD_TX1_RF_EN_BIT = BIT0,
} rc522_pcd_tx_control_reg_bit_t;

/**
 * Bits of RC522_PCD_RF_CFG_REG register
 */
typedef enum
{
    RC522_PCD_RX_GAIN_2_BIT = BIT6,
    RC522_PCD_RX_GAIN_1_BIT = BIT5,
    RC522_PCD_RX_GAIN_0_BIT = BIT4,
} rc522_pcd_rf_cfg_reg_bit_t;

/**
 * Receiver (antenna) gain
 */
typedef enum
{
    RC522_PCD_18_DB_RX_GAIN = (RC522_PCD_RX_GAIN_1_BIT),                                                     /* 18 dB */
    RC522_PCD_23_DB_RX_GAIN = (RC522_PCD_RX_GAIN_1_BIT | RC522_PCD_RX_GAIN_0_BIT),                           /* 23 dB */
    RC522_PCD_33_DB_RX_GAIN = (RC522_PCD_RX_GAIN_2_BIT),                                                     /* 33 dB */
    RC522_PCD_38_DB_RX_GAIN = (RC522_PCD_RX_GAIN_2_BIT | RC522_PCD_RX_GAIN_0_BIT),                           /* 38 dB */
    RC522_PCD_43_DB_RX_GAIN = (RC522_PCD_RX_GAIN_2_BIT | RC522_PCD_RX_GAIN_1_BIT),                           /* 43 dB */
    RC522_PCD_48_DB_RX_GAIN = (RC522_PCD_RX_GAIN_2_BIT | RC522_PCD_RX_GAIN_1_BIT | RC522_PCD_RX_GAIN_0_BIT), /* 48 dB */
} rc522_pcd_rx_gain_t;

typedef enum
{
    /**
     * Places the MFRC522 in Idle mode. The Idle command also terminates itself.
     */
    RC522_PCD_IDLE_CMD = 0b0000,

    /**
     * The FIFO buffer content is transferred to the CRC coprocessor and the CRC calculation is
     * started. The calculation result is stored in the CRCResultReg register. The CRC
     * calculation is not limited to a dedicated number of bytes. The calculation is not stopped
     * when the FIFO buffer is empty during the data stream. The next byte written to the FIFO
     * buffer is added to the calculation.
     *
     * The CRC preset value is controlled by the ModeReg register’s CRCPreset[1:0] bits. The
     * value is loaded in to the CRC coprocessor when the command starts.
     *
     * This command must be terminated by writing a command to the CommandReg register,
     * such as, the Idle command.
     *
     * If the AutoTestReg register’s SelfTest[3:0] bits are set correctly, the MFRC522 enters Self
     * Test mode. Starting the CalcCRC command initiates a digital self test. The result of the
     * self test is written to the FIFO buffer.
     */
    RC522_PCD_CALC_CRC_CMD = 0b0011,

    /**
     * This command continuously repeats the transmission of data from the FIFO buffer and the
     * reception of data from the RF field. The first action is transmit and after transmission the
     * command is changed to receive a data stream.
     *
     * Each transmit process must be started by setting the BitFramingReg register’s StartSend
     * bit to logic 1. This command must be cleared by writing any command to the
     * CommandReg register
     */
    RC522_PCD_TRANSCEIVE_CMD = 0b1100,

    /**
     * This command manages MIFARE authentication to enable a secure communication to
     * any MIFARE Mini, MIFARE 1K and MIFARE 4K card
     */
    RC522_PCD_MF_AUTH_CMD = 0b1110,

    /**
     * This command performs a reset of the device. The configuration data of the internal buffer
     * remains unchanged. All registers are set to the reset values. This command automatically
     * terminates when finished.
     */
    RC522_PCD_SOFT_RESET_CMD = 0b1111,
} rc522_pcd_command_t;

/**
 * Bits of RC522_PCD_TIMER_MODE_REG register
 */
typedef enum
{
    /**
     * When 1:
     * - Timer starts automatically at the end of the transmission in
     * all communication modes at all speeds
     * - If the RxModeReg register’s RxMultiple bit is not set, the
     * timer stops immediately after receiving the 5th bit (1 start bit,
     * 4 data bits)
     * - If the RxMultiple bit is set to logic 1 the timer never stops, in
     * which case the timer can be stopped by setting the
     * ControlReg register’s TStopNow bit to logic 1
     *
     * When 0:
     * - Indicates that the timer is not influenced by the protocol
     */
    RC522_PCD_T_AUTO_BIT = BIT7,
} rc522_pcd_timer_mode_reg_bit_t;

/**
 * Bits of RC522_PCD_TX_ASK_REG register
 */
typedef enum
{
    /**
     * Forces a 100 % ASK modulation independent of the ModGsPReg register setting
     */
    RC522_PCD_FORCE_100_ASK_BIT = BIT6,
} rc522_pcd_tx_ask_reg_bit_t;

/**
 * Bits of RC522_PCD_MODE_REG register
 */
typedef enum
{
    // Transmitter can only be started if an RF field is generated
    RC522_PCD_TX_WAIT_RF_BIT = BIT5,

    /**
     * Defines the polarity of pin MFIN
     *
     * 1 - Polarity of pin MFIN is active HIGH
     * 0 - Polarity of pin MFIN is active LOW
     */
    RC522_PCD_POL_MFIN_BIT = BIT3,

    RC522_PCD_CRC_PRESET_1_BIT = BIT1,
    RC522_PCD_CRC_PRESET_0_BIT = BIT0,
} rc522_pcd_mode_reg_bit_t;

typedef enum
{
    RC522_PCD_CRC_PRESET_0000H = (0x00),                                                    /* 0000h */
    RC522_PCD_CRC_PRESET_6363H = (RC522_PCD_CRC_PRESET_0_BIT),                              /* 6363h */
    RC522_PCD_CRC_PRESET_A671H = (RC522_PCD_CRC_PRESET_1_BIT),                              /* A671h */
    RC522_PCD_CRC_PRESET_FFFFH = (RC522_PCD_CRC_PRESET_1_BIT | RC522_PCD_CRC_PRESET_0_BIT), /* FFFFh */
} rc522_pcd_crc_preset_value_t;

/**
 * Bits of RC522_PCD_COMM_INT_REQ_REG
 */
typedef enum
{
    // The timer decrements the timer value in register TCounterValReg to zero
    RC522_PCD_TIMER_IRQ_BIT = BIT0,
} rc522_pcd_comm_int_req_reg_bit_t;

/**
 * Bits of RC522_PCD_DIV_INT_REQ_REG register
 */
typedef enum
{
    // The CalcCRC command is active and all data is processed (CRC calculation is done)
    RC522_PCD_CRC_IRQ_BIT = BIT2,
} rc522_pcd_div_int_req_reg_bit_t;

/**
 * Bits of RC522_PCD_FIFO_LEVEL_REG register
 */
typedef enum
{
    /**
     * Immediately clears the internal FIFO buffer’s read and write pointer
     * and ErrorReg register’s BufferOvfl bit
     *
     * Reading this bit always returns 0
     */
    RC522_PCD_FLUSH_BUFFER_BIT = BIT7,
} rc522_pcd_fifo_level_reg_bit_t;

/**
 * Bits of RC522_PCD_BIT_FRAMING_REG register
 */
typedef enum
{
    /**
     * Starts the transmission of data
     * Only valid in combination with the Transceive command
     */
    RC522_PCD_START_SEND_BIT = BIT7,
} rc522_pcd_bit_framing_reg_bit_t;

/**
 * Bits of RC522_PCD_STATUS_2_REG register
 */
typedef enum
{
    /**
     * Indicates that the MIFARE Crypto1 unit is switched on and
     * therefore all data communication with the card is encrypted
     * can only be set to logic 1 by a successful execution of the
     * MFAuthent command.
     * Only valid in Read/Write mode for MIFARE standard cards.
     * This bit is cleared by software.
     */
    RC522_PCD_MK_CRYPTO1_ON_BIT = BIT3,
} rc522_pcd_status_2_reg_bit_t;

/**
 * Bits of RC522_PCD_RX_MODE_REG register
 */
typedef enum
{
    /**
     * An invalid received data stream (less than 4 bits received) will
     * be ignored and the receiver remains active
     */
    RC522_RX_NO_ERR = BIT3,
} rc522_pcd_rx_mode_reg_bit_t;

typedef enum
{
    RC522_PCD_FIRMWARE_CLONE = 0x88,       // clone
    RC522_PCD_FIRMWARE_00 = 0x90,          // v0.0
    RC522_PCD_FIRMWARE_10 = 0x91,          // v1.0
    RC522_PCD_FIRMWARE_20 = 0x92,          // v2.0
    RC522_PCD_FIRMWARE_COUNTERFEIT = 0x12, // counterfeit chip
} rc522_pcd_firmware_t;

esp_err_t rc522_pcd_calculate_crc(rc522_handle_t rc522, uint8_t *data, uint8_t n, uint8_t *buffer);

esp_err_t rc522_pcd_init(rc522_handle_t rc522);

esp_err_t rc522_pcd_wait_for_any_bit(rc522_handle_t rc522, rc522_pcd_register_t addr, uint8_t bits, uint8_t stop_bits,
    uint32_t timeout_ms, uint8_t *ret_state);

esp_err_t rc522_pcd_firmware(rc522_handle_t rc522, rc522_pcd_firmware_t *result);

char *rc522_pcd_firmware_name(rc522_pcd_firmware_t firmware);

esp_err_t rc522_pcd_stop_active_command(rc522_handle_t rc522);

esp_err_t rc522_pcd_fifo_write(rc522_handle_t rc522, uint8_t *data, uint8_t data_length);

esp_err_t rc522_pcd_fifo_read(rc522_handle_t rc522, uint8_t *buffer, uint8_t length);

esp_err_t rc522_pcd_fifo_flush(rc522_handle_t rc522);

esp_err_t rc522_pcd_start_data_transmission(rc522_handle_t rc522);

esp_err_t rc522_pcd_stop_data_transmission(rc522_handle_t rc522);

esp_err_t rc522_pcd_stop_crypto1(rc522_handle_t rc522);

esp_err_t rc522_pcd_rw_test(rc522_handle_t rc522);

esp_err_t rc522_pcd_write_n(rc522_handle_t rc522, rc522_pcd_register_t addr, uint8_t n, uint8_t *data);

esp_err_t rc522_pcd_write(rc522_handle_t rc522, rc522_pcd_register_t addr, uint8_t val);

esp_err_t rc522_pcd_read_n(rc522_handle_t rc522, rc522_pcd_register_t addr, uint8_t n, uint8_t *buffer);

esp_err_t rc522_pcd_read(rc522_handle_t rc522, rc522_pcd_register_t addr, uint8_t *value_ref);

esp_err_t rc522_pcd_set_bits(rc522_handle_t rc522, rc522_pcd_register_t addr, uint8_t bits);

esp_err_t rc522_pcd_clear_bits(rc522_handle_t rc522, rc522_pcd_register_t addr, uint8_t bits);

esp_err_t rc522_pcd_write_map(rc522_handle_t rc522, const uint8_t map[][2], uint8_t map_length);

#ifdef __cplusplus
}
#endif
