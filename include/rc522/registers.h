#pragma once

#include <esp_bit_defs.h>

typedef enum
{
    // Starts and stops command execution
    RC522_COMMAND_REG = 0x01,

    // Communication Interrupt Enable Register.
    // Control bits to enable and disable the passing of interrupt requests
    RC522_COMM_INT_EN_REG = 0x02,

    // Diverted Interrupt Enable Register.
    // Control bits to enable and disable the passing of interrupt requests
    RC522_DIV_INT_EN_REG = 0x03,

    // Communication Interrupt request bits
    RC522_COMM_INT_REQ_REG = 0x04,

    // Diverted Interrupt request bits
    RC522_DIV_INT_REQ_REG = 0x05,

    // Error bits showing the error status of the last command  executed
    RC522_ERROR_REG = 0x06,

    // Contains status bits of the receiver, transmitter and data mode detector
    RC522_STATUS_2_REG = 0x08,

    // Shows the MFRC522 software version
    RC522_VERSION_REG = 0x37,

    // Controls the logical behavior of the antenna driver pins TX1 and TX2
    RC522_TX_CONTROL_REG = 0x14,

    // Configures the receiver gain
    RC522_RF_CFG_REG = 0x26,

    // Input and output of 64 byte FIFO buffer
    RC522_FIFO_DATA_REG = 0x09,

    // Number of bytes stored in the FIFO buffer
    RC522_FIFO_LEVEL_REG = 0x0A,

    // Miscellaneous control register
    RC522_CONTROL_REG = 0x0C,

    // Adjustments for bit-oriented frames
    RC522_BIT_FRAMING_REG = 0x0D,

    // MSB (higher bits) values of the CRC calculation
    RC522_CRC_RESULT_MSB_REG = 0x21,

    // LSB (lower bits) values of the CRC calculation
    RC522_CRC_RESULT_LSB_REG = 0x22,

    // Sets the modulation width
    RC522_MOD_WIDTH_REG = 0x24,

    // Defines the mode of the timer
    RC522_TIMER_MODE_REG = 0x2A,

    // Defines the timer prescaler settings
    RC522_TIMER_PRESCALER_REG = 0x2B,

    // MSB (higher bits) value of 16-bit timer reload value
    RC522_TIMER_RELOAD_MSB_REG = 0x2C,

    // LSB (lower bits) value of 16-bit timer reload value
    RC522_TIMER_RELOAD_LSB_REG = 0x2D,

    // Defines general modes for transmitting and receiving
    RC522_MODE_REG = 0x11,

    // Controls the setting of the transmission modulation
    RC522_TX_ASK_REG = 0x15,
} rc522_register_t;

/**
 * Bits of RC522_TX_CONTROL_REG register
 */
typedef enum
{
    // Output signal on pin TX1 delivers the 13.56 MHz energy carrier modulated by the transmission data
    RC522_TX1_RF_EN = BIT0,

    // Output signal on pin TX2 delivers the 13.56 MHz energy carrier modulated by the transmission data
    RC522_TX2_RF_EN = BIT1,
} rc522_tx_control_reg_bit_t;

/**
 * Bits of RC522_RF_CFG_REG register
 */
typedef enum
{
    RC522_RX_GAIN_2 = BIT6,
    RC522_RX_GAIN_1 = BIT5,
    RC522_RX_GAIN_0 = BIT4,
} rc522_rf_cfg_reg_bit_t;

/**
 * Receiver (antenna) gain
 */
typedef enum
{
    RC522_RX_GAIN_18_DB = (RC522_RX_GAIN_1),                                     /* 18 dB */
    RC522_RX_GAIN_23_DB = (RC522_RX_GAIN_1 | RC522_RX_GAIN_0),                   /* 23 dB */
    RC522_RX_GAIN_33_DB = (RC522_RX_GAIN_2),                                     /* 33 dB */
    RC522_RX_GAIN_38_DB = (RC522_RX_GAIN_2 | RC522_RX_GAIN_0),                   /* 38 dB */
    RC522_RX_GAIN_43_DB = (RC522_RX_GAIN_2 | RC522_RX_GAIN_1),                   /* 43 dB */
    RC522_RX_GAIN_48_DB = (RC522_RX_GAIN_2 | RC522_RX_GAIN_1 | RC522_RX_GAIN_0), /* 48 dB */
} rc522_rx_gain_t;

typedef enum
{
    /**
     * Places the MFRC522 in Idle mode. The Idle command also terminates itself.
     */
    RC522_CMD_IDLE = 0x00,

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
    RC522_CMD_CALC_CRC = 0b00000011,

    /**
     * This command continuously repeats the transmission of data from the FIFO buffer and the
     * reception of data from the RF field. The first action is transmit and after transmission the
     * command is changed to receive a data stream.
     *
     * Each transmit process must be started by setting the BitFramingReg register’s StartSend
     * bit to logic 1. This command must be cleared by writing any command to the
     * CommandReg register
     */
    RC522_CMD_TRANSCEIVE = 0b00001100,

    /**
     * This command manages MIFARE authentication to enable a secure communication to
     * any MIFARE Mini, MIFARE 1K and MIFARE 4K card
     */
    RC522_CMD_MF_AUTH = 0b00001110,

    /**
     * This command performs a reset of the device. The configuration data of the internal buffer
     * remains unchanged. All registers are set to the reset values. This command automatically
     * terminates when finished.
     */
    RC522_CMD_SOFT_RESET = 0b00001111,
} rc522_command_t;

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
    RC522_T_AUTO = BIT7,
} rc522_timer_mode_reg_bit_t;

typedef enum
{
    /**
     * Forces a 100 % ASK modulation independent of the ModGsPReg register setting
     */
    RC522_FORCE_100_ASK = BIT6,
} rc522_tx_ask_reg_bit_t;

typedef enum
{
    // Transmitter can only be started if an RF field is generated
    RC522_TX_WAIT_RF = BIT5,

    /**
     * Defines the polarity of pin MFIN
     *
     * 1 - Polarity of pin MFIN is active HIGH
     * 0 - Polarity of pin MFIN is active LOW
     */
    RC522_POL_MFIN = BIT3,

    RC522_CRC_PRESET_1 = BIT1,
    RC522_CRC_PRESET_0 = BIT0,
} rc522_mode_reg_bit_t;

typedef enum
{
    RC522_CRC_PRESET_0000H = (0x00),                                    /* 0000h */
    RC522_CRC_PRESET_6363H = (RC522_CRC_PRESET_0),                      /* 6363h */
    RC522_CRC_PRESET_A671H = (RC522_CRC_PRESET_1),                      /* A671h */
    RC522_CRC_PRESET_FFFFH = (RC522_CRC_PRESET_1 | RC522_CRC_PRESET_0), /* FFFFh */
} rc522_crc_preset_value_t;
