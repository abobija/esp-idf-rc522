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
    RX_GAIN_2 = BIT6,
    RX_GAIN_1 = BIT5,
    RX_GAIN_0 = BIT4,
} rc522_rf_cfg_reg_bit_t;

/**
 * Receiver (antenna) gain
 */
typedef enum
{
    RC522_RX_GAIN_18_DB = (RX_GAIN_1),                         /* 18 dB */
    RC522_RX_GAIN_23_DB = (RX_GAIN_1 | RX_GAIN_0),             /* 23 dB */
    RC522_RX_GAIN_33_DB = (RX_GAIN_2),                         /* 33 dB */
    RC522_RX_GAIN_38_DB = (RX_GAIN_2 | RX_GAIN_0),             /* 38 dB */
    RC522_RX_GAIN_43_DB = (RX_GAIN_2 | RX_GAIN_1),             /* 43 dB */
    RC522_RX_GAIN_48_DB = (RX_GAIN_2 | RX_GAIN_1 | RX_GAIN_0), /* 48 dB */
} rc522_rx_gain_t;
