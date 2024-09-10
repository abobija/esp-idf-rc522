#pragma once

// Starts and stops command execution
#define RC522_COMMAND_REG 0x01

// Communication Interrupt Enable Register.
// Control bits to enable and disable the passing of interrupt requests
#define RC522_COMM_INT_EN_REG 0x02

// Diverted Interrupt Enable Register.
// Control bits to enable and disable the passing of interrupt requests
#define RC522_DIV_INT_EN_REG 0x03

// Communication Interrupt request bits
#define RC522_COMM_INT_REQ_REG 0x04

// Diverted Interrupt request bits
#define RC522_DIV_INT_REQ_REG 0x05

// Error bits showing the error status of the last command  executed
#define RC522_ERROR_REG 0x06

// Contains status bits of the receiver, transmitter and data mode detector
#define RC522_STATUS_2_REG 0x08

// Shows the MFRC522 software version
#define RC522_VERSION_REG 0x37

// Controls the logical behavior of the antenna driver pins TX1 and TX2
#define RC522_TX_CONTROL_REG 0x14

// Configures the receiver gain
#define RC522_RF_CFG_REG 0x26

// Input and output of 64 byte FIFO buffer
#define RC522_FIFO_DATA_REG 0x09

// Number of bytes stored in the FIFO buffer
#define RC522_FIFO_LEVEL_REG 0x0A

// Miscellaneous control register
#define RC522_CONTROL_REG 0x0C

// Adjustments for bit-oriented frames
#define RC522_BIT_FRAMING_REG 0x0D

// MSB (higher bits) values of the CRC calculation
#define RC522_CRC_RESULT_MSB_REG 0x21

// LSB (lower bits) values of the CRC calculation
#define RC522_CRC_RESULT_LSB_REG 0x22

// Sets the modulation width
#define RC522_MOD_WIDTH_REG 0x24

// Defines the mode of the timer
#define RC522_TIMER_MODE_REG 0x2A

// Defines the timer prescaler settings
#define RC522_TIMER_PRESCALER_REG 0x2B

// MSB (higher bits) value of 16-bit timer reload value
#define RC522_TIMER_RELOAD_MSB_REG 0x2C

// LSB (lower bits) value of 16-bit timer reload value
#define RC522_TIMER_RELOAD_LSB_REG 0x2D

// Defines general modes for transmitting and receiving
#define RC522_MODE_REG 0x11

// Controls the setting of the transmission modulation
#define RC522_TX_ASK_REG 0x15
