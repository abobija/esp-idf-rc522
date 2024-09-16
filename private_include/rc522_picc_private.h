#pragma once

#include "rc522_pcd_private.h"
#include "rc522_picc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Commands sent to the PICC
 */
typedef enum
{
    /**
     * The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)
     * REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for
     * anticollision or selection. 7 bit frame.
     */
    RC522_PICC_CMD_REQA = 0x26,

    /**
     * Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare
     * for anticollision or selection. 7 bit frame.
     */
    RC522_PICC_CMD_WUPA = 0x52,

    /**
     * Cascade Tag. Not really a command, but used during anti collision.
     */
    RC522_PICC_CMD_CT = 0x88,

    /**
     * Anti collision/Select, Cascade Level 1
     */
    RC522_PICC_CMD_SEL_CL1 = 0x93,

    /**
     * Anti collision/Select, Cascade Level 2
     */
    RC522_PICC_CMD_SEL_CL2 = 0x95,

    /**
     * Anti collision/Select, Cascade Level 3
     */
    RC522_PICC_CMD_SEL_CL3 = 0x97,

    /**
     * HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
     */
    RC522_PICC_CMD_HLTA = 0x50,

    /**
     * Request command for Answer To Reset.
     *
     */
    RC522_PICC_CMD_RATS = 0xE0,
} rc522_picc_command_t;

esp_err_t rc522_picc_comm(rc522_handle_t rc522, rc522_pcd_command_t command, uint8_t wait_irq, uint8_t *send_data,
    uint8_t send_data_len, uint8_t *back_data, uint8_t *back_data_len, uint8_t *valid_bits, uint8_t rx_align,
    bool check_crc);

esp_err_t rc522_picc_transceive(rc522_handle_t rc522, uint8_t *send_data, uint8_t send_data_len, uint8_t *back_data,
    uint8_t *back_data_len, uint8_t *valid_bits, uint8_t rx_align, bool check_crc);

esp_err_t rc522_picc_find(rc522_handle_t rc522, rc522_picc_t *picc);

esp_err_t rc522_picc_activate(rc522_handle_t rc522, rc522_picc_t *picc);

esp_err_t rc522_picc_halta(rc522_handle_t rc522, rc522_picc_t *picc);

#ifdef __cplusplus
}
#endif
