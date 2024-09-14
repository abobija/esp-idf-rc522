#pragma once

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

    /**
     * The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
     * Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on
     * the sector.
     * The read/write commands can also be used for MIFARE Ultralight.
     *
     * Perform authentication with Key A
     */
    RC522_PICC_CMD_MF_AUTH_KEY_A = 0x60,

    /**
     * Perform authentication with Key B
     */
    RC522_PICC_CMD_MF_AUTH_KEY_B = 0x61,

    /**
     * Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
     */
    RC522_PICC_CMD_MF_READ = 0x30,

    /**
     * Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY
     * WRITE" for MIFARE Ultralight.
     */
    RC522_PICC_CMD_MF_WRITE = 0xA0,

    /**
     * Decrements the contents of a block and stores the result in the internal data register.
     */
    RC522_PICC_CMD_MF_DECREMENT = 0xC0,

    /**
     * Increments the contents of a block and stores the result in the internal data register.
     */
    RC522_PICC_CMD_MF_INCREMENT = 0xC1,

    /**
     * Reads the contents of a block into the internal data register.
     */
    RC522_PICC_CMD_MF_RESTORE = 0xC2,

    /**
     * Writes the contents of the internal data register to a block.
     */
    RC522_PICC_CMD_MF_TRANSFER = 0xB0,

    /**
     * The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
     * The RC522_PICC_CMD_MF_READ and RC522_PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
     *
     * Writes one 4 byte page to the PICC.
     */
    RC522_PICC_CMD_UL_WRITE = 0xA2,
} rc522_picc_command_t;

esp_err_t rc522_picc_find(rc522_handle_t rc522, rc522_picc_t *picc);

esp_err_t rc522_picc_fetch(rc522_handle_t rc522, rc522_picc_t *picc);

esp_err_t rc522_picc_halta(rc522_handle_t rc522, rc522_picc_t *picc);

#ifdef __cplusplus
}
#endif
