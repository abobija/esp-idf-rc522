#pragma once

#include "rc522_types.h"
#include "rc522_picc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC522_MIFARE_DEFAULT_KEY                                                                                       \
    {                                                                                                                  \
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF                                                                             \
    }

/**
 * The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
 * Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on
 * the sector.
 */
typedef enum
{
    /**
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
} rc522_mifare_command_t;

bool rc522_mifare_is_mifare_classic_compatible(rc522_picc_t *picc);

esp_err_t rc522_mifare_dump_data_to_log(
    rc522_handle_t rc522, rc522_picc_t *picc, const uint8_t *key, uint8_t key_length);

#ifdef __cplusplus
}
#endif
