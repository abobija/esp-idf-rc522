#pragma once

#include "rc522_types.h"
#include "rc522_picc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC522_ERR_MIFARE_BASE                            (ESP_ERR_RC522_BASE + 100)
#define RC522_ERR_MIFARE_ACCESS_BITS_INTEGRITY_VIOLATION (RC522_ERR_MIFARE_BASE + 1)

#define RC522_MIFARE_KEY_SIZE (6)
#define RC522_MIFARE_DEFAULT_KEY_VALUE                                                                                 \
    {                                                                                                                  \
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF                                                                             \
    }

typedef enum
{
    RC522_MIFARE_KEY_A,
    RC522_MIFARE_KEY_B,
} rc522_mifare_key_type_t;

typedef struct
{
    uint8_t value[RC522_MIFARE_KEY_SIZE];
} rc522_mifare_key_t;

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
    RC522_MIFARE_AUTH_KEY_A_CMD = 0x60,

    /**
     * Perform authentication with Key B
     */
    RC522_MIFARE_AUTH_KEY_B_CMD = 0x61,

    /**
     * Reads one 16 byte block from the authenticated sector of the PICC
     */
    RC522_MIFARE_READ_CMD = 0x30,

    /**
     * Writes one 16 byte block to the authenticated sector of the PICC
     */
    RC522_MIFARE_WRITE_CMD = 0xA0,

    /**
     * Decrements the contents of a block and stores the result in the internal data register
     */
    RC522_MIFARE_DECREMENT_CMD = 0xC0,

    /**
     * Increments the contents of a block and stores the result in the internal data register
     */
    RC522_MIFARE_INCREMENT_CMD = 0xC1,

    /**
     * Reads the contents of a block into the internal data register
     */
    RC522_MIFARE_RESTORE_CMD = 0xC2,

    /**
     * Writes the contents of the internal data register to a block
     */
    RC522_MIFARE_TRANSFER_CMD = 0xB0,
} rc522_mifare_command_t;

bool rc522_mifare_type_is_classic_compatible(rc522_picc_type_t type);

esp_err_t rc522_mifare_dump(rc522_handle_t rc522, rc522_picc_t *picc, rc522_mifare_key_t *key);

#ifdef __cplusplus
}
#endif
