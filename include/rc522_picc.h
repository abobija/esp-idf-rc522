#pragma once

#include "rc522_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC522_PICC_UID_SIZE_MAX        (10)
#define RC522_PICC_UID_STR_BUFFER_SIZE (RC522_PICC_UID_SIZE_MAX * 3)

typedef struct
{
    uint8_t value[RC522_PICC_UID_SIZE_MAX];
    uint8_t length;
} rc522_picc_uid_t;

typedef enum
{
    RC522_PICC_TYPE_UNKNOWN = -1,
    RC522_PICC_TYPE_UNDEFINED = 0,
    RC522_PICC_TYPE_ISO_14443_4,    // PICC compliant with ISO/IEC 14443-4
    RC522_PICC_TYPE_ISO_18092,      // PICC compliant with ISO/IEC 18092 (NFC)
    RC522_PICC_TYPE_MIFARE_MINI,    // MIFARE Classic protocol, 320 bytes
    RC522_PICC_TYPE_MIFARE_1K,      // MIFARE Classic protocol, 1KB
    RC522_PICC_TYPE_MIFARE_4K,      // MIFARE Classic protocol, 4KB
    RC522_PICC_TYPE_MIFARE_UL,      // MIFARE Ultralight or Ultralight C
    RC522_PICC_TYPE_MIFARE_PLUS,    // MIFARE Plus
    RC522_PICC_TYPE_MIFARE_DESFIRE, // MIFARE DESFire
    RC522_PICC_TYPE_TNP3XXX,        // Only mentioned in NXP AN 10833 MIFARE Type Identification Procedure
} rc522_picc_type_t;

typedef enum
{
    /**
     * In the POWER-OFF State, the PICC is not powered due to a lack of carrier energy
     */
    RC522_PICC_STATE_POWER_OFF = -1,

    /**
     * In the IDLE State, the PICC is powered.
     * It listens for commands and shall recognize REQA and WUPA Commands.
     * The PICC enters the READY State after it has received a valid
     * REQA or WUPA Command and transmitted its ATQA.
     */
    RC522_PICC_STATE_IDLE = 0,

    /**
     * In the READY State, either the bit frame anticollision or a proprietary
     * anticollision method can be applied. Cascade levels are handled inside
     * this state to get the complete UID.
     * The PICC enters the ACTIVE State when it is selected with its complete UID.
     */
    RC522_PICC_STATE_READY,

    /**
     * In the ACTIVE State, the PICC listens to any higher layer message.
     * The PICC enters the HALT State when a valid HLTA Command is received.
     *
     * NOTE:
     * In the higher layer protocol, specific commands may be defined
     * to return the PICC to its HALT State
     */
    RC522_PICC_STATE_ACTIVE,

    /**
     * In the HALT State, the PICC shall respond only to a WUPA Command.
     *
     * The PICC enters the READY State after it has received a valid
     * WUPA Command and transmitted its ATQA.
     */
    RC522_PICC_STATE_HALT,
} rc522_picc_state_t;

/**
 * All RFU bits shall be set to zero.
 */
typedef struct
{
    uint8_t rfu4          :4;
    uint8_t prop_coding   :4;
    uint8_t uid_size      :2; // 00b - single (4 bytes), 01b - double (7 bytes), 10b - triple (10 bytes)
    uint8_t rfu1          :1;
    uint8_t anticollision :5;
} rc522_picc_atqa_desc_t;

typedef struct
{
    rc522_picc_atqa_desc_t atqa;
    rc522_picc_uid_t uid;
    uint8_t sak;
    rc522_picc_type_t type;
    rc522_picc_state_t state;
} rc522_picc_t;

char *rc522_picc_type_name(rc522_picc_type_t type);

esp_err_t rc522_picc_uid_to_str(rc522_picc_uid_t *uid, char buffer[RC522_PICC_UID_STR_BUFFER_SIZE]);

#ifdef __cplusplus
}
#endif
