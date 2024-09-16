#pragma once

#include "rc522_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC522_PICC_UID_MAX_SIZE        (10)
#define RC522_PICC_UID_STR_BUFFER_SIZE (RC522_PICC_UID_MAX_SIZE * 3)

typedef struct
{
    uint8_t value[RC522_PICC_UID_MAX_SIZE];
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

typedef struct
{
    bool is_present;
    rc522_picc_uid_t uid;
    uint8_t sak; // Select acknowledge (SAK) byte returned from the PICC after successful selection
    rc522_picc_type_t type;
    uint32_t activated_at_ms;
} rc522_picc_t;

char *rc522_picc_type_name(rc522_picc_type_t type);

esp_err_t rc522_picc_uid_to_str(rc522_picc_uid_t *uid, char *buffer);

#ifdef __cplusplus
}
#endif
