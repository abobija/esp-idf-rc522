#pragma once

#include "rc522_types.h"
#include "rc522_picc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC522_ERR_MIFARE_BASE                            (ESP_ERR_RC522_BASE + 100)
#define RC522_ERR_MIFARE_ACCESS_BITS_INTEGRITY_VIOLATION (RC522_ERR_MIFARE_BASE + 1)
#define RC522_ERR_MIFARE_VALUE_BLOCK_INTEGRITY_VIOLATION (RC522_ERR_MIFARE_BASE + 2)

#define RC522_MIFARE_KEY_SIZE (6)
#define RC522_MIFARE_DEFAULT_KEY_VALUE                                                                                 \
    {                                                                                                                  \
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF                                                                             \
    }

typedef struct
{
    uint8_t number_of_sectors;
} rc522_mifare_t;

typedef enum
{
    RC522_MIFARE_KEY_A,
    RC522_MIFARE_KEY_B,
} rc522_mifare_key_type_t;

typedef struct
{
    uint8_t value[RC522_MIFARE_KEY_SIZE];
} rc522_mifare_key_t;

typedef struct
{
    uint8_t index;            // Zero-based index of Sector
    uint8_t number_of_blocks; // Total number of blocks inside of Sector
    uint8_t block_0_address;  // Zero-based index of the first Block inside of MIFARE memory
} rc522_mifare_sector_t;

typedef struct
{
    /**
     * [3] Access bits for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
     * [2] Access bits for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
     * [1] Access bits for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
     * [0] Access bits for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
     *
     * Bits of each element are in next format:
     *
     * | 7 | 6 | 5 | 4 | 3 |  2 |  1 |  0 |
     * | - | - | - | - | - | -- | -- | -- |
     * | 0 | 0 | 0 | 0 | 0 | C1 | C2 | C3 |
     *
     */
    uint8_t access_bits[4];
} rc522_mifare_sector_trailer_t;

typedef struct
{
    int32_t value;
    uint8_t address;
} rc522_mifare_value_block_t;

bool rc522_mifare_type_is_classic_compatible(rc522_picc_type_t type);

esp_err_t rc522_mifare_info(rc522_picc_t *picc, rc522_mifare_t *mifare);

esp_err_t rc522_mifare_sector_info(uint8_t sector_index, rc522_mifare_sector_t *result);

esp_err_t rc522_mifare_auth(rc522_handle_t rc522, rc522_picc_t *picc, rc522_mifare_key_type_t key_type,
    uint8_t block_addr, rc522_mifare_key_t *key);

esp_err_t rc522_mifare_autha(rc522_handle_t rc522, rc522_picc_t *picc, uint8_t block_addr, rc522_mifare_key_t *key);

esp_err_t rc522_mifare_authb(rc522_handle_t rc522, rc522_picc_t *picc, uint8_t block_addr, rc522_mifare_key_t *key);

esp_err_t rc522_mifare_read(
    rc522_handle_t rc522, rc522_picc_t *picc, uint8_t block_addr, uint8_t *buffer, uint8_t *buffer_length);

esp_err_t rc522_mifare_parse_sector_trailer(uint8_t *bytes, rc522_mifare_sector_trailer_t *trailer);

bool rc522_mifare_block_is_value(uint8_t access_bits);

esp_err_t rc522_mifare_parse_value_block(uint8_t *bytes, rc522_mifare_value_block_t *block);

esp_err_t rc522_mifare_transactions_end(rc522_handle_t rc522, rc522_picc_t *picc);

#ifdef __cplusplus
}
#endif
