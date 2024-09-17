#pragma once

#include "rc522_types.h"
#include "rc522_picc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC522_ERR_MIFARE_BASE                            (ESP_ERR_RC522_BASE + 100)
#define RC522_ERR_MIFARE_ACCESS_BITS_INTEGRITY_VIOLATION (RC522_ERR_MIFARE_BASE + 1)
#define RC522_ERR_MIFARE_VALUE_BLOCK_INTEGRITY_VIOLATION (RC522_ERR_MIFARE_BASE + 2)
#define RC522_ERR_MIFARE_NACK                            (RC522_ERR_MIFARE_BASE + 3)

#define RC522_MIFARE_MAX_SECTOR_INDEX (39) // No MIFARE Classic has more than 40 sectors
#define RC522_MIFARE_BLOCK_SIZE       (16)
#define RC522_MIFARE_ACK              (0x0A) // The MIFARE Classic uses a 4 bit ACK/NAK. Any other value than 0xA is NAK.
#define RC522_MIFARE_KEY_SIZE         (6)
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
    RC522_MIFARE_KEY_A = 0,
    RC522_MIFARE_KEY_B,
} rc522_mifare_key_type_t;

typedef struct
{
    rc522_mifare_key_type_t type;
    uint8_t value[RC522_MIFARE_KEY_SIZE];
} rc522_mifare_key_t;

typedef struct
{
    uint8_t index;            // Zero-based index of Sector
    uint8_t number_of_blocks; // Total number of blocks inside of Sector
    uint8_t block_0_address;  // Zero-based index of the first Block inside of MIFARE memory
} rc522_mifare_sector_t;

typedef enum
{
    RC522_MIFARE_BLOCK_UNDEFINED = 0,
    RC522_MIFARE_BLOCK_TRAILER,
    RC522_MIFARE_BLOCK_DATA,
    RC522_MIFARE_BLOCK_VALUE,
    RC522_MIFARE_BLOCK_MANUFACTURER_DATA,
} rc522_mifare_block_type_t;

typedef struct
{
    uint8_t group;
    uint8_t c1 :1;
    uint8_t c2 :1;
    uint8_t c3 :1;
} rc522_mifare_access_bits_t;

typedef struct
{
    int32_t value;
    uint8_t address;
} rc522_mifare_value_block_t;

typedef struct
{
    uint8_t sector_index;
    uint8_t address;
    uint8_t bytes[RC522_MIFARE_BLOCK_SIZE];
    rc522_mifare_block_type_t type;
    rc522_mifare_access_bits_t access_bits;
    esp_err_t access_bits_err;
    rc522_mifare_value_block_t *value;
    esp_err_t value_err;
} rc522_mifare_sector_block_t;

typedef esp_err_t (*rc522_mifare_sector_block_iterator)(rc522_mifare_sector_block_t *block);
typedef esp_err_t (*rc522_mifare_transaction_handler)(rc522_handle_t rc522, rc522_picc_t *picc);

bool rc522_mifare_type_is_classic_compatible(rc522_picc_type_t type);

esp_err_t rc522_mifare_info(rc522_picc_t *picc, rc522_mifare_t *mifare);

esp_err_t rc522_mifare_auth(rc522_handle_t rc522, rc522_picc_t *picc, uint8_t block_addr, rc522_mifare_key_t *key);

esp_err_t rc522_mifare_read(
    rc522_handle_t rc522, rc522_picc_t *picc, uint8_t block_addr, uint8_t buffer[RC522_MIFARE_BLOCK_SIZE]);

esp_err_t rc522_mifare_write(
    rc522_handle_t rc522, rc522_picc_t *picc, uint8_t block_addr, uint8_t buffer[RC522_MIFARE_BLOCK_SIZE]);

esp_err_t rc522_mifare_transactions_end(rc522_handle_t rc522, rc522_picc_t *picc);

esp_err_t rc522_mifare_handle_as_transaction(
    rc522_mifare_transaction_handler transaction_handler, rc522_handle_t rc522, rc522_picc_t *picc);

esp_err_t rc522_mifare_iterate_sector_blocks(rc522_handle_t rc522, rc522_picc_t *picc, uint8_t sector_index,
    rc522_mifare_key_t *key, rc522_mifare_sector_block_iterator iterator);

#ifdef __cplusplus
}
#endif
