#pragma once

#include "rc522_types.h"
#include "rc522_picc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC522_ERR_MIFARE_BASE                            (RC522_ERR_BASE + 0xFF)
#define RC522_ERR_MIFARE_ACCESS_BITS_INTEGRITY_VIOLATION (RC522_ERR_MIFARE_BASE + 1)
#define RC522_ERR_MIFARE_VALUE_BLOCK_INTEGRITY_VIOLATION (RC522_ERR_MIFARE_BASE + 2)
#define RC522_ERR_MIFARE_NACK                            (RC522_ERR_MIFARE_BASE + 3)

#define RC522_MIFARE_SECTOR_INDEX_MAX  (39) // No MIFARE Classic has more than 40 sectors
#define RC522_MIFARE_BLOCK_SIZE        (16)
#define RC522_MIFARE_ACK               (0x0A) // The MIFARE Classic uses a 4 bit ACK/NAK. Any other value than 0xA is NAK.
#define RC522_MIFARE_KEY_SIZE          (6)
#define RC522_MIFARE_KEY_VALUE_DEFAULT 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF

typedef struct
{
    uint8_t number_of_sectors;
} rc522_mifare_desc_t;

typedef struct
{
    uint8_t index;            // Zero-based index of Sector
    uint8_t number_of_blocks; // Total number of blocks inside of Sector
    uint8_t block_0_address;  // Zero-based index of the first Block inside of MIFARE memory
} rc522_mifare_sector_desc_t;

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
    uint8_t c1 :1;
    uint8_t c2 :1;
    uint8_t c3 :1;
} rc522_mifare_access_bits_t;

typedef struct
{
    rc522_mifare_access_bits_t access_bits[4]; // [3] = Trailer, [2] = Block 2, [1] = Block 1, [0] = Block 0
} rc522_mifare_sector_trailer_info_t;

typedef struct
{
    int32_t value; // Value stored in the block
    uint8_t addr;  // Value block address (this is not the memory address)
} rc522_mifare_sector_value_block_info_t;

typedef struct
{
    uint8_t address;
    rc522_mifare_block_type_t type;
    uint8_t bytes[RC522_MIFARE_BLOCK_SIZE];
    union
    {
        rc522_mifare_sector_trailer_info_t trailer_info;   // Valid only if type == RC522_MIFARE_BLOCK_TRAILER
        rc522_mifare_sector_value_block_info_t value_info; // Valid only if type == RC522_MIFARE_BLOCK_VALUE
    };
    rc522_mifare_access_bits_t access_bits;
    esp_err_t error; // e.g. acces bits or value block integrity violation
} rc522_mifare_sector_block_t;

// {{ MIFARE_Specific_Functions

esp_err_t rc522_mifare_auth(rc522_handle_t rc522, rc522_picc_t *picc, uint8_t block_addr, rc522_mifare_key_t *key);

esp_err_t rc522_mifare_read(
    rc522_handle_t rc522, rc522_picc_t *picc, uint8_t block_addr, uint8_t *buffer, uint8_t buffer_size);

esp_err_t rc522_mifare_write(
    rc522_handle_t rc522, rc522_picc_t *picc, uint8_t block_addr, uint8_t *buffer, uint8_t buffer_size);

// }} MIFARE_Specific_Functions

// {{ MIFARE_Utility_Functions

/**
 * @brief Authenticates read/write operations
 */
esp_err_t rc522_mifare_auth_sector(
    rc522_handle_t rc522, rc522_picc_t *picc, rc522_mifare_sector_desc_t *sector_desc, rc522_mifare_key_t *key);

/**
 * @brief Deauthenticates read/write operations and allows PCD to perform other commands
 */
esp_err_t rc522_mifare_deauth(rc522_handle_t rc522, rc522_picc_t *picc);

/**
 * @brief Checks if the PICC is MIFARE Classic
 */
bool rc522_mifare_type_is_classic_compatible(rc522_picc_type_t type);

/**
 * @brief Get MIFARE description (e.g number of sectors)
 */
esp_err_t rc522_mifare_get_desc(rc522_picc_t *picc, rc522_mifare_desc_t *out_mifare_desc);

/**
 * @brief Get MIFARE sector description (e.g number of blocks, block 0 address)
 */
esp_err_t rc522_mifare_get_sector_desc(uint8_t sector_index, rc522_mifare_sector_desc_t *out_sector_desc);

/**
 * @brief Read and parse MIFARE sector trailer block
 */
esp_err_t rc522_mifare_read_sector_trailer_block(rc522_handle_t rc522, rc522_picc_t *picc,
    rc522_mifare_sector_desc_t *sector_desc, rc522_mifare_sector_block_t *out_trailer);

/**
 * @brief Read and parse MIFARE sector (non-trailer) block
 */
esp_err_t rc522_mifare_read_sector_block(rc522_handle_t rc522, rc522_picc_t *picc,
    rc522_mifare_sector_desc_t *sector_desc, rc522_mifare_sector_block_t *trailer, uint8_t block_offset,
    rc522_mifare_sector_block_t *out_block);

// }} MIFARE_Utility_Functions

#ifdef __cplusplus
}
#endif
