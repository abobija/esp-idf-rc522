#pragma once

#include "rc522_types.h"
#include "rc522_picc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC522_ERR_MIFARE_BASE                             (RC522_ERR_BASE + 0xFF)
#define RC522_ERR_MIFARE_ACCESS_BITS_INTEGRITY_VIOLATION  (RC522_ERR_MIFARE_BASE + 1)
#define RC522_ERR_MIFARE_VALUE_BLOCK_INTEGRITY_VIOLATION  (RC522_ERR_MIFARE_BASE + 2)
#define RC522_ERR_MIFARE_NACK                             (RC522_ERR_MIFARE_BASE + 3)
#define RC522_ERR_MIFARE_AUTHENTICATION_FAILED            (RC522_ERR_MIFARE_BASE + 4)
#define RC522_ERR_MIFARE_SECTOR_TRAILER_WRITE_NOT_ALLOWED (RC522_ERR_MIFARE_BASE + 5)

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

/**
 * @brief Authenticate a block read/write operations.
 *
 * After a successful authentication, communication between
 * the PCD and the PICC is secured with MF1xxS20, MF1xxS70 or MF1xxS50 encryption.
 *
 * Once when read/write operations are authenticated for one block inside of a sector,
 * read/write operations are authenticated for all blocks inside of that sector as well.
 *
 * @note After read/write operations are done, the PICC must be deauthenticated using @c rc522_mifare_deauth().
 *
 * @param rc522 RC522 handle
 * @param picc PICC that is currently selected
 * @param block_address Address of the block to authenticate
 * @param key Key to authenticate with
 */
esp_err_t rc522_mifare_auth(
    const rc522_handle_t rc522, const rc522_picc_t *picc, uint8_t block_address, const rc522_mifare_key_t *key);

/**
 * @brief Read from a block at the given @c block_address on the PICC.
 *
 * @note The block must be authenticated before calling this function.
 *
 * @param rc522 RC522 handle
 * @param picc PICC that is currently selected
 * @param block_address Address of the block to read
 * @param[out] out_buffer Buffer of exactly @c RC522_MIFARE_BLOCK_SIZE bytes to store the retrived data in
 */
esp_err_t rc522_mifare_read(const rc522_handle_t rc522, const rc522_picc_t *picc, uint8_t block_address,
    uint8_t out_buffer[RC522_MIFARE_BLOCK_SIZE]);

/**
 * @brief Write to a block at the given @c block_address on the PICC.
 *
 * @note The block must be authenticated before calling this function.
 *
 * @param rc522 RC522 handle
 * @param picc PICC that is currently selected
 * @param block_address Address of the block to write
 * @param[in] buffer Buffer of exactly @c RC522_MIFARE_BLOCK_SIZE that contains the data to write
 */
esp_err_t rc522_mifare_write(const rc522_handle_t rc522, const rc522_picc_t *picc, uint8_t block_address,
    const uint8_t buffer[RC522_MIFARE_BLOCK_SIZE]);

// }}

// {{ MIFARE_Utility_Functions

/**
 * @brief Deauthenticates read/write operations and allows PCD to perform other commands
 *
 * @param rc522 RC522 handle
 * @param picc PICC that is currently selected
 */
esp_err_t rc522_mifare_deauth(const rc522_handle_t rc522, const rc522_picc_t *picc);

/**
 * @brief Authenticates read/write operations
 */
esp_err_t rc522_mifare_auth_sector(const rc522_handle_t rc522, const rc522_picc_t *picc,
    const rc522_mifare_sector_desc_t *sector_desc, const rc522_mifare_key_t *key);

/**
 * @brief Checks if the PICC is MIFARE Classic
 */
bool rc522_mifare_type_is_classic_compatible(rc522_picc_type_t type);

/**
 * @brief Get MIFARE description (e.g number of sectors)
 */
esp_err_t rc522_mifare_get_desc(const rc522_picc_t *picc, rc522_mifare_desc_t *out_mifare_desc);

/**
 * @brief Get MIFARE sector description (e.g number of blocks, block 0 address)
 */
esp_err_t rc522_mifare_get_sector_desc(uint8_t sector_index, rc522_mifare_sector_desc_t *out_sector_desc);

/**
 * @brief Read and parse MIFARE sector trailer block
 */
esp_err_t rc522_mifare_read_sector_trailer_block(const rc522_handle_t rc522, const rc522_picc_t *picc,
    const rc522_mifare_sector_desc_t *sector_desc, rc522_mifare_sector_block_t *out_trailer);

/**
 * @brief Read and parse MIFARE sector (non-trailer) block
 */
esp_err_t rc522_mifare_read_sector_block(const rc522_handle_t rc522, const rc522_picc_t *picc,
    const rc522_mifare_sector_desc_t *sector_desc, const rc522_mifare_sector_block_t *trailer, uint8_t block_offset,
    rc522_mifare_sector_block_t *out_block);

esp_err_t rc522_mifare_get_number_of_sectors(rc522_picc_type_t type, uint8_t *out_result);

uint8_t rc522_mifare_get_sector_index_by_block_address(uint8_t block_address);

esp_err_t rc522_mifare_get_number_of_blocks_in_sector(uint8_t sector_index, uint8_t *out_result);

esp_err_t rc522_mifare_get_sector_block_0_address(uint8_t sector_index, uint8_t *out_result);

// }}

#ifdef __cplusplus
}
#endif
