/** Functions to support interfacing with NXP tags
 *
 * This header defines functions for working with ISO14443-3 compliant NXP
 * PICCs, primarily the Ultralight and NTAG families. These PICCs use
 * predominately the same command set, with some small variations.
 *
 * For MIFARE Classic PICCs, see `include/picc/rc522_mifare.h`.
 *
 * Authentication:
 *   Ultralight {C,AES}: Key-based (3DES and AES respectively)
 *   Ultralight EV1, NTAG 21x: Password-based
 *
 * Some functions have been commented out; ideally these would be implemented
 * at some stage to implement full support for these PICC functions, but I don't
 * have any of the actual PICCs to properly test with.
 *
 * CMD/PICC     | UL      | UL C    | UL EV1  | UL AES  | UL Nano | NTAG21x |
 * -------------|---------|---------|---------|---------|---------|---------|
 * GET_VERSION  |         |         |       y |       y |       y |       y |
 * READ         |       y |       y |       y |       y |       y |       y |
 * FAST_READ    |         |         |       y |       y |         |       y |
 * WRITE        |       y |       y |       y |       y |       y |       y |
 * COMPAT_WRITE |       y |       y |       y |         |       y |       y |
 * READ_CNT     |         |         |       3 |       3 |         |       1 |
 * INCR_CNT     |         |         |       y |       y |         |         |
 * READ_SIG     |         |         |      32 |      48 |      32 |      32 | (bytes)
 * WRITE_SIG    |         |         |         |       y |       y |         |
 * LOCK_SIG     |         |         |         |       y |       y |         |
 * AUTHENTICATE |         |    3DES |         |     AES |         |         |
 * PWD_AUTH     |         |         |       y |         |         |       y |
 * VCSL         |         |         |       y |       y |         |         |
 */

#pragma once

#include "rc522_picc.h"
#include "rc522_types.h"

#ifdef __cplusus
extern "C" {
#endif

#define RC522_ERR_NXP_BASE                  (RC522_ERR_BASE + 0XF0)
#define RC522_ERR_NXP_NACK                  (RC522_ERR_NXP_BASE + 1)
#define RC522_ERR_NXP_AUTHENTICATION_FAILED (RC522_ERR_NXP_BASE + 2)
#define RC522_ERR_NXP_PACK_MISMATCH         (RC522_ERR_NXP_BASE + 3)

#define RC522_NXP_PWD_SIZE  4
#define RC522_NXP_PACK_SIZE 2
#define RC522_NXP_PAGE_SIZE 4
#define RC522_NXP_READ_SIZE (RC522_NXP_PAGE_SIZE * 4)

extern const uint8_t RC522_NXP_DEFAULT_PWD[RC522_NXP_PWD_SIZE];
extern const uint8_t RC522_NXP_DEFAULT_PACK[RC522_NXP_PACK_SIZE];

enum
{
    /**
     * Get PICC information.
     */
    RC522_NXP_GET_VERSION = 0x60,

    /**
     * Read four pages from memory.
     */
    RC522_NXP_READ = 0x30,

    /**
     * Read pages from START to END in memory.
     */
    RC522_NXP_FAST_READ = 0x3A,

    /**
     * Write a page to memory.
     */
    RC522_NXP_WRITE = 0xA2,

    /**
     * Write the first 4 bytes of 16 to a page in memory.
     */
    RC522_NXP_COMPAT_WRITE = 0xA0,

    /**
     * Read an onboard counter.
     */
    RC522_NXP_READ_CNT = 0x39,

    /**
     * Increment an onboard counter.
     */
    RC522_NXP_INCR_CNT = 0xA5,

    /**
     * Read the originality signature.
     */
    RC522_NXP_READ_SIG = 0x3C,

    /**
     * Write a new originality signature.
     */
    RC522_NXP_WRITE_SIG = 0xA9,

    /**
     * Control access to the originality signature.
     */
    RC522_NXP_LOCK_SIG = 0xAC,

    /**
     * Perform key-based authentication, stage 1.
     */
    RC522_NXP_AUTHENTICATE = 0x1A,

    /**
     * Perform key-based authentication, stage 2.
     */
    RC522_NXP_AUTHENTICATE_2 = 0xAF,

    /**
     * Perform password-based authentication.
     */
    RC522_NXP_PWD_AUTH = 0x1B,

    RC522_NXP_VCSL = 0x4B,
};

/**
 * L3 GET_VERSION: product_type field
 */
typedef enum
{
    RC522_NXP_PRODUCT_TYPE_UNKNOWN,
    RC522_NXP_PRODUCT_TYPE_UL = 0x03,
    RC522_NXP_PRODUCT_TYPE_NTAG = 0x04,
} rc522_nxp_product_type_t;

/**
 * L3 GET_VERSION: major_version field
 */
typedef enum
{
    RC522_NXP_MAJ_VER_UNKNOWN = 0,
    RC522_NXP_MAJ_VER_NTAG21 = 0x01,
    RC522_NXP_MAJ_VER_UL_EV1 = 0x01,
    RC522_NXP_MAJ_VER_UL_NANO = 0x02,
    RC522_NXP_MAJ_VER_UL_AES = 0x04,
} rc522_nxp_major_version_t;

/**
 * L3 GET_VERSION result
 */
typedef struct
{
    uint8_t header;
    uint8_t vendor;
    rc522_nxp_product_type_t product_type :8;
    uint8_t product_subtype;
    rc522_nxp_major_version_t major_version :8;
    uint8_t minor_version;
    uint8_t storage_size;
    uint8_t protocol_type;
} rc522_nxp_picc_version_t;

/**
 * L3 FAST_READ result
 *
 * Contains a buffer, the size of the buffer, and the amount of space occupied
 * by the data after a read
 */
typedef struct
{
    uint8_t *bytes;
    uint8_t buffer_size;
    uint8_t read_size;
} rc522_nxp_fast_read_data_t;

/**
 * Variable-length protection signature
 *
 * UL EV1: 32 bytes
 * UL AES: 48 bytes
 * UL Nano: 32 bytes
 * NTAG213/215/216: 32 bytes
 */
typedef struct
{
    uint8_t *bytes;
    uint8_t buffer_size;
    uint8_t sig_size;
} rc522_nxp_sig_t;

/**
 * @brief Determine the total memory size of an NXP PICC
 *
 * Given an NXP PICC type, returns the total size of the memory in pages. This
 * is not equivalent to generic user memory.
 *
 * For invalid types, unknown types, or types without pages, returns 0.
 *
 * @sa rc522_nxp_get_user_page_count()
 */
uint8_t rc522_nxp_get_page_count(rc522_picc_type_t type);

/**
 * @brief Determine the user memory size of an NXP PICC
 *
 * Given an NXP PICC type (RC522_PICC_TYPE_MIFARE_UL_* or RC522_PICC_TYPE_NTAG*),
 * returns the total size of the user memory in pages. The user memory always
 * starts at page 4.
 *
 * For invalid types, unknown types, or types without pages, returns 0.
 */
uint8_t rc522_nxp_get_user_page_count(rc522_picc_type_t type);

/**
 * @brief Determine the start page of user memory for an NXP PICC
 *
 * Currently this is 0x04 for all PICCs. It's unlikely to change in future
 * for compatibility reasons, but this function exists in case (and also
 * as a reference).
 *
 * For invalid or unknown types, returns 0.
 */
uint8_t rc522_nxp_get_user_mem_start(rc522_picc_type_t type);

/**
 * @brief Determine the final page of user memory for an NXP PICC
 *
 * Returns the address of the final page of user memory for a given PICC
 * type. This is the last page that can be safely written to without changing
 * the configuration of the PICC.
 *
 * For invalid or unknown types, returns 0.
 */
uint8_t rc522_nxp_get_user_mem_end(rc522_picc_type_t type);

/**
 * @brief Determine the type of an NXP PICC
 *
 * The Ultralight and NTAG families both use SAK=0, and need further processing
 * to identify. This function attempts to determine the PICC in use following
 * the procedure in NXP Application Note 10833.
 */
esp_err_t rc522_nxp_get_type(const rc522_handle_t rc522, const rc522_picc_t *picc, rc522_picc_type_t *out_type);

/**
 * @brief NXP Level 3 GET_VERSION
 *
 * If supported, returns information about the PICC, including manufacturer,
 * product, and memory size.
 */
esp_err_t rc522_nxp_get_version(
    const rc522_handle_t rc522, const rc522_picc_t *picc, rc522_nxp_picc_version_t *out_version);

/**
 * @brief NXP Level 3 READ
 *
 * Reads 4 pages (16 bytes) from a given page address on the PICC.
 */
esp_err_t rc522_nxp_read(
    const rc522_handle_t rc522, const rc522_picc_t *picc, uint8_t address, uint8_t out_buffer[RC522_NXP_PAGE_SIZE * 4]);

/**
 * NXP Level 3 FAST_READ
 *
 * If supported, allows a variable number of pages to be read at once, instead
 * of the fixed 4 in READ.
 *
 * @param start Page address to start reading from
 * @param end Page address to end reading (inclusive)
 * @param out_buffer Output buffer; should be at least (end-start+1) * 4 bytes
 *
 * Supported PICCs: UL EV1, UL AES, NTAG21x
 */
esp_err_t rc522_nxp_fast_read(const rc522_handle_t rc522, const rc522_picc_t *picc, uint8_t start, uint8_t end,
    rc522_nxp_fast_read_data_t *out_buffer);

/**
 * @brief NXP Level 3 WRITE
 *
 * Writes 4 bytes of data to a single page.
 */
esp_err_t rc522_nxp_write(
    const rc522_handle_t rc522, const rc522_picc_t *picc, uint8_t address, const uint8_t buffer[RC522_NXP_PAGE_SIZE]);

/**
 * @brief NXP Level 3 READ_CNT
 *
 * If supported, reads a counter from the PICC. Some PICCs have more than one
 * counter; if so counter_no specifies the counter to address.
 *
 * Supported PICCs: UL EV1 (3), UL AES (3), NTAG21x (1)
 */
esp_err_t rc522_nxp_read_cnt(
    const rc522_handle_t rc522, const rc522_picc_t *picc, uint8_t counter_no, uint32_t *out_count);

/**
 * @brief NXP Level 3 INCR_CNT
 *
 * If supported, increments a counter on the PICC. If the PICC has more than one
 * counter, counter_no specifies the counter to target.
 *
 * TODO
 */
// esp_err_t rc522_nxp_incr_cnt(const rc522_handle_t rc522, const rc522_picc_t *picc,
//         uint8_t counter_no);

/**
 * @brief Checks for support for AUTHENTICATE in L3 on a PICC
 *
 * Used to determine if a chip supports AUTHENTICATE, to differentiate between
 * Ultralight and Ultralight C. Does not perform actual authentication.
 */
esp_err_t rc522_nxp_keyauth_supported(const rc522_handle_t rc522, const rc522_picc_t *picc);

/**
 * @brief NXP Level 3 AUTHENTICATE
 *
 * If supported, performs key-based authentication with a PICC. Note support
 * for DES and/or AES algorithms is required.
 *
 * TODO
 */
// esp_err_t rc522_nxp_key_auth(const rc522_handle_t rc522, const rc522_picc_t *picc,
//         rc522_nxp_key_t *key);

/**
 * @brief NXP Level 3 PWD_AUTH
 *
 * If supported, performs password-based authentication with a PICC.
 *
 * @param pwd Password to send to pick
 * @param pack Password ACKnowledgement expected to be returned by PICC
 * @param[out] out_state PICC state on return. Correct PWD with mismatched PACK
 *   will still result in AUTHENTICATED
 *
 * Supported PICCs: UL EV1, NTAG21x
 */
esp_err_t rc522_nxp_pwd_auth(const rc522_handle_t rc522, const rc522_picc_t *picc,
    const uint8_t pwd[RC522_NXP_PWD_SIZE], const uint8_t pack[RC522_NXP_PACK_SIZE], rc522_picc_state_t *out_state);

/**
 * @brief NXP Level 3 READ_SIG
 *
 * If supported, reads the ECC signature from the PICC. This signature can
 * be used to verify originality. Does not actually perform verification.
 *
 * TODO: Implement verification per AN11350?
 *
 * Supported PICCs: UL EV1, UL AES, UL Nano, NTAG21x
 */
esp_err_t rc522_nxp_read_sig(const rc522_handle_t rc522, const rc522_picc_t *picc, rc522_nxp_sig_t *out_sig);

/**
 * @brief NXP Level 3 WRITE_SIG
 *
 * If supported, writes a new signature to the PICC.
 *
 * TODO
 */
// esp_err_t rc522_nxp_write_sig(const rc522_handle_t rc522, const rc522_picc_t *picc,
//         rc522_nxp_sig_t *sig);

/**
 * @brief NXP Level 3 LOCK_SIG
 *
 * If supported, controls the originality signature lock on the PICC.
 *
 * @param argument 0x00 for unlock, 0x01 for lock, 0x02 for permanent lock
 *
 * TODO
 */
// esp_err_t rc522_nxp_lock_sig(const rc522_handle_t rc522, const rc522_picc_t *picc,
//         rc522_nxp_sig_arg_t argument);

#ifdef __cplusplus
}
#endif
