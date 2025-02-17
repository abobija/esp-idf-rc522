#pragma once

#include "rc522_pcd_internal.h"
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
     * TODO:        move it from here in that case
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
     * AUTHenticate command for Ultralight C & AES.
     *
     */
    RC522_PICC_CMD_UL_AUTH = 0x1A,

    /**
     * GET Version level 3 command for Ultralight EV1 & NANO, and NTAGs.
     */
    RC522_PICC_CMD_GETV = 0x60,
} rc522_picc_command_t;

typedef struct
{
    rc522_pcd_command_t pcd_command;
    rc522_bytes_t bytes;
    uint8_t expected_interrupts;
    uint8_t rx_align;
    uint8_t valid_bits;
    bool check_crc;
} rc522_picc_transaction_t;

typedef struct rc522_picc_transaction_context rc522_picc_transaction_context_t;

typedef struct
{
    rc522_bytes_t bytes;
    uint8_t valid_bits;
} rc522_picc_transaction_result_t;

esp_err_t rc522_picc_send(const rc522_handle_t rc522, const rc522_picc_transaction_t *transaction,
    rc522_picc_transaction_context_t *out_context);

esp_err_t rc522_picc_transceive(const rc522_handle_t rc522, const rc522_picc_transaction_t *transaction,
    rc522_picc_transaction_result_t *out_result);

esp_err_t rc522_picc_reqa(const rc522_handle_t rc522, rc522_picc_atqa_desc_t *out_atqa);

esp_err_t rc522_picc_wupa(const rc522_handle_t rc522, rc522_picc_atqa_desc_t *out_atqa);

esp_err_t rc522_picc_select(
    const rc522_handle_t rc522, rc522_picc_uid_t *out_uid, uint8_t *out_sak, bool skip_anticoll);

esp_err_t rc522_picc_halta(const rc522_handle_t rc522, rc522_picc_t *picc);

esp_err_t rc522_picc_heartbeat(
    const rc522_handle_t rc522, const rc522_picc_t *picc, rc522_picc_uid_t *out_uid, uint8_t *out_sak);

rc522_picc_type_t rc522_picc_get_type(const rc522_picc_t *picc);

esp_err_t rc522_picc_set_state(
    const rc522_handle_t rc522, rc522_picc_t *picc, rc522_picc_state_t new_state, bool fire_event);

#ifdef __cplusplus
}
#endif
