#pragma once

#include "rc522_types.h"
#include "rc522_picc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    /**
     * The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
     * The RC522_PICC_CMD_MF_READ and RC522_PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
     *
     * Writes one 4 byte page to the PICC.
     */
    RC522_PICC_CMD_UL_WRITE = 0xA2,

    // Other command are same as for MIFARE Classic
} rc522_mifare_ul_command_t;

esp_err_t rc522_mifare_ul_dump_data_to_log(rc522_handle_t rc522, rc522_picc_t *picc);

#ifdef __cplusplus
}
#endif
