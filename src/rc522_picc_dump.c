#include <string.h>
#include "rc522_types_private.h"
#include "rc522_helpers_private.h"
#include "rc522_picc_private.h"
#include "rc522_picc_dump.h"

#include "picc/rc522_mifare.h"
#include "picc/rc522_mifare_ul.h"

RC522_LOG_DEFINE_BASE();

static esp_err_t rc522_picc_dump_data_to_log(rc522_handle_t rc522, rc522_picc_t *picc)
{
    switch (picc->type) {
        case RC522_PICC_TYPE_MIFARE_MINI:
        case RC522_PICC_TYPE_MIFARE_1K:
        case RC522_PICC_TYPE_MIFARE_4K:
            uint8_t key[6];

            // MIFARE factory key is set to FFFFFFFFFFFFh
            memset(key, 0xFF, sizeof(key));

            return rc522_mifare_dump_data_to_log(rc522, picc, key, sizeof(key));

        case RC522_PICC_TYPE_MIFARE_UL:
            return rc522_mifare_ul_dump_data_to_log(rc522, picc);

        case RC522_PICC_TYPE_ISO_14443_4:
        case RC522_PICC_TYPE_MIFARE_DESFIRE:
        case RC522_PICC_TYPE_ISO_18092:
        case RC522_PICC_TYPE_MIFARE_PLUS:
        case RC522_PICC_TYPE_TNP3XXX:
            RC522_LOGW("Dumping PICC data not implemented for type %02x", picc->type);
            return ESP_ERR_INVALID_ARG;

        default:
            RC522_LOGW("Unknown PICC type %02x", picc->type);
            return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t rc522_picc_dump_to_log(rc522_handle_t rc522, rc522_picc_t *picc)
{
    char uid_str[RC522_PICC_UID_MAX_SIZE * 3];
    rc522_buffer_to_hex_str(picc->uid.bytes, picc->uid.bytes_length, uid_str, sizeof(uid_str));

    ESP_LOGI(TAG, "PICC (uid=%s, sak=%02x, type=%s)", uid_str, picc->sak, rc522_picc_type_name(picc->type));
    RC522_RETURN_ON_ERROR(rc522_picc_dump_data_to_log(rc522, picc));

    return rc522_picc_halta(rc522, picc);
}
