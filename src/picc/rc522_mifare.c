#include "rc522_types_private.h"
#include "picc/rc522_mifare.h"

RC522_LOG_DEFINE_BASE();

esp_err_t rc522_mifare_dump_data_to_log(rc522_handle_t rc522, rc522_picc_t *picc, uint8_t *key, uint8_t key_length)
{
    // TODO: Implement
    RC522_LOGW("Not implemented");

    return ESP_OK;
}
