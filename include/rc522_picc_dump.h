#pragma once

#include "rc522_types.h"
#include "rc522_picc.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t rc522_picc_dump_to_log(rc522_handle_t rc522, rc522_picc_t *picc);

#ifdef __cplusplus
}
#endif
