#pragma once

#include "rc522.h"

#ifdef __cplusplus
extern "C" {
#endif

void rc522_task(void *arg);

esp_err_t rc522_stream(rc522_handle_t rc522, const char *data);

#ifdef __cplusplus
}
#endif
