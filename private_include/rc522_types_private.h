#pragma once

#include <esp_log.h>
#include <esp_check.h>
#include "rc522_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC522_LOG_TAG "rc522"

#define RC522_LOG_DEFINE_BASE() static const char *TAG = RC522_LOG_TAG

#define RC522_LOG_LEVEL esp_log_level_get(RC522_LOG_TAG)

#define RC522_LOG(esp_log_foo, format, ...) esp_log_foo(TAG, "%s(%d): " format, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define RC522_LOGE(format, ...)             RC522_LOG(ESP_LOGE, format, ##__VA_ARGS__)
#define RC522_LOGW(format, ...)             RC522_LOG(ESP_LOGW, format, ##__VA_ARGS__)
#define RC522_LOGI(format, ...)             RC522_LOG(ESP_LOGI, format, ##__VA_ARGS__)
#define RC522_LOGD(format, ...)             RC522_LOG(ESP_LOGD, format, ##__VA_ARGS__)
#define RC522_LOGV(format, ...)             RC522_LOG(ESP_LOGV, format, ##__VA_ARGS__)
#define RC522_LOG_WRITE(format, ...)        esp_log_write(ESP_LOG_INFO, TAG, format, ##__VA_ARGS__)

#define RC522_RETURN_ON_ERROR(x)                                                                                       \
    do {                                                                                                               \
        esp_err_t err_rc_ = (x);                                                                                       \
        if (unlikely(err_rc_ != ESP_OK)) {                                                                             \
            ESP_LOGE(TAG, "%s(%d): %04x", __FUNCTION__, __LINE__, err_rc_);                                            \
            return err_rc_;                                                                                            \
        }                                                                                                              \
    }                                                                                                                  \
    while (0)

#define RC522_RETURN_ON_FALSE(a, err_code) ESP_RETURN_ON_FALSE(a, err_code, TAG, "")
#define RC522_CHECK_AND_RETURN(a, ret_val) ESP_RETURN_ON_FALSE(!(a), ret_val, TAG, #a)
#define RC522_CHECK(a)                     RC522_CHECK_AND_RETURN(a, ESP_ERR_INVALID_ARG)

#define RC522_RETURN_ON_ERROR_SILENTLY(x)                                                                              \
    do {                                                                                                               \
        esp_err_t err_rc_ = (x);                                                                                       \
        if (unlikely(err_rc_ != ESP_OK)) {                                                                             \
            return err_rc_;                                                                                            \
        }                                                                                                              \
    }                                                                                                                  \
    while (0)

typedef enum
{
    RC522_STATE_UNDEFINED = 0,
    RC522_STATE_CREATED,
    RC522_STATE_SCANNING, /*<! Scanning for nearby PICCs */
    RC522_STATE_PAUSED,
} rc522_state_t;

struct rc522
{
    rc522_config_t *config;               /*<! Configuration */
    bool exit_requested;                  /*<! Indicates whether polling task exit is requested */
    TaskHandle_t task_handle;             /*<! Handle of task */
    esp_event_loop_handle_t event_handle; /*<! Handle of event loop */
    rc522_state_t state;                  /*<! Current state */
};

#define FREE(ptr)                                                                                                      \
    if ((ptr) != NULL) {                                                                                               \
        free(ptr);                                                                                                     \
        (ptr) = NULL;                                                                                                  \
    }

#ifdef __cplusplus
}
#endif
