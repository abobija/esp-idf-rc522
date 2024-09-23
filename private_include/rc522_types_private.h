#pragma once

#include <esp_log.h>
#include <esp_check.h>
#include <esp_bit_defs.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include "rc522_types.h"
#include "rc522_picc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC522_LOG_TAG "rc522"

#define RC522_POLL_INTERVAL_MS_DEFAULT (120)
#define RC522_POLL_INTERVAL_MS_MIN     (50)
#define RC522_TASK_STACK_SIZE_DEFAULT  (4 * 1024)
#define RC522_TASK_PRIORITY_DEFAULT    (3)

#define RC522_TASK_STOPPED_BIT (BIT0)

#define RC522_LOG_LEVEL LOG_LOCAL_LEVEL

typedef enum
{
    RC522_STATE_UNDEFINED = 0,
    RC522_STATE_CREATED,
    RC522_STATE_POLLING, /*<! Scanning for nearby PICCs */
    RC522_STATE_PAUSED,
} rc522_state_t;

struct rc522
{
    rc522_config_t *config;               /*<! Configuration */
    bool exit_requested;                  /*<! Indicates whether polling task exit is requested */
    TaskHandle_t task_handle;             /*<! Handle of task */
    esp_event_loop_handle_t event_handle; /*<! Handle of event loop */
    rc522_state_t state;                  /*<! Current state */
    rc522_picc_t picc;
    EventGroupHandle_t bits;
};

typedef struct
{
    uint8_t *ptr;
    uint8_t length;
} rc522_bytes_t;

#define RC522_LOG_DEFINE_BASE() static const char *TAG = RC522_LOG_TAG

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
            ESP_LOGE(TAG, "%s(%d): %04" RC522_X "", __FUNCTION__, __LINE__, err_rc_);                                  \
            return err_rc_;                                                                                            \
        }                                                                                                              \
    }                                                                                                                  \
    while (0)

#define RC522_RETURN_ON_FALSE(a, err_code)   ESP_RETURN_ON_FALSE(a, err_code, TAG, "")
#define RC522_CHECK_WITH_MESSAGE(a, message) ESP_RETURN_ON_FALSE(!(a), ESP_ERR_INVALID_ARG, TAG, message)
#define RC522_CHECK_AND_RETURN(a, ret_val)   ESP_RETURN_ON_FALSE(!(a), ret_val, TAG, #a)
#define RC522_CHECK(a)                       ESP_RETURN_ON_FALSE(!(a), ESP_ERR_INVALID_ARG, TAG, #a)
#define RC522_CHECK_BYTES(b)                 RC522_CHECK(b == NULL || (b)->ptr == NULL || (b)->length < 1)

#define RC522_RETURN_ON_ERROR_SILENTLY(x)                                                                              \
    do {                                                                                                               \
        esp_err_t err_rc_ = (x);                                                                                       \
        if (unlikely(err_rc_ != ESP_OK)) {                                                                             \
            return err_rc_;                                                                                            \
        }                                                                                                              \
    }                                                                                                                  \
    while (0)

#ifdef __cplusplus
}
#endif
