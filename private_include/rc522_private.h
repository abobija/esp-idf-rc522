#pragma once

#include "rc522.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    RC522_STATE_UNDEFINED = 0,
    RC522_STATE_CREATED,
    RC522_STATE_SCANNING,
    RC522_STATE_PAUSED,
} rc522_state_t;

struct rc522
{
    rc522_config_t *config;               /*<! Configuration */
    bool task_running;                    /*<! Indicates whether rc522 task is running or not */
    TaskHandle_t task_handle;             /*<! Handle of task */
    esp_event_loop_handle_t event_handle; /*<! Handle of event loop */
    rc522_state_t state;                  /*<! Current state */
    bool tag_was_present_last_time;
};

#define FREE(ptr)        \
    if ((ptr) != NULL) { \
        free(ptr);       \
        (ptr) = NULL;    \
    }

uint32_t rc522_millis();

#ifdef __cplusplus
}
#endif
