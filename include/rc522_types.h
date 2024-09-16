#pragma once

#include <esp_err.h>
#include <esp_event.h>
#include <inttypes.h>
#include "rc522_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC522_DEFAULT_TASK_THROTTLING_MS (125)
#define RC522_DEFAULT_TASK_STACK_SIZE    (4 * 1024)
#define RC522_DEFAULT_TASK_PRIORITY      (3)

#define ESP_ERR_RC522_BASE      (0xF522)
#define ESP_ERR_RC522_COLLISION (ESP_ERR_RC522_BASE + 1)
#define ESP_ERR_RC522_CRC_WRONG (ESP_ERR_RC522_BASE + 3)

typedef struct rc522 *rc522_handle_t;

typedef struct
{
    rc522_driver_handle_t driver;
    uint16_t task_throttling_ms; /*<! Delay (in milliseconds) between polls */
    size_t task_stack_size;      /*<! Stack size of rc522 task */
    uint8_t task_priority;       /*<! Priority of rc522 task */
} rc522_config_t;

typedef enum
{
    RC522_EVENT_ANY = ESP_EVENT_ANY_ID,
    RC522_EVENT_NONE,

    /**
     * Card is detected. UID is available.
     * PICC is in the field of the PCD and its listens to any higher level message.
     */
    RC522_EVENT_PICC_ACTIVE,
} rc522_event_t;

#ifdef __cplusplus
}
#endif
