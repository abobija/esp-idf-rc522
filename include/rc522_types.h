#pragma once

#include <esp_err.h>
#include <esp_event.h>
#include <inttypes.h>
#include "rc522_driver.h"
#include "rc522_picc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC522_X "X"

#define ESP_ERR_RC522_BASE                     (0xF522)
#define ESP_ERR_RC522_COLLISION                (ESP_ERR_RC522_BASE + 1)
#define ESP_ERR_RC522_COLLISION_UNSOLVABLE     (ESP_ERR_RC522_BASE + 2)
#define ESP_ERR_RC522_CRC_WRONG                (ESP_ERR_RC522_BASE + 3)
#define ESP_ERR_RC522_INVALID_ATQA             (ESP_ERR_RC522_BASE + 4)
#define ESP_ERR_RC522_RX_TIMEOUT               (ESP_ERR_RC522_BASE + 5)
#define ESP_ERR_RC522_RX_TIMER_TIMEOUT         (ESP_ERR_RC522_BASE + 6)
#define ESP_ERR_RC522_INVALID_SAK              (ESP_ERR_RC522_BASE + 7)
#define ESP_ERR_RC522_PICC_HEARTBEAT_CHANGES   (ESP_ERR_RC522_BASE + 8)
#define ESP_ERR_RC522_PCD_FIFO_BUFFER_OVERFLOW (ESP_ERR_RC522_BASE + 9)
#define ESP_ERR_RC522_PCD_PARITY_CHECK_FAILED  (ESP_ERR_RC522_BASE + 10)
#define ESP_ERR_RC522_PCD_PROTOCOL_ERROR       (ESP_ERR_RC522_BASE + 11)

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
    RC522_EVENT_PICC_STATE_CHANGED,
} rc522_event_t;

#ifdef __cplusplus
}
#endif
