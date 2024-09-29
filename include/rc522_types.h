#pragma once

#include <esp_err.h>
#include <esp_event.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "rc522_driver.h"
#include "rc522_picc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC522_X "X"

#define RC522_ERR_BASE                          (0xF522)
#define RC522_ERR_COLLISION                     (RC522_ERR_BASE + 1)
#define RC522_ERR_COLLISION_UNSOLVABLE          (RC522_ERR_BASE + 2)
#define RC522_ERR_CRC_WRONG                     (RC522_ERR_BASE + 3)
#define RC522_ERR_INVALID_ATQA                  (RC522_ERR_BASE + 4)
#define RC522_ERR_RX_TIMEOUT                    (RC522_ERR_BASE + 5)
#define RC522_ERR_RX_TIMER_TIMEOUT              (RC522_ERR_BASE + 6)
#define RC522_ERR_INVALID_SAK                   (RC522_ERR_BASE + 7)
#define RC522_ERR_PICC_POST_HEARTBEAT_MISSMATCH (RC522_ERR_BASE + 8)
#define RC522_ERR_PCD_FIFO_BUFFER_OVERFLOW      (RC522_ERR_BASE + 9)
#define RC522_ERR_PCD_PARITY_CHECK_FAILED       (RC522_ERR_BASE + 10)
#define RC522_ERR_PCD_PROTOCOL_ERROR            (RC522_ERR_BASE + 11)
#define RC522_ERR_RST_PIN_UNUSED                (RC522_ERR_BASE + 12)
#define RC522_ERR_PCD_FIFO_EMPTY                (RC522_ERR_BASE + 13)
#define RC522_ERR_HLTA_NOT_ACKED                (RC522_ERR_BASE + 14)

typedef struct rc522 *rc522_handle_t;

typedef struct
{
    rc522_driver_handle_t driver;
    uint16_t poll_interval_ms;    /*<! Delay (in milliseconds) between polls */
    size_t task_stack_size;       /*<! Stack size of rc522 task */
    uint8_t task_priority;        /*<! Priority of rc522 task */
    SemaphoreHandle_t task_mutex; /*<! Mutex for rc522 task */
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
