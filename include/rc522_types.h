#pragma once

#include <esp_err.h>
#include <esp_event.h>
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RC522_DEFAULT_SCAN_INTERVAL_MS    (125)
#define RC522_DEFAULT_TASK_STACK_SIZE     (4 * 1024)
#define RC522_DEFAULT_TASK_STACK_PRIORITY (4)

#define ESP_ERR_RC522_BASE        (0xF522)
#define ESP_ERR_RC522_COLLISION   (ESP_ERR_RC522_BASE + 1)
#define ESP_ERR_RC522_MIFARE_NACK (ESP_ERR_RC522_BASE + 2)
#define ESP_ERR_RC522_CRC_WRONG   (ESP_ERR_RC522_BASE + 3)

typedef struct rc522 *rc522_handle_t;

typedef esp_err_t (*rc522_send_handler_t)(uint8_t *buffer, uint8_t length);
typedef esp_err_t (*rc522_receive_handler_t)(uint8_t address, uint8_t *buffer, uint8_t length);

typedef struct
{
    rc522_send_handler_t send_handler;
    rc522_receive_handler_t receive_handler;
    uint16_t scan_interval_ms; /*<! How fast will ESP32 scan for nearby PICCs, in miliseconds */
    size_t task_stack_size;    /*<! Stack size of rc522 task */
    uint8_t task_priority;     /*<! Priority of rc522 task */
} rc522_config_t;

typedef enum
{
    RC522_EVENT_ANY = ESP_EVENT_ANY_ID,
    RC522_EVENT_NONE,
    RC522_EVENT_PICC_SELECTED, /*<! PICC selected (scanned) */
} rc522_event_t;

typedef struct
{
    rc522_handle_t rc522;
    void *ptr;
} rc522_event_data_t;

#ifdef __cplusplus
}
#endif
