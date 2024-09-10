#pragma once

#include <esp_event.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RC522_DEFAULT_SCAN_INTERVAL_MS    (125)
#define RC522_DEFAULT_TASK_STACK_SIZE     (4 * 1024)
#define RC522_DEFAULT_TASK_STACK_PRIORITY (4)

ESP_EVENT_DECLARE_BASE(RC522_EVENTS);

typedef struct rc522 *rc522_handle_t;

typedef esp_err_t (*rc522_send_handler_t)(uint8_t *buffer, uint8_t length);
typedef esp_err_t (*rc522_receive_handler_t)(uint8_t *buffer, uint8_t length, uint8_t address);

typedef struct
{
    rc522_send_handler_t send_handler;
    rc522_receive_handler_t receive_handler;
    uint16_t scan_interval_ms; /*<! How fast will ESP32 scan for nearby tags, in miliseconds */
    size_t task_stack_size;    /*<! Stack size of rc522 task */
    uint8_t task_priority;     /*<! Priority of rc522 task */
} rc522_config_t;

typedef enum
{
    RC522_EVENT_ANY = ESP_EVENT_ANY_ID,
    RC522_EVENT_NONE,
    RC522_EVENT_TAG_SCANNED, /*<! Tag scanned */
} rc522_event_t;

typedef struct
{
    rc522_handle_t rc522;
    void *ptr;
} rc522_event_data_t;

typedef struct
{
    uint64_t uid;
} rc522_tag_t;

/**
 * @brief Create RC522 scanner handle.
 *        To start scanning tags call the rc522_start function.
 * @param config Configuration
 * @param out_rc522 Pointer to resulting new handle
 * @return ESP_OK on success
 */
esp_err_t rc522_create(rc522_config_t *config, rc522_handle_t *out_rc522);

esp_err_t rc522_register_events(
    rc522_handle_t rc522, rc522_event_t event, esp_event_handler_t event_handler, void *event_handler_arg);

esp_err_t rc522_unregister_events(rc522_handle_t rc522, rc522_event_t event, esp_event_handler_t event_handler);

/**
 * @brief Start to scan tags. If already started, ESP_OK will just be returned.
 *        Initialization function had to be called before this one.
 * @param rc522 Handle
 * @return ESP_OK on success
 */
esp_err_t rc522_start(rc522_handle_t rc522);

/**
 * @brief Start to scan tags. If already started, ESP_OK will just be returned.
 * @param rc522 Handle
 * @return ESP_OK on success
 */
#define rc522_resume(rc522) rc522_start(rc522)

/**
 * @brief Pause scan tags. If already paused, ESP_OK will just be returned.
 * @param rc522 Handle
 * @return ESP_OK on success
 */
esp_err_t rc522_pause(rc522_handle_t rc522);

/**
 * @brief Destroy RC522 and free all resources. Cannot be called from event handler.
 * @param rc522 Handle
 */
esp_err_t rc522_destroy(rc522_handle_t rc522);

#ifdef __cplusplus
}
#endif
