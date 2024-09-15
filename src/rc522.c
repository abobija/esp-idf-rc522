#include <esp_system.h>
#include <esp_log.h>
#include <esp_check.h>
#include <string.h>
#include <sys/time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "rc522_pcd_private.h"
#include "rc522_picc_private.h"
#include "rc522_helpers_private.h"
#include "rc522_types_private.h"
#include "rc522_private.h"

RC522_LOG_DEFINE_BASE();

ESP_EVENT_DEFINE_BASE(RC522_EVENTS);

inline bool rc522_is_able_to_start(rc522_handle_t rc522)
{
    return rc522->state >= RC522_STATE_CREATED && rc522->state != RC522_STATE_SCANNING;
}

static esp_err_t rc522_clone_config(rc522_config_t *config, rc522_config_t **result)
{
    rc522_config_t *_clone_config = calloc(1, sizeof(rc522_config_t));
    ESP_RETURN_ON_FALSE(_clone_config != NULL, ESP_ERR_NO_MEM, TAG, "No memory");

    memcpy(_clone_config, config, sizeof(rc522_config_t));

    // defaults
    _clone_config->scan_interval_ms =
        config->scan_interval_ms < 50 ? RC522_DEFAULT_SCAN_INTERVAL_MS : config->scan_interval_ms;
    _clone_config->task_stack_size =
        config->task_stack_size == 0 ? RC522_DEFAULT_TASK_STACK_SIZE : config->task_stack_size;
    _clone_config->task_priority =
        config->task_priority == 0 ? RC522_DEFAULT_TASK_STACK_PRIORITY : config->task_priority;

    *result = _clone_config;

    return ESP_OK;
}

esp_err_t rc522_create(rc522_config_t *config, rc522_handle_t *out_rc522)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config is null");
    ESP_RETURN_ON_FALSE(out_rc522 != NULL, ESP_ERR_INVALID_ARG, TAG, "out_rc522 is null");

    rc522_handle_t rc522 = calloc(1, sizeof(struct rc522));
    ESP_RETURN_ON_FALSE(rc522 != NULL, ESP_ERR_NO_MEM, TAG, "No memory");

    esp_err_t ret = ESP_OK;

    ESP_GOTO_ON_ERROR(rc522_clone_config(config, &(rc522->config)), _error, TAG, "Failed to clone config");

    esp_event_loop_args_t event_args = {
        .queue_size = 1,
        .task_name = NULL, // no task will be created
    };

    ESP_GOTO_ON_ERROR(esp_event_loop_create(&event_args, &rc522->event_handle),
        _error,
        TAG,
        "Failed to create event loop");

    rc522->task_running = true;

    BaseType_t task_create_result = xTaskCreate(rc522_task,
        "rc522_polling_task",
        rc522->config->task_stack_size,
        rc522,
        rc522->config->task_priority,
        &rc522->task_handle);

    ESP_GOTO_ON_FALSE(task_create_result == pdTRUE, ESP_FAIL, _error, TAG, "Failed to create task");

    goto _success;
_error:
    rc522_destroy(rc522);
    rc522 = NULL;
    goto _return;
_success:
    rc522->state = RC522_STATE_CREATED;
    *out_rc522 = rc522;
_return:
    return ret;
}

esp_err_t rc522_register_events(
    rc522_handle_t rc522, rc522_event_t event, esp_event_handler_t event_handler, void *event_handler_arg)
{
    ESP_RETURN_ON_FALSE(rc522 != NULL, ESP_ERR_INVALID_ARG, TAG, "rc522 is null");

    return esp_event_handler_register_with(rc522->event_handle, RC522_EVENTS, event, event_handler, event_handler_arg);
}

esp_err_t rc522_unregister_events(rc522_handle_t rc522, rc522_event_t event, esp_event_handler_t event_handler)
{
    ESP_RETURN_ON_FALSE(rc522 != NULL, ESP_ERR_INVALID_ARG, TAG, "rc522 is null");

    return esp_event_handler_unregister_with(rc522->event_handle, RC522_EVENTS, event, event_handler);
}

esp_err_t rc522_start(rc522_handle_t rc522)
{
    ESP_RETURN_ON_FALSE(rc522 != NULL, ESP_ERR_INVALID_ARG, TAG, "rc522 is null");
    ESP_RETURN_ON_FALSE(rc522_is_able_to_start(rc522),
        ESP_ERR_INVALID_STATE,
        TAG,
        "Unable to start (state=%d)",
        rc522->state);

    if (rc522->state == RC522_STATE_PAUSED) {
        // Scanning has been paused. No need for reinitialization. Just resume
        rc522->state = RC522_STATE_SCANNING;
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(rc522_pcd_rw_test(rc522), TAG, "RW test failed");

    ESP_RETURN_ON_ERROR(rc522_pcd_init(rc522), TAG, "Unable to init PCD");

    rc522_pcd_firmware_t fw;
    ESP_RETURN_ON_ERROR(rc522_pcd_firmware(rc522, &fw), TAG, "Failed to get firmware version");

    // When 0x00 or 0xFF is returned, communication probably failed
    if (fw == 0x00 || fw == 0xFF) {
        RC522_LOGE("Communication failure, is the MFRC522 properly connected?");

        return ESP_FAIL;
    }

    rc522->state = RC522_STATE_SCANNING;

    ESP_LOGI(TAG, "Scanning started (firmware=%s)", rc522_pcd_firmware_name(fw));

    return ESP_OK;
}

esp_err_t rc522_pause(rc522_handle_t rc522)
{
    ESP_RETURN_ON_FALSE(rc522 != NULL, ESP_ERR_INVALID_ARG, TAG, "rc522 is null");

    ESP_RETURN_ON_FALSE(rc522->state == RC522_STATE_SCANNING,
        ESP_ERR_INVALID_STATE,
        TAG,
        "Invalid state (state=%d)",
        rc522->state);

    rc522->state = RC522_STATE_PAUSED;

    return ESP_OK;
}

esp_err_t rc522_destroy(rc522_handle_t rc522)
{
    ESP_RETURN_ON_FALSE(rc522 != NULL, ESP_ERR_INVALID_ARG, TAG, "rc522 is null");
    ESP_RETURN_ON_FALSE(xTaskGetCurrentTaskHandle() != rc522->task_handle,
        ESP_ERR_INVALID_STATE,
        TAG,
        "Cannot destroy from event handler");

    rc522->task_running = false; //  task will delete itself

    // TODO: Wait for task to exit

    esp_err_t err = ESP_OK;

    if (rc522->event_handle) {
        if (esp_event_loop_delete(rc522->event_handle) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to delete event loop");
        }

        rc522->event_handle = NULL;
    }

    FREE(rc522->config);
    FREE(rc522);

    return err;
}

static esp_err_t rc522_dispatch_event(rc522_handle_t rc522, rc522_event_t event, const void *data, size_t data_size)
{
    RC522_RETURN_ON_ERROR(esp_event_post_to(rc522->event_handle, RC522_EVENTS, event, data, data_size, portMAX_DELAY));

    return esp_event_loop_run(rc522->event_handle, 0);
}

inline esp_err_t rc522_stream(rc522_handle_t rc522, const char *data /** Null terminated */)
{
    return rc522_dispatch_event(rc522, RC522_EVENT_STREAMING, data, strlen(data) + 1);
}

void rc522_task(void *arg)
{
    rc522_handle_t rc522 = (rc522_handle_t)arg;

    while (rc522->task_running) {
        if (rc522->state != RC522_STATE_SCANNING) {
            rc522_delay_ms(125);

            taskYIELD();
            continue;
        }

        rc522_picc_t picc;
        memset(&picc, 0, sizeof(picc));
        rc522_picc_find(rc522, &picc);

        if (picc.is_present && rc522_picc_fetch(rc522, &picc) == ESP_OK) {
            rc522_dispatch_event(rc522, RC522_EVENT_PICC_SELECTED, &picc, sizeof(picc));
        }

        rc522_delay_ms(rc522->config->scan_interval_ms);

        taskYIELD();
    }

    vTaskDelete(NULL); // self-delete
}
