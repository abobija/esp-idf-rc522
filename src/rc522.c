#include <esp_system.h>
#include <esp_log.h>
#include <esp_check.h>
#include <string.h>
#include <sys/time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include "rc522_pcd_private.h"
#include "rc522_picc_private.h"
#include "rc522_helpers_private.h"
#include "rc522_types_private.h"
#include "rc522_private.h"

RC522_LOG_DEFINE_BASE();

ESP_EVENT_DEFINE_BASE(RC522_EVENTS);

inline bool rc522_is_able_to_start(rc522_handle_t rc522)
{
    return rc522->state >= RC522_STATE_CREATED && rc522->state != RC522_STATE_POLLING;
}

static esp_err_t rc522_clone_config(rc522_config_t *config, rc522_config_t **result)
{
    rc522_config_t *config_clone = calloc(1, sizeof(rc522_config_t));
    ESP_RETURN_ON_FALSE(config_clone != NULL, ESP_ERR_NO_MEM, TAG, "nomem");

    memcpy(config_clone, config, sizeof(rc522_config_t));

    // defaults
    if (config_clone->task_throttling_ms < RC522_TASK_THROTTLING_MS_MIN) {
        config_clone->task_throttling_ms = RC522_TASK_THROTTLING_MS_DEFAULT;
    }

    if (config_clone->task_stack_size == 0) {
        config_clone->task_stack_size = RC522_TASK_STACK_SIZE_DEFAULT;
    }

    if (config_clone->task_priority == 0) {
        config_clone->task_priority = RC522_TASK_PRIORITY_DEFAULT;
    }
    // ~defaults

    *result = config_clone;

    return ESP_OK;
}

esp_err_t rc522_register_events(
    rc522_handle_t rc522, rc522_event_t event, esp_event_handler_t event_handler, void *event_handler_arg)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(event_handler == NULL);

    return esp_event_handler_register_with(rc522->event_handle, RC522_EVENTS, event, event_handler, event_handler_arg);
}

esp_err_t rc522_unregister_events(rc522_handle_t rc522, rc522_event_t event, esp_event_handler_t event_handler)
{
    RC522_CHECK(rc522 == NULL);
    RC522_CHECK(event_handler == NULL);

    return esp_event_handler_unregister_with(rc522->event_handle, RC522_EVENTS, event, event_handler);
}

esp_err_t rc522_start(rc522_handle_t rc522)
{
    RC522_CHECK(rc522 == NULL);

    ESP_RETURN_ON_FALSE(rc522 != NULL, ESP_ERR_INVALID_ARG, TAG, "rc522 is null");
    ESP_RETURN_ON_FALSE(rc522_is_able_to_start(rc522),
        ESP_ERR_INVALID_STATE,
        TAG,
        "Unable to start (state=%d)",
        rc522->state);

    if (rc522->state == RC522_STATE_PAUSED) {
        // Scanning has been paused. No need for reinitialization. Just resume
        rc522->state = RC522_STATE_POLLING;
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(rc522_pcd_rw_test(rc522), TAG, "RW test failed");

    ESP_RETURN_ON_ERROR(rc522_pcd_init(rc522), TAG, "Unable to init PCD");

    rc522->state = RC522_STATE_POLLING;

    return ESP_OK;
}

esp_err_t rc522_pause(rc522_handle_t rc522)
{
    RC522_CHECK(rc522 == NULL);

    ESP_RETURN_ON_FALSE(rc522->state == RC522_STATE_POLLING,
        ESP_ERR_INVALID_STATE,
        TAG,
        "Invalid state (state=%d)",
        rc522->state);

    rc522->state = RC522_STATE_PAUSED;

    return ESP_OK;
}

/**
 * Exit and delete the task
 */
inline static void rc522_request_task_exit(rc522_handle_t rc522)
{
    rc522->exit_requested = true; // task will delete itself
}

esp_err_t rc522_create(rc522_config_t *config, rc522_handle_t *out_rc522)
{
    RC522_CHECK(config == NULL);
    RC522_CHECK(out_rc522 == NULL);

    rc522_handle_t rc522 = calloc(1, sizeof(struct rc522));
    ESP_RETURN_ON_FALSE(rc522 != NULL, ESP_ERR_NO_MEM, TAG, "nomem");

    rc522->picc.state = RC522_PICC_STATE_IDLE;

    esp_err_t ret = ESP_OK;

    ESP_GOTO_ON_ERROR(rc522_clone_config(config, &(rc522->config)), _error, TAG, "clone config failed");

    esp_event_loop_args_t event_args = {
        .queue_size = 1,
        .task_name = NULL, // no task will be created
    };

    rc522->bits = xEventGroupCreate();
    ESP_GOTO_ON_FALSE(rc522->bits != NULL, ESP_ERR_NO_MEM, _error, TAG, "nomem");

    ESP_GOTO_ON_ERROR(esp_event_loop_create(&event_args, &rc522->event_handle),
        _error,
        TAG,
        "Failed to create event loop");

    BaseType_t task_create_result = xTaskCreate(rc522_task,
        "rc522_polling_task",
        rc522->config->task_stack_size,
        rc522,
        rc522->config->task_priority,
        &rc522->task_handle);

    ESP_GOTO_ON_FALSE(task_create_result == pdTRUE, ESP_FAIL, _error, TAG, "task create failed");

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

esp_err_t rc522_destroy(rc522_handle_t rc522)
{
    RC522_CHECK(rc522 == NULL);

    ESP_RETURN_ON_FALSE(xTaskGetCurrentTaskHandle() != rc522->task_handle,
        ESP_ERR_INVALID_STATE,
        TAG,
        "Cannot destroy from event handler");

    rc522_request_task_exit(rc522);

    if (rc522->bits) {
        xEventGroupWaitBits(rc522->bits, RC522_TASK_STOPPED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
        vEventGroupDelete(rc522->bits);
        rc522->bits = NULL;
    }

    if (rc522->event_handle) {
        if (esp_event_loop_delete(rc522->event_handle) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to delete event loop");
        }

        rc522->event_handle = NULL;
    }

    if (rc522->config) {
        free(rc522->config);
        rc522->config = NULL;
    }

    free(rc522);

    return ESP_OK;
}

static esp_err_t rc522_dispatch_event(rc522_handle_t rc522, rc522_event_t event, const void *data, size_t data_size)
{
    RC522_RETURN_ON_ERROR(esp_event_post_to(rc522->event_handle, RC522_EVENTS, event, data, data_size, portMAX_DELAY));

    return esp_event_loop_run(rc522->event_handle, 0);
}

void rc522_task(void *arg)
{
    rc522_handle_t rc522 = (rc522_handle_t)arg;
    esp_err_t ret = ESP_OK;

    xEventGroupClearBits(rc522->bits, RC522_TASK_STOPPED_BIT);

    // TODO: consider sending picc clone to the event handlers
    //       to avoid modifying picc state by the user

    while (!rc522->exit_requested) {
        if (rc522->state == RC522_STATE_POLLING) {
            rc522_delay_ms(rc522->config->task_throttling_ms);
        }
        else {
            // wait for state change to polling
            rc522_delay_ms(125);
            continue;
        }

        if (rc522->picc.state == RC522_PICC_STATE_IDLE || rc522->picc.state == RC522_PICC_STATE_HALT) {
            rc522_picc_atqa_desc_t atqa;

            if (rc522->picc.state == RC522_PICC_STATE_IDLE && rc522_picc_reqa(rc522, &atqa) != ESP_OK) {
                continue;
            }

            if (rc522->picc.state == RC522_PICC_STATE_HALT && rc522_picc_wupa(rc522, &atqa) != ESP_OK) {
                continue;
            }

            // card is present
            rc522->picc.atqa = atqa;
            rc522->picc.state = RC522_PICC_STATE_READY;
        }

        if (rc522->picc.state == RC522_PICC_STATE_READY || rc522->picc.state == RC522_PICC_STATE_ACTIVE) {
            ret = rc522_picc_anticoll_and_select(rc522, &rc522->picc);

            if (ret != ESP_OK) {
                RC522_LOGE("select failed: %04x", ret);

                rc522_picc_state_t prev_state = rc522->picc.state;
                rc522->picc.state = RC522_PICC_STATE_IDLE;

                if (prev_state == RC522_PICC_STATE_ACTIVE) {
                    if (rc522_dispatch_event(rc522, RC522_EVENT_PICC_DISAPPEARED, &rc522->picc, sizeof(rc522_picc_t))
                        != ESP_OK) {
                        RC522_LOGW("event dispatch failed");
                    }
                }

                continue;
            }

            if (rc522->picc.state == RC522_PICC_STATE_ACTIVE) {
                // cars is still in the field
                // TODO: maybe to check if SAK is still the same?
                continue;
            }

            rc522->picc.type = rc522_picc_type(rc522->picc.sak);
            rc522->picc.state = RC522_PICC_STATE_ACTIVE;

            if (rc522_dispatch_event(rc522, RC522_EVENT_PICC_ACTIVATED, &rc522->picc, sizeof(rc522_picc_t)) != ESP_OK) {
                RC522_LOGW("event dispatch failed");
            }
        }
    }

    xEventGroupSetBits(rc522->bits, RC522_TASK_STOPPED_BIT);
    ESP_LOGI(TAG, "Task exited");
    vTaskDelete(NULL); // self-delete
}
