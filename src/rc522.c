#include <esp_system.h>
#include <esp_log.h>
#include <esp_check.h>
#include <string.h>
#include <sys/time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "rc522_private.h"
#include "rc522_registers.h"
#include "comm/rc522_comm.h"
#include "comm/rc522_init.h"
#include "comm/rc522_crc.h"
#include "comm/rc522_rw_test.h"
#include "rc522_io.h"

static const char *TAG = "rc522";

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
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "Config ptr is NULL");
    ESP_RETURN_ON_FALSE(out_rc522 != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle ptr is NULL");

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
    ESP_RETURN_ON_FALSE(rc522 != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");

    return esp_event_handler_register_with(rc522->event_handle, RC522_EVENTS, event, event_handler, event_handler_arg);
}

esp_err_t rc522_unregister_events(rc522_handle_t rc522, rc522_event_t event, esp_event_handler_t event_handler)
{
    ESP_RETURN_ON_FALSE(rc522 != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");

    return esp_event_handler_unregister_with(rc522->event_handle, RC522_EVENTS, event, event_handler);
}

static esp_err_t rc522_card_write(
    rc522_handle_t rc522, uint8_t cmd, uint8_t *data, uint8_t n, uint8_t *res_n, uint8_t **result)
{
    esp_err_t ret = ESP_OK;
    uint8_t *_result = NULL;
    uint8_t _res_n = 0;
    uint8_t irq = 0x00;
    uint8_t irq_wait = 0x00;
    uint8_t nn = 0;

    if (cmd == RC522_CMD_MF_AUTH) {
        irq = 0x12;
        irq_wait = 0x10;
    }
    else if (cmd == RC522_CMD_TRANSCEIVE) {
        irq = 0x77;
        irq_wait = 0x30;
    }

    ESP_RETURN_ON_ERROR(rc522_write(rc522, RC522_COMM_INT_EN_REG, irq | 0x80), TAG, "");
    ESP_RETURN_ON_ERROR(rc522_clear_bitmask(rc522, RC522_COMM_INT_REQ_REG, 0x80), TAG, "");
    ESP_RETURN_ON_ERROR(rc522_flush_fifo_buffer(rc522), TAG, "");
    ESP_RETURN_ON_ERROR(rc522_stop_active_command(rc522), TAG, "");
    ESP_RETURN_ON_ERROR(rc522_write_n(rc522, RC522_FIFO_DATA_REG, n, data), TAG, "");
    ESP_RETURN_ON_ERROR(rc522_write(rc522, RC522_COMMAND_REG, cmd), TAG, "");

    if (cmd == RC522_CMD_TRANSCEIVE) {
        ESP_RETURN_ON_ERROR(rc522_start_data_transmission(rc522), TAG, "");
    }

    uint16_t i = 1000;

    for (;;) {
        ESP_RETURN_ON_ERROR(rc522_read(rc522, RC522_COMM_INT_REQ_REG, &nn), TAG, "");

        i--;

        if (!(i != 0 && (((nn & 0x01) == 0) && ((nn & irq_wait) == 0)))) {
            break;
        }
    }

    ESP_RETURN_ON_ERROR(rc522_stop_data_transmission(rc522), TAG, "");

    if (i != 0) {
        uint8_t tmp;
        ESP_RETURN_ON_ERROR(rc522_read(rc522, RC522_ERROR_REG, &tmp), TAG, "");

        if ((tmp & 0x1B) == 0x00) {
            if (cmd == RC522_CMD_TRANSCEIVE) {
                ESP_RETURN_ON_ERROR(rc522_read(rc522, RC522_FIFO_LEVEL_REG, &nn), TAG, "");
                ESP_RETURN_ON_ERROR(rc522_read(rc522, RC522_CONTROL_REG, &tmp), TAG, "");

                uint8_t last_bits = tmp & 0x07;

                if (last_bits != 0) {
                    _res_n = (nn - 1) + last_bits;
                }
                else {
                    _res_n = nn;
                }

                if (_res_n > 0) {
                    _result = (uint8_t *)malloc(_res_n);
                    ESP_RETURN_ON_FALSE(_result != NULL, ESP_ERR_NO_MEM, TAG, "No memory");

                    for (i = 0; i < _res_n; i++) {
                        ESP_GOTO_ON_ERROR(rc522_read(rc522, RC522_FIFO_DATA_REG, &tmp), _error, TAG, "");
                        _result[i] = tmp;
                    }
                }
            }
        }
    }

    goto _success;
_error:
    FREE(_result);
    _res_n = 0;
    goto _return;
_success:
    *res_n = _res_n;
    *result = _result;
_return:
    return ret;
}

static esp_err_t rc522_request(rc522_handle_t rc522, uint8_t *res_n, uint8_t **result)
{
    uint8_t *_result = NULL;
    uint8_t _res_n = 0;
    uint8_t req_mode = 0x26;

    ESP_RETURN_ON_ERROR(rc522_write(rc522, RC522_BIT_FRAMING_REG, 0x07), TAG, "");
    ESP_RETURN_ON_ERROR(rc522_card_write(rc522, 0x0C, &req_mode, 1, &_res_n, &_result), TAG, "");

    if (_res_n * 8 != 0x10) {
        FREE(_result);
        return ESP_FAIL;
    }

    *res_n = _res_n;
    *result = _result;

    return ESP_OK;
}

static esp_err_t rc522_anticoll(rc522_handle_t rc522, uint8_t **result)
{
    uint8_t *_result = NULL;
    uint8_t _res_n;

    ESP_RETURN_ON_ERROR(rc522_write(rc522, RC522_BIT_FRAMING_REG, 0x00), TAG, "");
    ESP_RETURN_ON_ERROR(rc522_card_write(rc522, 0x0C, (uint8_t[]) { 0x93, 0x20 }, 2, &_res_n, &_result), TAG, "");

    esp_err_t ret = ESP_OK;

    // TODO: Some cards have length of 4, and some of them have length of 7 bytes
    //       here we are using one extra byte which is not part of UID.
    //       Implement logic to determine the length of the UID and use that info
    //       to retrieve the real UID
    //
    // UID of all cards/tags is 5 bytes long (??)
    ESP_GOTO_ON_FALSE(_res_n == 5, ESP_FAIL, _error, TAG, "Invalid UID length");

    goto _success;
_error:
    FREE(_result);
    _res_n = 0;
    goto _return;
_success:
    *result = _result;
_return:
    return ret;
}

static esp_err_t rc522_get_tag(rc522_handle_t rc522, uint8_t **result)
{
    esp_err_t ret = ESP_OK;
    uint8_t *_result = NULL;
    uint8_t *res_data = NULL;
    uint8_t res_data_n;

    if ((ret = rc522_request(rc522, &res_data_n, &res_data)) != ESP_OK) {
        // Failed (most probably) because card is not near the scanner
        return ret;
    }

    FREE(res_data);
    ESP_GOTO_ON_ERROR(rc522_anticoll(rc522, &_result), _error, TAG, "");
    uint8_t buf[] = { 0x50, 0x00, 0x00, 0x00 };
    ESP_GOTO_ON_ERROR(rc522_calculate_crc(rc522, buf, 2, buf + 2), _error, TAG, "");
    ESP_GOTO_ON_ERROR(rc522_card_write(rc522, 0x0C, buf, 4, &res_data_n, &res_data), _error, TAG, "");
    FREE(res_data);
    ESP_GOTO_ON_ERROR(rc522_clear_bitmask(rc522, RC522_STATUS_2_REG, 0x08), _error, TAG, "");

    goto _success;
_error:
    FREE(_result);
    goto _return;
_success:
    *result = _result;
_return:
    FREE(res_data);
    return ret;
}

esp_err_t rc522_start(rc522_handle_t rc522)
{
    ESP_RETURN_ON_FALSE(rc522 != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
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

    ESP_RETURN_ON_ERROR(rc522_rw_test(rc522, RC522_MOD_WIDTH_REG, 5), TAG, "RW test failed");

    ESP_RETURN_ON_ERROR(rc522_init(rc522), TAG, "Unable to init");

    uint8_t fw_ver;
    ESP_RETURN_ON_ERROR(rc522_firmware(rc522, &fw_ver), TAG, "Failed to get firmware version");

    rc522->state = RC522_STATE_SCANNING;

    ESP_LOGI(TAG, "Scanning started (firmware=%d.0)", fw_ver);

    return ESP_OK;
}

esp_err_t rc522_pause(rc522_handle_t rc522)
{
    ESP_RETURN_ON_FALSE(rc522 != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");

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
    ESP_RETURN_ON_FALSE(rc522 != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
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

static esp_err_t rc522_dispatch_event(rc522_handle_t rc522, rc522_event_t event, void *data)
{
    rc522_event_data_t e_data = {
        .rc522 = rc522,
        .ptr = data,
    };

    ESP_RETURN_ON_ERROR(
        esp_event_post_to(rc522->event_handle, RC522_EVENTS, event, &e_data, sizeof(rc522_event_data_t), portMAX_DELAY),
        TAG,
        "");

    return esp_event_loop_run(rc522->event_handle, 0);
}

void rc522_task(void *arg)
{
    rc522_handle_t rc522 = (rc522_handle_t)arg;

    while (rc522->task_running) {
        if (rc522->state != RC522_STATE_SCANNING) {
            // Idling...
            rc522_delay_ms(100);
            continue;
        }

        uint8_t *uid_bytes = NULL;

        if (ESP_OK != rc522_get_tag(rc522, &uid_bytes)) {
            // Tag is not present
            //
            // TODO: Implement logic to know when the error is due to
            //       tag absence or some other protocol issue
        }

        if (uid_bytes == NULL) {
            rc522->tag_was_present_last_time = false;
        }
        else if (!rc522->tag_was_present_last_time) {
            rc522_tag_uid_t uid = {
                .bytes = uid_bytes,
                .length = 5, // TODO: Change once when we dynamicaly find the length of UID
            };
            rc522_tag_t tag = { .uid = uid };
            rc522_dispatch_event(rc522, RC522_EVENT_TAG_SCANNED, &tag);
            FREE(uid_bytes);
            rc522->tag_was_present_last_time = true;
        }
        else {
            FREE(uid_bytes);
        }

        int delay_interval_ms = rc522->config->scan_interval_ms;

        if (rc522->tag_was_present_last_time) {
            delay_interval_ms *= 2; // extra scan-bursting prevention
        }

        rc522_delay_ms(delay_interval_ms);
    }

    vTaskDelete(NULL);
}

uint32_t rc522_millis()
{
    struct timeval now;
    gettimeofday(&now, NULL);

    return (uint32_t)((now.tv_sec * 1000000 + now.tv_usec) / 1000);
}

void rc522_delay_ms(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}
