#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <string.h>
#include <esp_check.h>

#include "rc522.h"
#include "rc522/guards.h"
#include "rc522/registers.h"

static const char *TAG = "RC522";

ESP_EVENT_DEFINE_BASE(RC522_EVENTS);

struct rc522
{
    bool running;                         /*<! Indicates whether rc522 task is running or not */
    rc522_config_t *config;               /*<! Configuration */
    TaskHandle_t task_handle;             /*<! Handle of task */
    esp_event_loop_handle_t event_handle; /*<! Handle of event loop */
    // TODO: Use new 'status' field, instead of initialized, scanning, etc...
    bool scanning; /*<! Whether the rc522 is in scanning or idle mode */
    bool tag_was_present_last_time;
};

static void rc522_task(void *arg);

static esp_err_t rc522_write_n(rc522_handle_t rc522, uint8_t addr, uint8_t n, uint8_t *data)
{
    esp_err_t err = ESP_OK;
    uint8_t *buffer = NULL;

    // TODO: Find a way to send address + data without memory allocation
    ALLOC_JMP_GUARD(buffer = (uint8_t *)malloc(n + 1));

    buffer[0] = addr;
    memcpy(buffer + 1, data, n);

    ESP_ERR_JMP_GUARD(rc522->config->send_handler(buffer, n + 1));

    JMP_GUARD_GATES({ ESP_LOGE(TAG, "Failed to write data (err: %s)", esp_err_to_name(err)); }, {});

    FREE(buffer);

    return err;
}

static inline esp_err_t rc522_write(rc522_handle_t rc522, uint8_t addr, uint8_t val)
{
    return rc522_write_n(rc522, addr, 1, &val);
}

static esp_err_t rc522_write_map(rc522_handle_t rc522, const uint8_t map[][2], uint8_t map_length)
{
    for (uint8_t i = 0; i < map_length; i++) {
        const uint8_t address = map[i][0];
        const uint8_t value = map[i][1];

        ESP_RETURN_ON_ERROR(rc522_write(rc522, address, value),
            TAG,
            "Failed to write %d into 0x%20X register",
            value,
            address);
    }

    return ESP_OK;
}

static esp_err_t rc522_read_n(rc522_handle_t rc522, uint8_t addr, uint8_t n, uint8_t *buffer)
{
    esp_err_t err;

    ESP_ERR_JMP_GUARD(rc522->config->receive_handler(buffer, n, addr));

    JMP_GUARD_GATES({ ESP_LOGE(TAG, "Failed to read data (err: %s)", esp_err_to_name(err)); }, {});

    return err;
}

static inline esp_err_t rc522_read(rc522_handle_t rc522, uint8_t addr, uint8_t *value_ref)
{
    return rc522_read_n(rc522, addr, 1, value_ref);
}

static esp_err_t rc522_set_bitmask(rc522_handle_t rc522, uint8_t addr, uint8_t mask)
{
    esp_err_t err = ESP_OK;
    uint8_t tmp;

    ESP_ERR_RET_GUARD(rc522_read(rc522, addr, &tmp));

    return rc522_write(rc522, addr, tmp | mask);
}

static esp_err_t rc522_clear_bitmask(rc522_handle_t rc522, uint8_t addr, uint8_t mask)
{
    esp_err_t err = ESP_OK;
    uint8_t tmp;

    ESP_ERR_RET_GUARD(rc522_read(rc522, addr, &tmp));

    return rc522_write(rc522, addr, tmp & ~mask);
}

static inline esp_err_t rc522_firmware(rc522_handle_t rc522, uint8_t *result)
{
    uint8_t value;
    ESP_RETURN_ON_ERROR(rc522_read(rc522, RC522_VERSION_REG, &value), TAG, "");

    *result = value & 0x03;
    return ESP_OK;
}

static esp_err_t rc522_antenna_on(rc522_handle_t rc522)
{
    esp_err_t err = ESP_OK;
    uint8_t tmp;

    ESP_ERR_RET_GUARD(rc522_read(rc522, RC522_TX_CONTROL_REG, &tmp));

    if (~(tmp & 0x03)) {
        ESP_ERR_RET_GUARD(rc522_set_bitmask(rc522, RC522_TX_CONTROL_REG, 0x03));
    }

    return rc522_write(rc522, RC522_RF_CFG_REG, 0x60); // 43dB gain
}

static esp_err_t rc522_clone_config(rc522_config_t *config, rc522_config_t **result)
{
    rc522_config_t *_clone_config = NULL;

    ALLOC_RET_GUARD(_clone_config = calloc(1, sizeof(rc522_config_t)));

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
    if (!config || !out_rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;
    rc522_handle_t rc522 = NULL;

    ALLOC_RET_GUARD(rc522 = calloc(1, sizeof(struct rc522)));

    ESP_ERR_LOG_AND_JMP_GUARD(rc522_clone_config(config, &(rc522->config)), "Fail to clone config");

    esp_event_loop_args_t event_args = {
        .queue_size = 1,
        .task_name = NULL, // no task will be created
    };

    ESP_ERR_LOG_AND_JMP_GUARD(esp_event_loop_create(&event_args, &rc522->event_handle), "Fail to create event loop");

    rc522->running = true;

    CONDITION_LOG_AND_JMP_GUARD(pdTRUE != xTaskCreate(rc522_task,
                                              "rc522_task",
                                              rc522->config->task_stack_size,
                                              rc522,
                                              rc522->config->task_priority,
                                              &rc522->task_handle),
        "Fail to create task");

    JMP_GUARD_GATES(
        {
            rc522_destroy(rc522);
            rc522 = NULL;
        },
        { *out_rc522 = rc522; });

    return err;
}

esp_err_t rc522_register_events(
    rc522_handle_t rc522, rc522_event_t event, esp_event_handler_t event_handler, void *event_handler_arg)
{
    if (!rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    return esp_event_handler_register_with(rc522->event_handle, RC522_EVENTS, event, event_handler, event_handler_arg);
}

esp_err_t rc522_unregister_events(rc522_handle_t rc522, rc522_event_t event, esp_event_handler_t event_handler)
{
    if (!rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    return esp_event_handler_unregister_with(rc522->event_handle, RC522_EVENTS, event, event_handler);
}

static uint64_t rc522_uid_to_u64(uint8_t *sn)
{
    uint64_t result = 0;

    if (!sn) {
        return 0;
    }

    for (int i = 4; i >= 0; i--) {
        result |= ((uint64_t)sn[i] << (i * 8));
    }

    return result;
}

// Buffer should be length of 2, or more
// Only first 2 elements will be used where the result will be stored
// TODO: Use 2+ bytes data type instead of buffer array
static esp_err_t rc522_calculate_crc(rc522_handle_t rc522, uint8_t *data, uint8_t n, uint8_t *buffer)
{
    esp_err_t err = ESP_OK;
    uint8_t i = 255;
    uint8_t nn = 0;

    ESP_ERR_RET_GUARD(rc522_clear_bitmask(rc522, RC522_DIV_INT_REQ_REG, 0x04));
    ESP_ERR_RET_GUARD(rc522_set_bitmask(rc522, RC522_FIFO_LEVEL_REG, 0x80));
    ESP_ERR_RET_GUARD(rc522_write_n(rc522, RC522_FIFO_DATA_REG, n, data));
    ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_COMMAND_REG, 0x03));

    for (;;) {
        ESP_ERR_RET_GUARD(rc522_read(rc522, RC522_DIV_INT_REQ_REG, &nn));

        i--;

        if (!(i != 0 && !(nn & 0x04))) {
            break;
        }
    }

    uint8_t tmp;

    ESP_ERR_RET_GUARD(rc522_read(rc522, RC522_CRC_RESULT_LSB_REG, &tmp));
    buffer[0] = tmp;

    ESP_ERR_RET_GUARD(rc522_read(rc522, RC522_CRC_RESULT_MSB_REG, &tmp));
    buffer[1] = tmp;

    return ESP_OK;
}

static esp_err_t rc522_card_write(
    rc522_handle_t rc522, uint8_t cmd, uint8_t *data, uint8_t n, uint8_t *res_n, uint8_t **result)
{
    esp_err_t err = ESP_OK;
    uint8_t *_result = NULL;
    uint8_t _res_n = 0;
    uint8_t irq = 0x00;
    uint8_t irq_wait = 0x00;
    uint8_t last_bits = 0;
    uint8_t nn = 0;
    uint8_t tmp;

    if (cmd == 0x0E) {
        irq = 0x12;
        irq_wait = 0x10;
    }
    else if (cmd == 0x0C) {
        irq = 0x77;
        irq_wait = 0x30;
    }

    ESP_ERR_JMP_GUARD(rc522_write(rc522, RC522_COMM_INT_EN_REG, irq | 0x80));
    ESP_ERR_JMP_GUARD(rc522_clear_bitmask(rc522, RC522_COMM_INT_REQ_REG, 0x80));
    ESP_ERR_JMP_GUARD(rc522_set_bitmask(rc522, RC522_FIFO_LEVEL_REG, 0x80));
    ESP_ERR_JMP_GUARD(rc522_write(rc522, RC522_COMMAND_REG, 0x00));
    ESP_ERR_JMP_GUARD(rc522_write_n(rc522, RC522_FIFO_DATA_REG, n, data));
    ESP_ERR_JMP_GUARD(rc522_write(rc522, RC522_COMMAND_REG, cmd));

    if (cmd == 0x0C) {
        ESP_ERR_JMP_GUARD(rc522_set_bitmask(rc522, RC522_BIT_FRAMING_REG, 0x80));
    }

    uint16_t i = 1000;

    for (;;) {
        ESP_ERR_JMP_GUARD(rc522_read(rc522, RC522_COMM_INT_REQ_REG, &nn));

        i--;

        if (!(i != 0 && (((nn & 0x01) == 0) && ((nn & irq_wait) == 0)))) {
            break;
        }
    }

    ESP_ERR_JMP_GUARD(rc522_clear_bitmask(rc522, RC522_BIT_FRAMING_REG, 0x80));

    if (i != 0) {
        ESP_ERR_JMP_GUARD(rc522_read(rc522, RC522_ERROR_REG, &tmp));

        if ((tmp & 0x1B) == 0x00) {
            if (cmd == 0x0C) {
                ESP_ERR_JMP_GUARD(rc522_read(rc522, RC522_FIFO_LEVEL_REG, &nn));
                ESP_ERR_JMP_GUARD(rc522_read(rc522, RC522_CONTROL_REG, &tmp));

                last_bits = tmp & 0x07;

                if (last_bits != 0) {
                    _res_n = (nn - 1) + last_bits;
                }
                else {
                    _res_n = nn;
                }

                if (_res_n > 0) {
                    ALLOC_JMP_GUARD(_result = (uint8_t *)malloc(_res_n));

                    for (i = 0; i < _res_n; i++) {
                        ESP_ERR_JMP_GUARD(rc522_read(rc522, RC522_FIFO_DATA_REG, &tmp));
                        _result[i] = tmp;
                    }
                }
            }
        }
    }

    JMP_GUARD_GATES(
        {
            FREE(_result);
            _res_n = 0;
        },
        {
            *res_n = _res_n;
            *result = _result;
        });

    return err;
}

static esp_err_t rc522_request(rc522_handle_t rc522, uint8_t *res_n, uint8_t **result)
{
    esp_err_t err = ESP_OK;
    uint8_t *_result = NULL;
    uint8_t _res_n = 0;
    uint8_t req_mode = 0x26;

    ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_BIT_FRAMING_REG, 0x07));
    ESP_ERR_RET_GUARD(rc522_card_write(rc522, 0x0C, &req_mode, 1, &_res_n, &_result));

    if (_res_n * 8 != 0x10) {
        free(_result);

        return ESP_ERR_INVALID_STATE;
    }

    *res_n = _res_n;
    *result = _result;

    return err;
}

static esp_err_t rc522_anticoll(rc522_handle_t rc522, uint8_t **result)
{
    esp_err_t err = ESP_OK;
    uint8_t *_result = NULL;
    uint8_t _res_n;

    ESP_ERR_JMP_GUARD(rc522_write(rc522, RC522_BIT_FRAMING_REG, 0x00));
    ESP_ERR_JMP_GUARD(rc522_card_write(rc522, 0x0C, (uint8_t[]) { 0x93, 0x20 }, 2, &_res_n, &_result));

    // TODO: Some cards have length of 4, and some of them have length of 7 bytes
    //       here we are using one extra byte which is not part of UID.
    //       Implement logic to determine the length of the UID and use that info
    //       to retrieve the real UID
    if (_result && _res_n != 5) { // UID of all cards/tags is 5 bytes long (??)
        ESP_ERR_LOG_AND_JMP_GUARD(ESP_ERR_INVALID_RESPONSE, "invalid length of UID");
    }

    JMP_GUARD_GATES(
        {
            FREE(_result);
            _res_n = 0;
        },
        { *result = _result; });

    return err;
}

static esp_err_t rc522_get_tag(rc522_handle_t rc522, uint8_t **result)
{
    esp_err_t err = ESP_OK;
    uint8_t *_result = NULL;
    uint8_t *res_data = NULL;
    uint8_t res_data_n;

    ESP_ERR_JMP_GUARD(rc522_request(rc522, &res_data_n, &res_data));

    if (res_data != NULL) {
        FREE(res_data);
        ESP_ERR_JMP_GUARD(rc522_anticoll(rc522, &_result));

        if (_result != NULL) {
            uint8_t buf[] = { 0x50, 0x00, 0x00, 0x00 };
            ESP_ERR_JMP_GUARD(rc522_calculate_crc(rc522, buf, 2, buf + 2));
            ESP_ERR_JMP_GUARD(rc522_card_write(rc522, 0x0C, buf, 4, &res_data_n, &res_data));
            FREE(res_data);
            ESP_ERR_JMP_GUARD(rc522_clear_bitmask(rc522, RC522_STATUS_2_REG, 0x08));
        }
    }

    JMP_GUARD_GATES(
        {
            FREE(_result);
            FREE(res_data);
        },
        { *result = _result; });

    return err;
}

static esp_err_t rc522_rw_test(rc522_handle_t rc522, uint8_t test_register, uint8_t times)
{
    uint8_t origin_value;

    ESP_RETURN_ON_ERROR(rc522_read(rc522, test_register, &origin_value),
        TAG,
        "Unable to read origin value of 0x%20X register",
        test_register);

    for (uint8_t val = 0; val < times; val++) {
        ESP_RETURN_ON_ERROR(rc522_write(rc522, test_register, val),
            TAG,
            "Unable to write value %d into 0x%20X register",
            val,
            test_register);

        uint8_t real_value;

        ESP_RETURN_ON_ERROR(rc522_read(rc522, test_register, &real_value),
            TAG,
            "Unable to read value of 0x%20X register",
            test_register);

        ESP_RETURN_ON_FALSE(val == real_value,
            ESP_FAIL,
            TAG,
            "Value %d of register 0x%20X does not match previously written value %d",
            real_value,
            test_register,
            val);
    }

    ESP_RETURN_ON_ERROR(rc522_write(rc522, test_register, origin_value),
        TAG,
        "Unable to restore origin value of 0x%20X register",
        test_register);

    return ESP_OK;
}

esp_err_t rc522_start(rc522_handle_t rc522)
{
    ESP_RETURN_ON_FALSE(rc522 != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle cannot be null");
    ESP_RETURN_ON_FALSE(!rc522->scanning, ESP_ERR_INVALID_STATE, TAG, "Already in scan mode");

    ESP_RETURN_ON_ERROR(rc522_rw_test(rc522, RC522_MOD_WIDTH_REG, 5), TAG, "RW test failed");

    const uint8_t map[][2] = {
        { RC522_COMMAND_REG, 0x0F },
        { RC522_TIMER_MODE_REG, 0x8D },
        { RC522_TIMER_PRESCALER_REG, 0x3E },
        { RC522_TIMER_RELOAD_LSB_REG, 0x1E },
        { RC522_TIMER_RELOAD_MSB_REG, 0x00 },
        { RC522_TX_ASK_REG, 0x40 },
        { RC522_MODE_REG, 0x3D },
    };

    ESP_RETURN_ON_ERROR(rc522_write_map(rc522, map, sizeof(map) / sizeof(uint8_t) / 2),
        TAG,
        "Failed to write initialization map");

    ESP_RETURN_ON_ERROR(rc522_antenna_on(rc522), TAG, "Unable to turn on antenna");

    uint8_t fw_ver;
    ESP_RETURN_ON_ERROR(rc522_firmware(rc522, &fw_ver), TAG, "Failed to get firmware version");
    ESP_LOGI(TAG, "Started (firmware=%d.0)", fw_ver);

    rc522->scanning = true;

    return ESP_OK;
}

esp_err_t rc522_pause(rc522_handle_t rc522)
{
    if (!rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!rc522->scanning) {
        return ESP_OK;
    }

    rc522->scanning = false;

    return ESP_OK;
}

esp_err_t rc522_destroy(rc522_handle_t rc522)
{
    esp_err_t err = ESP_OK;

    if (!rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xTaskGetCurrentTaskHandle() == rc522->task_handle) {
        ESP_LOGE(TAG, "Cannot destroy rc522 from event handler");

        return ESP_ERR_INVALID_STATE;
    }

    err = rc522_pause(rc522); // stop task
    rc522->running = false;   // stop rc522 -> task will delete itself

    // TODO: Wait here for task to exit

    if (rc522->event_handle) {
        err = esp_event_loop_delete(rc522->event_handle);
        rc522->event_handle = NULL;
    }

    FREE(rc522->config);
    FREE(rc522);

    return err;
}

static esp_err_t rc522_dispatch_event(rc522_handle_t rc522, rc522_event_t event, void *data)
{
    esp_err_t err;

    if (!rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    rc522_event_data_t e_data = {
        .rc522 = rc522,
        .ptr = data,
    };

    ESP_ERR_RET_GUARD(esp_event_post_to(rc522->event_handle,
        RC522_EVENTS,
        event,
        &e_data,
        sizeof(rc522_event_data_t),
        portMAX_DELAY));

    return esp_event_loop_run(rc522->event_handle, 0);
}

static void rc522_task(void *arg)
{
    rc522_handle_t rc522 = (rc522_handle_t)arg;

    while (rc522->running) {
        if (!rc522->scanning) {
            // Idling...
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        uint8_t *uid_bytes = NULL;

        if (ESP_OK != rc522_get_tag(rc522, &uid_bytes)) {
            // Tag is not present
            //
            // TODO: Implement logic to know when the error is due to
            //       tag absence or some other protocol issue
        }

        if (!uid_bytes) {
            rc522->tag_was_present_last_time = false;
        }
        else if (!rc522->tag_was_present_last_time) {
            rc522_tag_t tag = {
                .uid = rc522_uid_to_u64(uid_bytes),
            };
            FREE(uid_bytes);
            rc522_dispatch_event(rc522, RC522_EVENT_TAG_SCANNED, &tag);
            rc522->tag_was_present_last_time = true;
        }
        else {
            FREE(uid_bytes);
        }

        int delay_interval_ms = rc522->config->scan_interval_ms;

        if (rc522->tag_was_present_last_time) {
            delay_interval_ms *= 2; // extra scan-bursting prevention
        }

        vTaskDelay(delay_interval_ms / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}
