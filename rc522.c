#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <string.h>

#include "rc522.h"

static const char* TAG = "rc522";

/**
 * @brief Macro guard for *alloc functions.
 *        Wrapping memory allocation expression inside of this macro will ensure that
 *        function, in which this macro is called, is going to immediately return
 *        ESP_ERR_NO_MEM if *alloc function fail to reserve the memory.
 * @param EXP Memory allocation expression
 * @return Exit from caller if EXP returns NULL.
*/
#define __alloc_ret_guard(EXP) \
    if((EXP) == NULL) { return ESP_ERR_NO_MEM; }

#define __err_ret_check(EXP) \
    if((err = (EXP)) != ESP_OK) { return err; }

#define __success_label __success_lbl
#define __error_label __error_lbl
#define __exit_label __exit_lbl

#define __labels(on_error, on_success) \
    goto __success_label; \
        __error_label: { on_error; goto __exit_label; };  \
        __success_label: { on_success; goto __exit_label; };  \
        __exit_label: {};

#define __err_jmp_check(EXP) \
    if((err = (EXP)) != ESP_OK) { goto __error_label; }

#define __err_jmp_check_with_log(EXP, message) \
    if((err = (EXP)) != ESP_OK) { ESP_LOGE(TAG, message); goto __error_label; }

#define __err_jmp_condition_with_log(EXP, message) \
    if(EXP) { ESP_LOGE(TAG, message); goto __error_label; }

#define FREE(ptr) \
    if(ptr) { free(ptr); ptr = NULL; }

struct rc522 {
    bool running;                          /*<! Indicates whether rc522 task is running or not */
    rc522_config_t* config;                /*<! Configuration */
    TaskHandle_t task_handle;              /*<! Handle of task */
    esp_event_loop_handle_t event_handle;  /*<! Handle of event loop */
    spi_device_handle_t spi_handle;
    // TODO: Use new 'status' field, instead of initialized, scanning, etc...
    bool initialized;                      /*<! Set on the first start() when configuration is sent to rc522 */
    bool scanning;                         /*<! Whether the rc522 is in scanning or idle mode */
    bool tag_was_present_last_time;
    bool bus_initialized_by_user;          /*<! Whether the bus has been initialized manually by the user, before calling rc522_create function */
};

ESP_EVENT_DEFINE_BASE(RC522_EVENTS);

static esp_err_t rc522_spi_send(rc522_handle_t rc522, uint8_t* buffer, uint8_t length);
static esp_err_t rc522_spi_receive(rc522_handle_t rc522, uint8_t* buffer, uint8_t length, uint8_t addr);
static esp_err_t rc522_i2c_send(rc522_handle_t rc522, uint8_t* buffer, uint8_t length);
static esp_err_t rc522_i2c_receive(rc522_handle_t rc522, uint8_t* buffer, uint8_t length, uint8_t addr);

static void rc522_task(void* arg);

static esp_err_t rc522_write_n(rc522_handle_t rc522, uint8_t addr, uint8_t n, uint8_t *data)
{
    esp_err_t ret;
    uint8_t* buffer = NULL;
    
    __alloc_ret_guard(buffer = (uint8_t*) malloc(n + 1));

    buffer[0] = addr;
    memcpy(buffer + 1, data, n);

    switch(rc522->config->transport) {
        case RC522_TRANSPORT_SPI:
            ret = rc522_spi_send(rc522, buffer, n + 1);
            break;
        case RC522_TRANSPORT_I2C:
            ret = rc522_i2c_send(rc522, buffer, n + 1);
            break;
        default:
            ESP_LOGE(TAG, "write: Unknown transport");
            ret = ESP_ERR_INVALID_STATE; // unknown transport
    }

    free(buffer);

    if(ESP_OK != ret) {
        ESP_LOGE(TAG, "Failed to write data (err: %s)", esp_err_to_name(ret));
    }

    return ret;
}

static inline esp_err_t rc522_write(rc522_handle_t rc522, uint8_t addr, uint8_t val)
{
    return rc522_write_n(rc522, addr, 1, &val);
}

static esp_err_t rc522_read_n(rc522_handle_t rc522, uint8_t addr, uint8_t n, uint8_t* buffer)
{
    esp_err_t ret;

    switch(rc522->config->transport) {
        case RC522_TRANSPORT_SPI:
            ret = rc522_spi_receive(rc522, buffer, n, addr);
            break;
        case RC522_TRANSPORT_I2C:
            ret = rc522_i2c_receive(rc522, buffer, n, addr);
            break;
        default:
            ESP_LOGE(TAG, "read: Unknown transport");
            ret = ESP_ERR_INVALID_STATE; // unknown transport
    }

    if(ESP_OK != ret) {
        ESP_LOGE(TAG, "Failed to read data (err: %s)", esp_err_to_name(ret));
    }

    return ret;
}

static inline esp_err_t rc522_read(rc522_handle_t rc522, uint8_t addr, uint8_t* value_ref)
{
    return rc522_read_n(rc522, addr, 1, value_ref);
}

static esp_err_t rc522_set_bitmask(rc522_handle_t rc522, uint8_t addr, uint8_t mask)
{
    esp_err_t err = ESP_OK;
    uint8_t tmp;

    __err_ret_check(rc522_read(rc522, addr, &tmp));

    return rc522_write(rc522, addr, tmp | mask);
}

static esp_err_t rc522_clear_bitmask(rc522_handle_t rc522, uint8_t addr, uint8_t mask)
{
    esp_err_t err = ESP_OK;
    uint8_t tmp;

    __err_ret_check(rc522_read(rc522, addr, &tmp));

    return rc522_write(rc522, addr, tmp & ~mask);
}

static inline esp_err_t rc522_firmware(rc522_handle_t rc522, uint8_t* result)
{
    return rc522_read(rc522, 0x37, result);
}

static esp_err_t rc522_antenna_on(rc522_handle_t rc522)
{
    esp_err_t err = ESP_OK;
    uint8_t tmp;

    __err_ret_check(rc522_read(rc522, 0x14, &tmp));

    if(~ (tmp & 0x03)) {
        __err_ret_check(rc522_set_bitmask(rc522, 0x14, 0x03));
    }

    return rc522_write(rc522, 0x26, 0x60); // 43dB gain
}

static esp_err_t rc522_clone_config(rc522_config_t* config, rc522_config_t** result)
{
    rc522_config_t* _clone_config = NULL;
    
    __alloc_ret_guard(_clone_config = calloc(1, sizeof(rc522_config_t)));
    
    memcpy(_clone_config, config, sizeof(rc522_config_t));

    // defaults
    _clone_config->scan_interval_ms = config->scan_interval_ms < 50 ? RC522_DEFAULT_SCAN_INTERVAL_MS : config->scan_interval_ms;
    _clone_config->task_stack_size = config->task_stack_size == 0 ? RC522_DEFAULT_TASK_STACK_SIZE : config->task_stack_size;
    _clone_config->task_priority = config->task_priority == 0 ? RC522_DEFAULT_TASK_STACK_PRIORITY : config->task_priority;
    _clone_config->spi.clock_speed_hz = config->spi.clock_speed_hz == 0 ? RC522_DEFAULT_SPI_CLOCK_SPEED_HZ : config->spi.clock_speed_hz;
    _clone_config->i2c.rw_timeout_ms = config->i2c.rw_timeout_ms == 0 ? RC522_DEFAULT_I2C_RW_TIMEOUT_MS : config->i2c.rw_timeout_ms;
    _clone_config->i2c.clock_speed_hz = config->i2c.clock_speed_hz == 0 ? RC522_DEFAULT_I2C_CLOCK_SPEED_HZ : config->i2c.clock_speed_hz;

    *result = _clone_config;

    return ESP_OK;
}

static esp_err_t rc522_create_transport(rc522_handle_t rc522)
{
    esp_err_t err = ESP_OK;

    switch(rc522->config->transport) {
        case RC522_TRANSPORT_SPI: {
                spi_device_interface_config_t devcfg = {
                    .clock_speed_hz = rc522->config->spi.clock_speed_hz,
                    .mode = 0,
                    .spics_io_num = rc522->config->spi.sda_gpio,
                    .queue_size = 7,
                    .flags = rc522->config->spi.device_flags,
                };

                rc522->bus_initialized_by_user = rc522->config->spi.bus_is_initialized;

                if(! rc522->bus_initialized_by_user) {
                    spi_bus_config_t buscfg = {
                        .miso_io_num = rc522->config->spi.miso_gpio,
                        .mosi_io_num = rc522->config->spi.mosi_gpio,
                        .sclk_io_num = rc522->config->spi.sck_gpio,
                        .quadwp_io_num = -1,
                        .quadhd_io_num = -1,
                    };

                    __err_ret_check(spi_bus_initialize(rc522->config->spi.host, &buscfg, 0));
                }

                __err_ret_check(spi_bus_add_device(rc522->config->spi.host, &devcfg, &rc522->spi_handle));
            }
            break;
        case RC522_TRANSPORT_I2C: {
                i2c_config_t conf = {
                    .mode = I2C_MODE_MASTER,
                    .sda_io_num = rc522->config->i2c.sda_gpio,
                    .scl_io_num = rc522->config->i2c.scl_gpio,
                    .sda_pullup_en = GPIO_PULLUP_ENABLE,
                    .scl_pullup_en = GPIO_PULLUP_ENABLE,
                    .master.clk_speed = rc522->config->i2c.clock_speed_hz,
                };

                __err_ret_check(i2c_param_config(rc522->config->i2c.port, &conf));
                __err_ret_check(i2c_driver_install(rc522->config->i2c.port, conf.mode, false, false, 0x00));
            }
            break;
        default:
            ESP_LOGE(TAG, "create_transport: Unknown transport");
            err = ESP_ERR_INVALID_STATE; // unknown transport
            break;
    }

    return err;
}

esp_err_t rc522_create(rc522_config_t* config, rc522_handle_t* out_rc522)
{
    if(! config || ! out_rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;
    rc522_handle_t rc522 = NULL;
    
    __alloc_ret_guard(rc522 = calloc(1, sizeof(struct rc522)));

    __err_jmp_check_with_log(rc522_clone_config(
        config, 
        &(rc522->config)
    ), "Fail to clone config");

    __err_jmp_check_with_log(rc522_create_transport(
        rc522
    ), "Fail to create transport");

    esp_event_loop_args_t event_args = {
        .queue_size = 1,
        .task_name = NULL, // no task will be created
    };

    __err_jmp_check_with_log(esp_event_loop_create(
        &event_args,
        &rc522->event_handle
    ), "Fail to create event loop");

    rc522->running = true;

    __err_jmp_condition_with_log(pdTRUE != xTaskCreate(
        rc522_task,
        "rc522_task",
        rc522->config->task_stack_size,
        rc522,
        rc522->config->task_priority,
        &rc522->task_handle
    ), "Fail to create task");

    __labels({
        rc522_destroy(rc522);
        rc522 = NULL;
    }, {
        *out_rc522 = rc522;
    });

    return err;
}

esp_err_t rc522_register_events(rc522_handle_t rc522, rc522_event_t event, esp_event_handler_t event_handler, void* event_handler_arg)
{
    if(! rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    return esp_event_handler_register_with(rc522->event_handle, RC522_EVENTS, event, event_handler, event_handler_arg);
}

esp_err_t rc522_unregister_events(rc522_handle_t rc522, rc522_event_t event, esp_event_handler_t event_handler)
{
    if(! rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    return esp_event_handler_unregister_with(rc522->event_handle, RC522_EVENTS, event, event_handler);
}

static uint64_t rc522_sn_to_u64(uint8_t* sn)
{
    uint64_t result = 0;

    if(! sn) {
        return 0;
    }

    for(int i = 4; i >= 0; i--) {
        result |= ((uint64_t) sn[i] << (i * 8));
    }

    return result;
}

// Buffer should be length of 2, or more
// Only first 2 elements will be used where the result will be stored
// TODO: Use 2+ bytes data type instead of buffer array
static esp_err_t rc522_calculate_crc(rc522_handle_t rc522, uint8_t *data, uint8_t n, uint8_t* buffer)
{
    esp_err_t err = ESP_OK;
    uint8_t i = 255;
    uint8_t nn = 0;

    __err_ret_check(rc522_clear_bitmask(rc522, 0x05, 0x04));
    __err_ret_check(rc522_set_bitmask(rc522, 0x0A, 0x80));
    __err_ret_check(rc522_write_n(rc522, 0x09, n, data));
    __err_ret_check(rc522_write(rc522, 0x01, 0x03));

    for(;;) {
        __err_ret_check(rc522_read(rc522, 0x05, &nn));

        i--;

        if(! (i != 0 && ! (nn & 0x04))) {
            break;
        }
    }

    // TODO: Use read_n here to read into res[0], res[1]
    uint8_t tmp;

    __err_ret_check(rc522_read(rc522, 0x22, &tmp));
    buffer[0] = tmp;

    __err_ret_check(rc522_read(rc522, 0x21, &tmp));
    buffer[1] = tmp;

    return ESP_OK;
}

static esp_err_t rc522_card_write(rc522_handle_t rc522, uint8_t cmd, uint8_t *data, uint8_t n, uint8_t* res_n, uint8_t** result)
{
    esp_err_t err = ESP_OK;
    uint8_t* _result = NULL;
    uint8_t _res_n = 0;
    uint8_t irq = 0x00;
    uint8_t irq_wait = 0x00;
    uint8_t last_bits = 0;
    uint8_t nn = 0;
    uint8_t tmp;
    
    if(cmd == 0x0E) {
        irq = 0x12;
        irq_wait = 0x10;
    }
    else if(cmd == 0x0C) {
        irq = 0x77;
        irq_wait = 0x30;
    }

    __err_ret_check(rc522_write(rc522, 0x02, irq | 0x80));
    __err_ret_check(rc522_clear_bitmask(rc522, 0x04, 0x80));
    __err_ret_check(rc522_set_bitmask(rc522, 0x0A, 0x80));
    __err_ret_check(rc522_write(rc522, 0x01, 0x00));
    __err_ret_check(rc522_write_n(rc522, 0x09, n, data));
    __err_ret_check(rc522_write(rc522, 0x01, cmd));

    if(cmd == 0x0C) {
        __err_ret_check(rc522_set_bitmask(rc522, 0x0D, 0x80));
    }

    uint16_t i = 1000;

    for(;;) {
        __err_ret_check(rc522_read(rc522, 0x04, &nn));

        i--;

        if(! (i != 0 && (((nn & 0x01) == 0) && ((nn & irq_wait) == 0)))) {
            break;
        }
    }

    __err_ret_check(rc522_clear_bitmask(rc522, 0x0D, 0x80));

    if(i != 0) {
        __err_ret_check(rc522_read(rc522, 0x06, &tmp));

        if((tmp & 0x1B) == 0x00) {
            if(cmd == 0x0C) {
                __err_ret_check(rc522_read(rc522, 0x0A, &nn));
                __err_ret_check(rc522_read(rc522, 0x0C, &tmp));

                last_bits = tmp & 0x07;

                if (last_bits != 0) {
                    _res_n = (nn - 1) + last_bits;
                } else {
                    _res_n = nn;
                }

                if(_res_n > 0) {
                    __alloc_ret_guard(_result = (uint8_t*) malloc(_res_n));

                    for(i = 0; i < _res_n; i++) {
                        __err_ret_check(rc522_read(rc522, 0x09, &tmp));
                        _result[i] = tmp;
                    }
                }
            }
        }
    }

    *res_n = _res_n;
    *result = _result;

    return ESP_OK;
}

static esp_err_t rc522_request(rc522_handle_t rc522, uint8_t* res_n, uint8_t** result)
{
    esp_err_t err = ESP_OK;
    uint8_t* _result = NULL;
    uint8_t _res_n = 0;
    uint8_t req_mode = 0x26;

    __err_ret_check(rc522_write(rc522, 0x0D, 0x07));
    __err_ret_check(rc522_card_write(rc522, 0x0C, &req_mode, 1, &_res_n, &_result));

    if(_res_n * 8 != 0x10) {
        free(_result);
        
        return ESP_ERR_INVALID_STATE;
    }

    *res_n = _res_n;
    *result = _result;

    return err;
}

static esp_err_t rc522_anticoll(rc522_handle_t rc522, uint8_t** result)
{
    esp_err_t err = ESP_OK;
    uint8_t* _result = NULL;
    uint8_t res_n;

    __err_ret_check(rc522_write(rc522, 0x0D, 0x00));
    __err_ret_check(rc522_card_write(rc522, 0x0C, (uint8_t[]) { 0x93, 0x20 }, 2, &res_n, &_result));

    if(_result && res_n != 5) { // all cards/tags serial numbers is 5 bytes long (??)
        free(_result);

        return ESP_ERR_INVALID_RESPONSE;
    }

    *result = _result;

    return err;
}

static esp_err_t rc522_get_tag(rc522_handle_t rc522, uint8_t** result)
{
    esp_err_t err = ESP_OK;
    uint8_t* _result = NULL;
    uint8_t* res_data = NULL;
    uint8_t res_data_n;

    __err_jmp_check(rc522_request(rc522, &res_data_n, &res_data));

    if(res_data != NULL) {
        FREE(res_data);
        __err_jmp_check(rc522_anticoll(rc522, &_result));

        if(_result != NULL) {
            uint8_t buf[] = { 0x50, 0x00, 0x00, 0x00 };
            __err_jmp_check(rc522_calculate_crc(rc522, buf, 2, buf + 2));
            __err_jmp_check(rc522_card_write(rc522, 0x0C, buf, 4, &res_data_n, &res_data));
            FREE(res_data);
            __err_jmp_check(rc522_clear_bitmask(rc522, 0x08, 0x08));
        }
    }

    __labels({
        FREE(_result);
        FREE(res_data);
    }, {
        *result = _result;
    });

    return err;
}

esp_err_t rc522_start(rc522_handle_t rc522)
{
    esp_err_t err = ESP_OK;

    if(! rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    if(rc522->scanning) { // Already in scan mode
        return ESP_OK;
    }

    uint8_t tmp = 0;

    if(! rc522->initialized) {
        // Initialization will be done only once, on the first call of start function

        // ---------- RW test ------------
        const uint8_t test_addr = 0x24, test_val = 0x25;
        uint8_t pass = 0;

        for(uint8_t i = test_val; i < test_val + 2; i++) {
            err = rc522_write(rc522, test_addr, i);

            if(err == ESP_OK) {
                err = rc522_read(rc522, test_addr, &tmp);

                if(err == ESP_OK && tmp == i) {
                    pass = 1;
                }
            }

            if(pass != 1) {
                ESP_LOGE(TAG, "Read/write test failed");
                rc522_destroy(rc522);

                return err;
            }
        }
        // ------- End of RW test --------

        __err_ret_check(rc522_write(rc522, 0x01, 0x0F));
        __err_ret_check(rc522_write(rc522, 0x2A, 0x8D));
        __err_ret_check(rc522_write(rc522, 0x2B, 0x3E));
        __err_ret_check(rc522_write(rc522, 0x2D, 0x1E));
        __err_ret_check(rc522_write(rc522, 0x2C, 0x00));
        __err_ret_check(rc522_write(rc522, 0x15, 0x40));
        __err_ret_check(rc522_write(rc522, 0x11, 0x3D));

        __err_ret_check(rc522_antenna_on(rc522));
        __err_ret_check(rc522_firmware(rc522, &tmp));

        rc522->initialized = true;

        ESP_LOGI(TAG, "Initialized (firmware v%d.0)", (tmp & 0x03));
    }

    rc522->scanning = true;

    return ESP_OK;
}

esp_err_t rc522_pause(rc522_handle_t rc522)
{
    if(! rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    if(! rc522->scanning) {
        return ESP_OK;
    }

    rc522->scanning = false;

    return ESP_OK;
}

static esp_err_t rc522_destroy_transport(rc522_handle_t rc522)
{
    esp_err_t err;

    switch(rc522->config->transport) {
        case RC522_TRANSPORT_SPI:
            err = spi_bus_remove_device(rc522->spi_handle);
            if(rc522->bus_initialized_by_user) {
                err = spi_busFREE(rc522->config->spi.host);
            }
            break;
        case RC522_TRANSPORT_I2C:
            err = i2c_driver_delete(rc522->config->i2c.port);
            break;
        default:
            ESP_LOGW(TAG, "destroy_transport: Unknown transport");
            err = ESP_ERR_INVALID_STATE;
    }

    return err;
}

esp_err_t rc522_destroy(rc522_handle_t rc522)
{
    esp_err_t err = ESP_OK;

    if(! rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    if(xTaskGetCurrentTaskHandle() == rc522->task_handle) {
        ESP_LOGE(TAG, "Cannot destroy rc522 from event handler");

        return ESP_ERR_INVALID_STATE;
    }

    err = rc522_pause(rc522); // stop task
    rc522->running = false; // stop rc522 -> task will delete itself

    // TODO: Wait here for task to exit

    err = rc522_destroy_transport(rc522);

    if(rc522->event_handle) {
        err = esp_event_loop_delete(rc522->event_handle); 
        rc522->event_handle = NULL;
    }

    FREE(rc522->config);
    FREE(rc522);

    return err;
}

static esp_err_t rc522_dispatch_event(rc522_handle_t rc522, rc522_event_t event, void* data)
{
    esp_err_t err;

    if(! rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    rc522_event_data_t e_data = {
        .rc522 = rc522,
        .ptr = data,
    };

    __err_ret_check(esp_event_post_to(
        rc522->event_handle,
        RC522_EVENTS,
        event,
        &e_data,
        sizeof(rc522_event_data_t),
        portMAX_DELAY
    ));

    return esp_event_loop_run(rc522->event_handle, 0);
}

static esp_err_t rc522_spi_send(rc522_handle_t rc522, uint8_t* buffer, uint8_t length)
{
    buffer[0] = (buffer[0] << 1) & 0x7E;

    return spi_device_transmit(rc522->spi_handle, &(spi_transaction_t){
        .length = 8 * length,
        .tx_buffer = buffer,
    });
}

static esp_err_t rc522_spi_receive(rc522_handle_t rc522, uint8_t* buffer, uint8_t length, uint8_t addr)
{
    esp_err_t err = ESP_OK;

    addr = ((addr << 1) & 0x7E) | 0x80;

    if(SPI_DEVICE_HALFDUPLEX & rc522->config->spi.device_flags) {
        __err_ret_check(spi_device_transmit(rc522->spi_handle, &(spi_transaction_t) {
            .flags = SPI_TRANS_USE_TXDATA,
            .length = 8,
            .tx_data[0] = addr,
            .rxlength = 8 * length,
            .rx_buffer = buffer,
        }));
    } else { // Fullduplex
        __err_ret_check(spi_device_transmit(rc522->spi_handle, &(spi_transaction_t) {
            .flags = SPI_TRANS_USE_TXDATA,
            .length = 8,
            .tx_data[0] = addr,
        }));

        __err_ret_check(spi_device_transmit(rc522->spi_handle, &(spi_transaction_t) {
            .flags = 0x00,
            .length = 8,
            .rxlength = 8 * length,
            .rx_buffer = buffer,
        }));
    }

    return err;
}

static inline esp_err_t rc522_i2c_send(rc522_handle_t rc522, uint8_t* buffer, uint8_t length)
{
    return i2c_master_write_to_device(
        rc522->config->i2c.port,
        RC522_I2C_ADDRESS,
        buffer,
        length,
        rc522->config->i2c.rw_timeout_ms / portTICK_PERIOD_MS
    );
}

static inline esp_err_t rc522_i2c_receive(rc522_handle_t rc522, uint8_t* buffer, uint8_t length, uint8_t addr)
{
    return i2c_master_write_read_device(
        rc522->config->i2c.port,
        RC522_I2C_ADDRESS,
        &addr,
        1,
        buffer,
        length,
        rc522->config->i2c.rw_timeout_ms / portTICK_PERIOD_MS
    );
}

static void rc522_task(void* arg)
{
    rc522_handle_t rc522 = (rc522_handle_t) arg;

    while(rc522->running) {
        if(! rc522->scanning) {
            // Idling...
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        uint8_t* serial_no_array = NULL;

        if(ESP_OK != rc522_get_tag(rc522, &serial_no_array)) {
            // Tag is not present
            //
            // TODO: Implement logic to know when the error is due to
            //       tag absence or some other protocol issue
        }
        
        if(! serial_no_array) {
            rc522->tag_was_present_last_time = false;
        } else if(! rc522->tag_was_present_last_time) {
            rc522_tag_t tag = {
                .serial_number = rc522_sn_to_u64(serial_no_array),
            };
            FREE(serial_no_array);
            rc522_dispatch_event(rc522, RC522_EVENT_TAG_SCANNED, &tag);
            rc522->tag_was_present_last_time = true;
        } else {
            FREE(serial_no_array);
        }

        int delay_interval_ms = rc522->config->scan_interval_ms;

        if(rc522->tag_was_present_last_time) {
            delay_interval_ms *= 2; // extra scan-bursting prevention
        }

        vTaskDelay(delay_interval_ms / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}
