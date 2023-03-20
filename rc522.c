#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <string.h>

#include "rc522.h"

static const char* TAG = "rc522";

struct rc522 {
    bool running;                          /*<! Indicates whether rc522 task is running or not */
    rc522_config_t* config;                /*<! Configuration */
    TaskHandle_t task_handle;              /*<! Handle of task */
    esp_event_loop_handle_t event_handle;  /*<! Handle of event loop */
    spi_device_handle_t spi_handle;
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
    uint8_t* buffer = (uint8_t*) malloc(n + 1); // FIXME: memcheck
    buffer[0] = addr;
    memcpy(buffer + 1, data, n);
    esp_err_t ret;
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

static uint8_t* rc522_read_n(rc522_handle_t rc522, uint8_t addr, uint8_t n)
{
    uint8_t* buffer = (uint8_t*) malloc(n); // FIXME: memcheck
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
        free(buffer);
        buffer = NULL;
        ESP_LOGE(TAG, "Failed to read data (err: %s)", esp_err_to_name(ret));
    }
    return buffer;
}

static inline uint8_t rc522_read(rc522_handle_t rc522, uint8_t addr)
{
    uint8_t* buffer = rc522_read_n(rc522, addr, 1);
    uint8_t res = buffer[0];
    free(buffer);

    return res;
}

static inline esp_err_t rc522_set_bitmask(rc522_handle_t rc522, uint8_t addr, uint8_t mask)
{
    return rc522_write(rc522, addr, rc522_read(rc522, addr) | mask);
}

static inline esp_err_t rc522_clear_bitmask(rc522_handle_t rc522, uint8_t addr, uint8_t mask)
{
    return rc522_write(rc522, addr, rc522_read(rc522, addr) & ~mask);
}

static inline uint8_t rc522_firmware(rc522_handle_t rc522)
{
    return rc522_read(rc522, 0x37);
}

static esp_err_t rc522_antenna_on(rc522_handle_t rc522)
{
    esp_err_t err;
    if(~ (rc522_read(rc522, 0x14) & 0x03)) {
        err = rc522_set_bitmask(rc522, 0x14, 0x03);
        if(err != ESP_OK) {
            return err;
        }
    }

    return rc522_write(rc522, 0x26, 0x60); // 43dB gain
}

rc522_config_t* rc522_clone_config(rc522_config_t* config)
{
    rc522_config_t* new_config = calloc(1, sizeof(rc522_config_t)); // FIXME: memcheck
    memcpy(new_config, config, sizeof(rc522_config_t));

    // defaults
    new_config->scan_interval_ms = config->scan_interval_ms < 50 ? RC522_DEFAULT_SCAN_INTERVAL_MS : config->scan_interval_ms;
    new_config->task_stack_size = config->task_stack_size == 0 ? RC522_DEFAULT_TASK_STACK_SIZE : config->task_stack_size;
    new_config->task_priority = config->task_priority == 0 ? RC522_DEFAULT_TASK_STACK_PRIORITY : config->task_priority;
    new_config->spi.clock_speed_hz = config->spi.clock_speed_hz == 0 ? RC522_DEFAULT_SPI_CLOCK_SPEED_HZ : config->spi.clock_speed_hz;
    new_config->i2c.rw_timeout_ms = config->i2c.rw_timeout_ms == 0 ? RC522_DEFAULT_I2C_RW_TIMEOUT_MS : config->i2c.rw_timeout_ms;
    new_config->i2c.clock_speed_hz = config->i2c.clock_speed_hz == 0 ? RC522_DEFAULT_I2C_CLOCK_SPEED_HZ : config->i2c.clock_speed_hz;

    return new_config;
}

static esp_err_t rc522_create_transport(rc522_handle_t rc522)
{
    esp_err_t ret;

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

                    if(ESP_OK != (ret = spi_bus_initialize(rc522->config->spi.host, &buscfg, 0))) {
                        break;
                    }
                }

                ret = spi_bus_add_device(rc522->config->spi.host, &devcfg, &rc522->spi_handle);
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

                if(ESP_OK != (ret = i2c_param_config(rc522->config->i2c.port, &conf))) {
                    break;
                }

                ret = i2c_driver_install(rc522->config->i2c.port, conf.mode, false, false, 0x00);
            }
            break;
        default:
            ESP_LOGE(TAG, "create_transport: Unknown transport");
            ret = ESP_ERR_INVALID_STATE; // unknown transport
            break;
    }

    return ret;
}

esp_err_t rc522_create(rc522_config_t* config, rc522_handle_t* out_rc522)
{
    if(! config || ! out_rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    rc522_handle_t rc522 = calloc(1, sizeof(struct rc522)); // FIXME: memcheck
    rc522->config = rc522_clone_config(config);

    if(ESP_OK != (ret = rc522_create_transport(rc522))) {
        ESP_LOGE(TAG, "Cannot create transport");
        rc522_destroy(rc522);
        return ret;
    }

    esp_event_loop_args_t event_args = {
        .queue_size = 1,
        .task_name = NULL, // no task will be created
    };

    if(ESP_OK != (ret = esp_event_loop_create(&event_args, &rc522->event_handle))) {
        ESP_LOGE(TAG, "Cannot create event loop");
        rc522_destroy(rc522);
        return ret;
    }

    rc522->running = true;
    if (xTaskCreate(rc522_task, "rc522_task", rc522->config->task_stack_size, rc522, rc522->config->task_priority, &rc522->task_handle) != pdTRUE) {
        ESP_LOGE(TAG, "Cannot create task");
        rc522_destroy(rc522);
        return ret;
    }

    *out_rc522 = rc522;
    return ESP_OK;
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
    if(! sn) {
        return 0;
    }

    uint64_t result = 0;
    for(int i = 4; i >= 0; i--) {
        result |= ((uint64_t) sn[i] << (i * 8));
    }

    return result;
}

/* Returns pointer to dynamically allocated array of two element */
static uint8_t* rc522_calculate_crc(rc522_handle_t rc522, uint8_t *data, uint8_t n)
{
    rc522_clear_bitmask(rc522, 0x05, 0x04);
    rc522_set_bitmask(rc522, 0x0A, 0x80);
    rc522_write_n(rc522, 0x09, n, data);
    rc522_write(rc522, 0x01, 0x03);

    uint8_t i = 255;
    uint8_t nn = 0;

    for(;;) {
        nn = rc522_read(rc522, 0x05);
        i--;

        if(! (i != 0 && ! (nn & 0x04))) {
            break;
        }
    }

    uint8_t* res = (uint8_t*) malloc(2); // FIXME: memcheck
    res[0] = rc522_read(rc522, 0x22);
    res[1] = rc522_read(rc522, 0x21);

    return res;
}

static uint8_t* rc522_card_write(rc522_handle_t rc522, uint8_t cmd, uint8_t *data, uint8_t n, uint8_t* res_n)
{
    uint8_t *result = NULL;
    uint8_t irq = 0x00;
    uint8_t irq_wait = 0x00;
    uint8_t last_bits = 0;
    uint8_t nn = 0;
    
    if(cmd == 0x0E) {
        irq = 0x12;
        irq_wait = 0x10;
    }
    else if(cmd == 0x0C) {
        irq = 0x77;
        irq_wait = 0x30;
    }

    rc522_write(rc522, 0x02, irq | 0x80);
    rc522_clear_bitmask(rc522, 0x04, 0x80);
    rc522_set_bitmask(rc522, 0x0A, 0x80);
    rc522_write(rc522, 0x01, 0x00);
    rc522_write_n(rc522, 0x09, n, data);
    rc522_write(rc522, 0x01, cmd);

    if(cmd == 0x0C) {
        rc522_set_bitmask(rc522, 0x0D, 0x80);
    }

    uint16_t i = 1000;

    for(;;) {
        nn = rc522_read(rc522, 0x04);
        i--;

        if(! (i != 0 && (((nn & 0x01) == 0) && ((nn & irq_wait) == 0)))) {
            break;
        }
    }

    rc522_clear_bitmask(rc522, 0x0D, 0x80);

    if(i != 0) {
        if((rc522_read(rc522, 0x06) & 0x1B) == 0x00) {
            if(cmd == 0x0C) {
                nn = rc522_read(rc522, 0x0A);
                last_bits = rc522_read(rc522, 0x0C) & 0x07;

                if (last_bits != 0) {
                    *res_n = (nn - 1) + last_bits;
                } else {
                    *res_n = nn;
                }

                result = (uint8_t*) malloc(*res_n); // FIXME: memcheck

                for(i = 0; i < *res_n; i++) {
                    result[i] = rc522_read(rc522, 0x09);
                }
            }
        }
    }

    return result;
}

static uint8_t* rc522_request(rc522_handle_t rc522, uint8_t* res_n)
{
    uint8_t* result = NULL;
    rc522_write(rc522, 0x0D, 0x07);

    uint8_t req_mode = 0x26;
    result = rc522_card_write(rc522, 0x0C, &req_mode, 1, res_n);

    if(*res_n * 8 != 0x10) {
        free(result);
        return NULL;
    }

    return result;
}

static uint8_t* rc522_anticoll(rc522_handle_t rc522)
{
    uint8_t res_n;

    rc522_write(rc522, 0x0D, 0x00);
    uint8_t* result = rc522_card_write(rc522, 0x0C, (uint8_t[]) { 0x93, 0x20 }, 2, &res_n);

    if(result && res_n != 5) { // all cards/tags serial numbers is 5 bytes long (?)
        free(result);
        return NULL;
    }

    return result;
}

static uint8_t* rc522_get_tag(rc522_handle_t rc522)
{
    uint8_t* result = NULL;
    uint8_t* res_data = NULL;
    uint8_t res_data_n;

    res_data = rc522_request(rc522, &res_data_n);

    if(res_data != NULL) {
        free(res_data);

        result = rc522_anticoll(rc522);

        if(result != NULL) {
            uint8_t buf[] = { 0x50, 0x00, 0x00, 0x00 };
            uint8_t* crc = rc522_calculate_crc(rc522, buf, 2);
            buf[2] = crc[0];
            buf[3] = crc[1];
            free(crc);
            res_data = rc522_card_write(rc522, 0x0C, buf, 4, &res_data_n);
            free(res_data);
            rc522_clear_bitmask(rc522, 0x08, 0x08);

            return result;
        }
    }

    return NULL;
}

esp_err_t rc522_start(rc522_handle_t rc522)
{
    if(! rc522) {
        return ESP_ERR_INVALID_ARG;
    }
    if(rc522->scanning) { // Already in scan mode
        return ESP_OK;
    }

    if(! rc522->initialized) {
        // Initialization will be done only once, on the first call of start function

        esp_err_t err;

        // ---------- RW test ------------
        const uint8_t test_addr = 0x24, test_val = 0x25;
        for(uint8_t i = test_val; i < test_val + 2; i++) {
            if((err = rc522_write(rc522, test_addr, i)) != ESP_OK || rc522_read(rc522, test_addr) != i) {
                ESP_LOGE(TAG, "Read/write test failed");
                rc522_destroy(rc522);
                return err;
            }
        }
        // ------- End of RW test --------

        rc522_write(rc522, 0x01, 0x0F);
        rc522_write(rc522, 0x2A, 0x8D);
        rc522_write(rc522, 0x2B, 0x3E);
        rc522_write(rc522, 0x2D, 0x1E);
        rc522_write(rc522, 0x2C, 0x00);
        rc522_write(rc522, 0x15, 0x40);
        rc522_write(rc522, 0x11, 0x3D);

        rc522_antenna_on(rc522);

        rc522->initialized = true;

        ESP_LOGI(TAG, "Initialized (firmware v%d.0)", (rc522_firmware(rc522) & 0x03));
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

static void rc522_destroy_transport(rc522_handle_t rc522)
{
    switch(rc522->config->transport) {
        case RC522_TRANSPORT_SPI:
            spi_bus_remove_device(rc522->spi_handle);
            if(rc522->bus_initialized_by_user) {
                spi_bus_free(rc522->config->spi.host);
            }
            break;
        case RC522_TRANSPORT_I2C:
            i2c_driver_delete(rc522->config->i2c.port);
            break;
        default:
            ESP_LOGW(TAG, "destroy_transport: Unknown transport");
    }
}

void rc522_destroy(rc522_handle_t rc522)
{
    if(! rc522) {
        return;
    }

    if(xTaskGetCurrentTaskHandle() == rc522->task_handle) {
        ESP_LOGE(TAG, "Cannot destroy rc522 from event handler");
        return;
    }

    rc522_pause(rc522); // stop task
    rc522->running = false; // task will delete himself
    // FIXME: Wait for task to exit
    rc522_destroy_transport(rc522);
    if(rc522->event_handle) {
        esp_event_loop_delete(rc522->event_handle);
        rc522->event_handle = NULL;
    }
    free(rc522->config);
    rc522->config = NULL;
    free(rc522);
    rc522 = NULL;
}

static esp_err_t rc522_dispatch_event(rc522_handle_t rc522, rc522_event_t event, void* data)
{
    if(! rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    rc522_event_data_t e_data = {
        .rc522 = rc522,
        .ptr = data,
    };
    esp_err_t err;
    if(ESP_OK != (err = esp_event_post_to(rc522->event_handle, RC522_EVENTS, event, &e_data, sizeof(rc522_event_data_t), portMAX_DELAY))) {
        return err;
    }

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
    addr = ((addr << 1) & 0x7E) | 0x80;

    esp_err_t ret;

    if(SPI_DEVICE_HALFDUPLEX & rc522->config->spi.device_flags) {
        ret = spi_device_transmit(rc522->spi_handle, &(spi_transaction_t){
            .flags = SPI_TRANS_USE_TXDATA,
            .length = 8,
            .tx_data[0] = addr,
            .rxlength = 8 * length,
            .rx_buffer = buffer,
        });
    } else { // Fullduplex
        if(ESP_OK != (ret = spi_device_transmit(rc522->spi_handle, &(spi_transaction_t){
            .flags = SPI_TRANS_USE_TXDATA,
            .length = 8,
            .tx_data[0] = addr,
        }))) {
            return ret;
        };

        ret = spi_device_transmit(rc522->spi_handle, &(spi_transaction_t){
            .flags = 0x00,
            .length = 8,
            .rxlength = 8 * length,
            .rx_buffer = buffer,
        });
    }

    return ret;
}

static esp_err_t rc522_i2c_send(rc522_handle_t rc522, uint8_t* buffer, uint8_t length)
{
    return i2c_master_write_to_device(rc522->config->i2c.port, RC522_I2C_ADDRESS, buffer, length, rc522->config->i2c.rw_timeout_ms / portTICK_PERIOD_MS);
}

static esp_err_t rc522_i2c_receive(rc522_handle_t rc522, uint8_t* buffer, uint8_t length, uint8_t addr)
{
    return i2c_master_write_read_device(rc522->config->i2c.port, RC522_I2C_ADDRESS, &addr, 1, buffer, length, rc522->config->i2c.rw_timeout_ms / portTICK_PERIOD_MS);
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

        uint8_t* serial_no_array = rc522_get_tag(rc522);
        
        if(! serial_no_array) {
            rc522->tag_was_present_last_time = false;
        } else if(! rc522->tag_was_present_last_time) {
            rc522_tag_t tag = {
                .serial_number = rc522_sn_to_u64(serial_no_array),
            };
            free(serial_no_array);
            rc522_dispatch_event(rc522, RC522_EVENT_TAG_SCANNED, &tag);
            rc522->tag_was_present_last_time = true;
        } else {
            free(serial_no_array);
        }

        int delay_interval_ms = rc522->config->scan_interval_ms;

        if(rc522->tag_was_present_last_time) {
            delay_interval_ms *= 2; // extra scan-bursting prevention
        }

        vTaskDelay(delay_interval_ms / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}
