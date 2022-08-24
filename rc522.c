#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "rc522.h"

static const char* TAG = "rc522";

struct rc522 {
    bool running;
    rc522_config_t* config;
    TaskHandle_t task_handle;
    esp_event_loop_handle_t event_handle;
    spi_device_handle_t spi;
    bool scan_started;
    bool tag_was_present_last_time;
};

ESP_EVENT_DEFINE_BASE(RC522_EVENTS);

#define rc522_fw_version(rc522) rc522_read(rc522, 0x37)

static esp_err_t rc522_spi_init(rc522_handle_t rc522)
{
    if(! rc522 || ! rc522->config) {
        ESP_LOGE(TAG, "Fail to init SPI. Invalid handle");
        return ESP_ERR_INVALID_ARG;
    }

    if(rc522->spi) {
        ESP_LOGW(TAG, "SPI already initialized");
        return ESP_ERR_INVALID_ARG;
    }

    spi_bus_config_t buscfg = {
        .miso_io_num = rc522->config->miso_io,
        .mosi_io_num = rc522->config->mosi_io,
        .sclk_io_num = rc522->config->sck_io,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 5000000,
        .mode = 0,
        .spics_io_num = rc522->config->sda_io,
        .queue_size = 7,
        .flags = SPI_DEVICE_HALFDUPLEX
    };

    esp_err_t err = spi_bus_initialize(rc522->config->spi_host_id, &buscfg, 0);

    if(err != ESP_OK) {
        return err;
    }

    err = spi_bus_add_device(rc522->config->spi_host_id, &devcfg, &rc522->spi);

    if(err != ESP_OK) {
        spi_bus_free(rc522->config->spi_host_id);
        rc522->spi = NULL;
    }

    return err;
}

static esp_err_t rc522_write_n(rc522_handle_t rc522, uint8_t addr, uint8_t n, uint8_t *data)
{
    uint8_t* buffer = (uint8_t*) malloc(n + 1);
    buffer[0] = (addr << 1) & 0x7E;

    for (uint8_t i = 1; i <= n; i++) {
        buffer[i] = data[i-1];
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    t.length = 8 * (n + 1);
    t.tx_buffer = buffer;

    esp_err_t ret = spi_device_transmit(rc522->spi, &t);

    free(buffer);

    return ret;
}

static esp_err_t rc522_write(rc522_handle_t rc522, uint8_t addr, uint8_t val)
{
    return rc522_write_n(rc522, addr, 1, &val);
}

static uint8_t* rc522_read_n(rc522_handle_t rc522, uint8_t addr, uint8_t n)
{
    if (n <= 0) {
        return NULL;
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    uint8_t* buffer = (uint8_t*) malloc(n);
    
    t.flags = SPI_TRANS_USE_TXDATA;
    t.length = 8;
    t.tx_data[0] = ((addr << 1) & 0x7E) | 0x80;
    t.rxlength = 8 * n;
    t.rx_buffer = buffer;

    esp_err_t ret = spi_device_transmit(rc522->spi, &t);
    assert(ret == ESP_OK);

    return buffer;
}

static uint8_t rc522_read(rc522_handle_t rc522, uint8_t addr)
{
    uint8_t* buffer = rc522_read_n(rc522, addr, 1);
    uint8_t res = buffer[0];
    free(buffer);

    return res;
}

static esp_err_t rc522_set_bitmask(rc522_handle_t rc522, uint8_t addr, uint8_t mask)
{
    return rc522_write(rc522, addr, rc522_read(rc522, addr) | mask);
}

static esp_err_t rc522_clear_bitmask(rc522_handle_t rc522, uint8_t addr, uint8_t mask)
{
    return rc522_write(rc522, addr, rc522_read(rc522, addr) & ~mask);
}

static esp_err_t rc522_antenna_on(rc522_handle_t rc522)
{
    esp_err_t ret;

    if(~ (rc522_read(rc522, 0x14) & 0x03)) {
        ret = rc522_set_bitmask(rc522, 0x14, 0x03);

        if(ret != ESP_OK) {
            return ret;
        }
    }

    return rc522_write(rc522, 0x26, 0x60); // 43dB gain
}

static void rc522_task(void* arg);

esp_err_t rc522_init(rc522_config_t* config, rc522_handle_t* out_rc522)
{
    if(! config || ! out_rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;

    rc522_handle_t rc522 = calloc(1, sizeof(struct rc522)); // TODO: memcheck
    rc522->config = calloc(1, sizeof(rc522_config_t)); // TODO: memcheck

    // copy config considering defaults
    rc522->config->miso_io          = config->miso_io == 0 ? RC522_DEFAULT_MISO : config->miso_io;
    rc522->config->mosi_io          = config->mosi_io == 0 ? RC522_DEFAULT_MOSI : config->mosi_io;
    rc522->config->sck_io           = config->sck_io == 0 ? RC522_DEFAULT_SCK : config->sck_io;
    rc522->config->sda_io           = config->sda_io == 0 ? RC522_DEFAULT_SDA : config->sda_io;
    rc522->config->spi_host_id      = config->spi_host_id == 0 ? RC522_DEFAULT_SPI_HOST : config->spi_host_id;
    rc522->config->scan_interval_ms = config->scan_interval_ms < 50 ? RC522_DEFAULT_SCAN_INTERVAL_MS : config->scan_interval_ms;
    rc522->config->task_stack_size  = config->task_stack_size == 0 ? RC522_DEFAULT_TACK_STACK_SIZE : config->task_stack_size;
    rc522->config->task_priority    = config->task_priority == 0 ? RC522_DEFAULT_TACK_STACK_PRIORITY : config->task_priority;

    err = rc522_spi_init(rc522);

    if(err != ESP_OK) {
        rc522_destroy(rc522);
        return err;
    }
    
    // ---------- RW test ------------
    const uint8_t test_addr = 0x24, test_val = 0x25;
    for(uint8_t i = test_val; i < test_val + 2; i++) {
        if((err = rc522_write(rc522, test_addr, i)) != ESP_OK || rc522_read(rc522, test_addr) != i) {
            ESP_LOGE(TAG, "RW test fail");
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

    esp_event_loop_args_t event_args = {
        .queue_size = 1,
        .task_name = NULL, // no task will be created
    };

    if(ESP_OK != (err = esp_event_loop_create(&event_args, &rc522->event_handle))) {
        ESP_LOGE(TAG, "Cannot create event loop");
        rc522_destroy(rc522);
        return err;
    }

    rc522->running = true;
    if (xTaskCreate(rc522_task, "rc522_task", rc522->config->task_stack_size, rc522, rc522->config->task_priority, &rc522->task_handle) != pdTRUE) {
        ESP_LOGE(TAG, "Cannot create task");
        rc522_destroy(rc522);
        return err;
    }

    *out_rc522 = rc522;
    ESP_LOGI(TAG, "Initialized (fw: 0x%x)", rc522_fw_version(rc522));
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

uint64_t rc522_sn_to_u64(uint8_t* sn)
{
    if(!sn) {
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

    uint8_t* res = (uint8_t*) malloc(2); 
    
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

                result = (uint8_t*) malloc(*res_n);

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

    rc522->scan_started = true;
    return ESP_OK;
}

esp_err_t rc522_pause(rc522_handle_t rc522)
{
    if(! rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    if(! rc522->scan_started) {
        return ESP_OK;
    }

    rc522->scan_started = false;

    return ESP_OK;
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

    rc522_pause(rc522); // stop timer
    rc522->running = false; // task will delete himself

    if(rc522->event_handle) {
        esp_event_loop_delete(rc522->event_handle);
        rc522->event_handle = NULL;
    }

    if(rc522->spi) {
        spi_bus_remove_device(rc522->spi);
        spi_bus_free(rc522->config->spi_host_id);
        rc522->spi = NULL;
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

static void rc522_task(void* arg)
{
    rc522_handle_t handle = (rc522_handle_t) arg;

    while(handle->running) {
        if(!handle->scan_started) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        uint8_t* serial_no = rc522_get_tag(handle);

        if(serial_no && ! handle->tag_was_present_last_time) {
            rc522_dispatch_event(handle, RC522_EVENT_TAG_SCANNED, serial_no);
        }

        handle->tag_was_present_last_time = (serial_no != NULL);
        free(serial_no);

        int delay_interval_ms = handle->config->scan_interval_ms;

        if(handle->tag_was_present_last_time) {
            delay_interval_ms *= 2; // extra scan-bursting prevention
        }

        vTaskDelay(delay_interval_ms / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}
