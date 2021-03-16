#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/spi_master.h"

typedef void(*rc522_tag_callback_t)(uint8_t*);

typedef struct {
    int miso_io;
    int mosi_io;
    int sck_io;
    int sda_io;
    spi_host_device_t spi_host_id; /*<! Default VSPI_HOST (SPI3) */
    rc522_tag_callback_t callback;
    uint16_t scan_interval_ms; /*<! How fast will ESP32 scan for nearby tags, in miliseconds. Default: 125ms */
} rc522_start_args_t;

esp_err_t rc522_init(rc522_start_args_t* config);
esp_err_t rc522_start(rc522_start_args_t start_args);
esp_err_t rc522_resume();
esp_err_t rc522_pause();
void rc522_destroy();

#ifdef __cplusplus
}
#endif