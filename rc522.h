#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/spi_master.h"

typedef void(*rc522_tag_callback_t)(uint8_t*);

typedef struct {
    int miso_io;                    /*<! MFRC522 MISO gpio (Default: 25) */
    int mosi_io;                    /*<! MFRC522 MOSI gpio (Default: 23) */
    int sck_io;                     /*<! MFRC522 SCK gpio  (Default: 19) */
    int sda_io;                     /*<! MFRC522 SDA gpio  (Default: 22) */
    spi_host_device_t spi_host_id;  /*<! Default VSPI_HOST (SPI3) */
    rc522_tag_callback_t callback;  /*<! Scanned tags handler */
    uint16_t scan_interval_ms;      /*<! How fast will ESP32 scan for nearby tags, in miliseconds. Default: 125ms */
} rc522_config_t;

typedef rc522_config_t rc522_start_args_t;

/**
 * @brief Initialize RC522 module.
 *        To start scanning tags - call rc522_resume or rc522_start2 function.
 * @param config Configuration
 * @return ESP_OK on success
 */
esp_err_t rc522_init(rc522_config_t* config);

/**
 * @brief This function will call rc522_init function and immediately start to scan tags by calling rc522_resume function.
 *        NOTE: This function will be refactored in future to just start scanning without
 *        initialization (same as rc522_resume). For initialization rc522_init will be required to call before this function.
 * @param start_args Configuration
 * @return ESP_OK on success
 */
esp_err_t rc522_start(rc522_start_args_t start_args);

/**
 * @brief Start to scan tags.
 *        NOTE: This function is implemented because in time of implementation rc522_start function is intented for
 *        initialization and scanning in once. In future, when rc522_start gonna be refactored to just start to scan tags
 *        without initialization, this function will be just alias of rc522_start.
 * @return ESP_OK on success
 */
esp_err_t rc522_start2();

/**
 * @brief Start to scan tags
 * @return ESP_OK on success
 */
#define rc522_resume() rc522_start2()

/**
 * @brief Pause scan tags
 * @return ESP_OK on success
 */
esp_err_t rc522_pause();

/**
 * @brief Destroy RC522 and free all resources
 */
void rc522_destroy();

#ifdef __cplusplus
}
#endif