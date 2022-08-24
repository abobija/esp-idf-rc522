#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <esp_event.h>
#include "driver/spi_master.h"

#define RC522_DEFAULT_MISO                 (25)
#define RC522_DEFAULT_MOSI                 (23)
#define RC522_DEFAULT_SCK                  (19)
#define RC522_DEFAULT_SDA                  (22)
#define RC522_DEFAULT_SPI_HOST             (VSPI_HOST)
#define RC522_DEFAULT_SCAN_INTERVAL_MS     (125)
#define RC522_DEFAULT_TACK_STACK_SIZE      (4 * 1024)
#define RC522_DEFAULT_TACK_STACK_PRIORITY  (4)

ESP_EVENT_DECLARE_BASE(RC522_EVENTS);

typedef struct rc522* rc522_handle_t;

typedef struct {
    int miso_io;                    /*<! MFRC522 MISO gpio (Default: 25) */
    int mosi_io;                    /*<! MFRC522 MOSI gpio (Default: 23) */
    int sck_io;                     /*<! MFRC522 SCK gpio  (Default: 19) */
    int sda_io;                     /*<! MFRC522 SDA gpio  (Default: 22) */
    spi_host_device_t spi_host_id;  /*<! Default VSPI_HOST (SPI3) */
    uint16_t scan_interval_ms;      /*<! How fast will ESP32 scan for nearby tags, in miliseconds. Default: 125ms */
    size_t task_stack_size;         /*<! Stack size of rc522 task (Default: 4 * 1024) */
    uint8_t task_priority;          /*<! Priority of rc522 task (Default: 4) */
} rc522_config_t;

typedef enum {
    RC522_EVENT_ANY = ESP_EVENT_ANY_ID,
    RC522_EVENT_NONE,
    RC522_EVENT_TAG_SCANNED,             /*<! Tag scanned */
} rc522_event_t;

typedef struct {
    rc522_handle_t handle;
    void* ptr;
} rc522_event_data_t;

/**
 * @brief Initialize RC522 module.
 *        To start scanning tags call the rc522_start function.
 * @param config Configuration
 * @param out_handle Pointer to resulting new handle
 * @return ESP_OK on success
 */
esp_err_t rc522_init(rc522_config_t* config, rc522_handle_t* out_handle);

esp_err_t rc522_register_events(rc522_handle_t handle, rc522_event_t event, esp_event_handler_t event_handler, void* event_handler_arg);

esp_err_t rc522_unregister_events(rc522_handle_t handle, rc522_event_t event, esp_event_handler_t event_handler);

/**
 * @brief Convert serial number (array of 5 bytes) to uint64_t number
 * @param sn Serial number
 * @return Serial number in number representation. If fail, 0 will be retured
 */
uint64_t rc522_sn_to_u64(uint8_t* sn);

/**
 * @brief Start to scan tags. If already started, ESP_OK will just be returned. Initialization function had to be
 *        called before this one.
 * @param handle Handle
 * @return ESP_OK on success
 */
esp_err_t rc522_start(rc522_handle_t handle);

/**
 * @brief Start to scan tags. If already started, ESP_OK will just be returned.
 * @param handle Handle
 * @return ESP_OK on success
 */
#define rc522_resume(handle) rc522_start(handle)

/**
 * @brief Pause scan tags. If already paused, ESP_OK will just be returned.
 * @param handle Handle
 * @return ESP_OK on success
 */
esp_err_t rc522_pause(rc522_handle_t handle);

/**
 * @brief Destroy RC522 and free all resources. Cannot be called from event handler.
 * @param handle Handle
 */
void rc522_destroy(rc522_handle_t handle);

#ifdef __cplusplus
}
#endif
