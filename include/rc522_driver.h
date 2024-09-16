#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct rc522_driver_handle *rc522_driver_handle_t;

esp_err_t rc522_driver_install(rc522_driver_handle_t driver);

esp_err_t rc522_driver_send(rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length);

esp_err_t rc522_driver_receive(rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length);

esp_err_t rc522_driver_uninstall(rc522_driver_handle_t driver);

#ifdef __cplusplus
}
#endif
