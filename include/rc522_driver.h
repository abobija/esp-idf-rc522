#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct rc522_driver_handle *rc522_driver_handle_t;

esp_err_t rc522_driver_install(const rc522_driver_handle_t driver);

esp_err_t rc522_driver_uninstall(const rc522_driver_handle_t driver);

#ifdef __cplusplus
}
#endif
