#pragma once

#include "rc522_types.h"

#ifdef __cplusplus
extern "C" {
#endif

ESP_EVENT_DECLARE_BASE(RC522_EVENTS);

esp_err_t rc522_create(const rc522_config_t *config, rc522_handle_t *out_rc522);

esp_err_t rc522_register_events(
    const rc522_handle_t rc522, rc522_event_t event, esp_event_handler_t event_handler, void *event_handler_arg);

esp_err_t rc522_unregister_events(const rc522_handle_t rc522, rc522_event_t event, esp_event_handler_t event_handler);

esp_err_t rc522_start(rc522_handle_t rc522);

esp_err_t rc522_pause(rc522_handle_t rc522);

esp_err_t rc522_destroy(rc522_handle_t rc522);

#ifdef __cplusplus
}
#endif
