#include <driver/gpio.h>
#include "rc522_driver.h"

typedef esp_err_t (*rc522_driver_install_handler_t)(rc522_driver_handle_t driver);

typedef esp_err_t (*rc522_driver_send_handler_t)(
    rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length);

typedef esp_err_t (*rc522_driver_receive_handler_t)(
    rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length);

typedef esp_err_t (*rc522_driver_reset_handler_t)(rc522_driver_handle_t driver);

typedef esp_err_t (*rc522_driver_uninstall_handler_t)(rc522_driver_handle_t driver);

struct rc522_driver_handle
{
    void *config;
    void *device;
    rc522_driver_install_handler_t install;
    rc522_driver_send_handler_t send;
    rc522_driver_receive_handler_t receive;
    rc522_driver_reset_handler_t reset;
    rc522_driver_uninstall_handler_t uninstall;
};

esp_err_t rc522_driver_init_rst_pin(gpio_num_t rst_io_num);

esp_err_t rc522_driver_create(void *config, size_t config_size, rc522_driver_handle_t *driver);

esp_err_t rc522_driver_send(rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length);

esp_err_t rc522_driver_receive(rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length);

esp_err_t rc522_driver_reset(rc522_driver_handle_t driver);

esp_err_t rc522_driver_destroy(rc522_driver_handle_t driver);
