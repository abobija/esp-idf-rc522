#include "rc522_driver.h"

typedef esp_err_t (*rc522_driver_install_handler_t)(rc522_driver_handle_t driver);
typedef esp_err_t (*rc522_driver_send_handler_t)(
    rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length);
typedef esp_err_t (*rc522_driver_receive_handler_t)(
    rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length);
typedef esp_err_t (*rc522_driver_uninstall_handler_t)(rc522_driver_handle_t driver);

struct rc522_driver_handle
{
    rc522_driver_config_t *config;
    rc522_driver_install_handler_t install;
    rc522_driver_send_handler_t send;
    rc522_driver_receive_handler_t receive;
    rc522_driver_uninstall_handler_t uninstall;

    spi_device_handle_t spi;
};

esp_err_t rc522_driver_create(rc522_driver_config_t *config, rc522_driver_handle_t *driver);
esp_err_t rc522_driver_destroy(rc522_driver_handle_t driver);
