#include "rc522_driver_private.h"

inline esp_err_t rc522_driver_install(rc522_driver_handle_t driver)
{
    return driver->install(driver);
}

inline esp_err_t rc522_driver_send(rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length)
{
    return driver->send(driver, address, buffer, length);
}

inline esp_err_t rc522_driver_receive(rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length)
{
    return driver->receive(driver, address, buffer, length);
}

inline esp_err_t rc522_driver_uninstall(rc522_driver_handle_t driver)
{
    return driver->uninstall(driver);
}
