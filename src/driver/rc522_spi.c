#include <string.h>
#include "rc522_helpers_private.h"
#include "rc522_types_private.h"
#include "rc522_driver_private.h"
#include "driver/rc522_spi.h"

RC522_LOG_DEFINE_BASE();

static esp_err_t rc522_spi_install(rc522_driver_handle_t driver)
{
    RC522_CHECK(driver == NULL);

    rc522_spi_config_t *conf = (rc522_spi_config_t *)(driver->config);

    if (conf->bus_config) {
        RC522_RETURN_ON_ERROR(spi_bus_initialize(conf->host_id, conf->bus_config, conf->dma_chan));
    }
    else {
        RC522_LOGD("Skips SPI bus initialization");
    }

    RC522_RETURN_ON_ERROR(
        spi_bus_add_device(conf->host_id, &conf->dev_config, (spi_device_handle_t *)(&driver->device)));

    if (conf->rst_io_num > GPIO_NUM_NC) {
        RC522_RETURN_ON_ERROR(rc522_driver_init_rst_pin(conf->rst_io_num));
    }

    return ESP_OK;
}

static esp_err_t rc522_spi_send(rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length)
{
    address <<= 1;
    address &= (~0x80);

    // FIXME: Find a way to send [address + buffer]
    //        without need for second buffer
    uint8_t buffer2[64];

    buffer2[0] = address;
    memcpy(buffer2 + 1, buffer, length);

    esp_err_t ret = spi_device_polling_transmit((spi_device_handle_t)(driver->device),
        &(spi_transaction_t) {
            .length = 8 * (length + 1),
            .tx_buffer = buffer2,
        });

    return ret;
}

static esp_err_t rc522_spi_receive(rc522_driver_handle_t driver, uint8_t address, uint8_t *buffer, uint8_t length)
{
    address <<= 1;
    address |= 0x80;

    // TODO: Do transactions on higher level
    // RC522_RETURN_ON_ERROR(spi_device_acquire_bus((spi_device_handle_t)(driver->device), portMAX_DELAY));

    for (uint8_t i = 0; i < length; i++) {
        RC522_RETURN_ON_ERROR(spi_device_polling_transmit((spi_device_handle_t)(driver->device),
            &(spi_transaction_t) {
                .length = 8,
                .tx_buffer = &address,
                .rxlength = 8,
                .rx_buffer = (buffer + i),
            }));
    }

    // spi_device_release_bus((spi_device_handle_t)(driver->device));

    return ESP_OK;
}

static esp_err_t rc522_spi_reset(rc522_driver_handle_t driver)
{
    rc522_spi_config_t *conf = (rc522_spi_config_t *)(driver->config);

    if (conf->rst_io_num < 0) {
        return RC522_ERR_RST_PIN_UNUSED;
    }

    RC522_RETURN_ON_ERROR(gpio_set_level(conf->rst_io_num, RC522_DRIVER_HARD_RST_PIN_PWR_DOWN_LEVEL));
    rc522_delay_ms(RC522_DRIVER_HARD_RST_PULSE_DURATION_MS);
    RC522_RETURN_ON_ERROR(gpio_set_level(conf->rst_io_num, !RC522_DRIVER_HARD_RST_PIN_PWR_DOWN_LEVEL));
    rc522_delay_ms(RC522_DRIVER_HARD_RST_PULSE_DURATION_MS);

    return ESP_OK;
}

static esp_err_t rc522_spi_uninstall(rc522_driver_handle_t driver)
{
    RC522_CHECK(driver == NULL);

    RC522_RETURN_ON_ERROR(spi_bus_remove_device((spi_device_handle_t)(driver->device)));
    driver->device = NULL;

    rc522_spi_config_t *conf = (rc522_spi_config_t *)(driver->config);

    if (conf->bus_config) {
        RC522_RETURN_ON_ERROR(spi_bus_free(conf->host_id));
    }

    return ESP_OK;
}

esp_err_t rc522_spi_create(rc522_spi_config_t *config, rc522_driver_handle_t *driver)
{
    RC522_CHECK(config == NULL);
    RC522_CHECK(driver == NULL);

    RC522_RETURN_ON_ERROR(rc522_driver_create(config, sizeof(rc522_spi_config_t), driver));

    (*driver)->install = rc522_spi_install;
    (*driver)->send = rc522_spi_send;
    (*driver)->receive = rc522_spi_receive;
    (*driver)->reset = rc522_spi_reset;
    (*driver)->uninstall = rc522_spi_uninstall;

    return ESP_OK;
}
