#include <string.h>
#include "rc522_helpers_internal.h"
#include "rc522_types_internal.h"
#include "rc522_driver_internal.h"
#include "driver/rc522_spi.h"

RC522_LOG_DEFINE_BASE();

typedef struct
{
    gpio_num_t cs_io_num;
} rc522_spi_meta_t;

static void rc522_spi_transaction_pre_cb(spi_transaction_t *trans);

static void rc522_spi_transaction_post_cb(spi_transaction_t *trans);

static esp_err_t rc522_spi_uninstall(const rc522_driver_handle_t driver);

static esp_err_t rc522_spi_install(const rc522_driver_handle_t driver)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(driver->config == NULL);

    esp_err_t ret = ESP_OK;

    rc522_spi_meta_t *meta = calloc(1, sizeof(rc522_spi_meta_t));
    ESP_GOTO_ON_FALSE(meta != NULL, ESP_ERR_NO_MEM, error, TAG, "nomem");
    driver->meta = (void *)meta;

    rc522_spi_config_t *conf = (rc522_spi_config_t *)(driver->config);

    if (conf->bus_config) {
        // overwrite bus config
        conf->bus_config->quadwp_io_num = GPIO_NUM_NC;
        conf->bus_config->quadhd_io_num = GPIO_NUM_NC;

        ESP_GOTO_ON_ERROR(spi_bus_initialize(conf->host_id, conf->bus_config, conf->dma_chan),
            error,
            TAG,
            "spi bus initialization failed");
    }
    else {
        RC522_LOGD("skips spi bus initialization");
    }

    // {{ overwrite device config
    if (conf->dev_config.clock_speed_hz == 0) {
        conf->dev_config.clock_speed_hz = 5000000;
    }

    conf->dev_config.mode = 0;

    if (conf->dev_config.queue_size == 0) {
        conf->dev_config.queue_size = 7;
    }

    if (conf->dev_config.flags == 0) {
        conf->dev_config.flags = SPI_DEVICE_HALFDUPLEX;
    }

    conf->dev_config.command_bits = 1;
    conf->dev_config.address_bits = 6;
    conf->dev_config.dummy_bits = 1;
    // }}

    // ESP32 SPI bus has limitation of 3 CS lines, so we need to use
    // software control for CS line in order to use more devices.
    {
        meta->cs_io_num = conf->dev_config.spics_io_num;
        conf->dev_config.spics_io_num = GPIO_NUM_NC;

        gpio_config_t cs_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << meta->cs_io_num),
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
        };

        ESP_GOTO_ON_ERROR(gpio_config(&cs_conf), error, TAG, "cs gpio config failed");
        ESP_GOTO_ON_ERROR(gpio_set_level(meta->cs_io_num, 1), error, TAG, "cs set level failed");

        conf->dev_config.pre_cb = &rc522_spi_transaction_pre_cb;
        conf->dev_config.post_cb = &rc522_spi_transaction_post_cb;
    }

    ESP_GOTO_ON_ERROR(spi_bus_add_device(conf->host_id, &conf->dev_config, (spi_device_handle_t *)(&driver->device)),
        error,
        TAG,
        "spi bus add device failed");

    if (conf->rst_io_num > GPIO_NUM_NC) {
        ESP_GOTO_ON_ERROR(rc522_driver_init_rst_pin(conf->rst_io_num), error, TAG, "init rst pin failed");
    }

    goto exit;
error:
    RC522_LOG_ON_ERROR(rc522_spi_uninstall(driver));
exit:
    return ret;
}

static esp_err_t rc522_spi_send(const rc522_driver_handle_t driver, uint8_t address, const rc522_bytes_t *bytes)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(driver->device == NULL);
    RC522_CHECK_BYTES(bytes);

    esp_err_t ret = spi_device_polling_transmit((spi_device_handle_t)(driver->device),
        &(spi_transaction_t) {
            .cmd = RC522_SPI_WRITE,
            .addr = address,
            .length = 8 * bytes->length,
            .tx_buffer = bytes->ptr,
            .user = driver,
        });

    return ret;
}

static esp_err_t rc522_spi_receive(const rc522_driver_handle_t driver, uint8_t address, rc522_bytes_t *bytes)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(driver->device == NULL);
    RC522_CHECK_BYTES(bytes);

    // TODO: Do transactions on higher level
    // RC522_RETURN_ON_ERROR(spi_device_acquire_bus((spi_device_handle_t)(driver->device), portMAX_DELAY));

    for (uint8_t i = 0; i < bytes->length; i++) {
        RC522_RETURN_ON_ERROR(spi_device_polling_transmit((spi_device_handle_t)(driver->device),
            &(spi_transaction_t) {
                .cmd = RC522_SPI_READ,
                .addr = address,
                .rxlength = 8,
                .rx_buffer = (bytes->ptr + i),
                .user = driver,
            }));
    }

    // spi_device_release_bus((spi_device_handle_t)(driver->device));

    return ESP_OK;
}

static esp_err_t rc522_spi_reset(const rc522_driver_handle_t driver)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(driver->config == NULL);

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

static esp_err_t rc522_spi_uninstall(const rc522_driver_handle_t driver)
{
    RC522_CHECK(driver == NULL);

    if (driver->meta) {
        free(driver->meta);
        driver->meta = NULL;
    }

    if (driver->device) {
        RC522_LOG_ON_ERROR(spi_bus_remove_device((spi_device_handle_t)(driver->device)));
        driver->device = NULL;
    }

    if (driver->config) {
        rc522_spi_config_t *conf = (rc522_spi_config_t *)(driver->config);

        if (conf->bus_config) {
            RC522_LOG_ON_ERROR(spi_bus_free(conf->host_id));
        }
    }

    return ESP_OK;
}

esp_err_t rc522_spi_create(const rc522_spi_config_t *config, rc522_driver_handle_t *driver)
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

static void rc522_spi_transaction_pre_cb(spi_transaction_t *trans)
{
    rc522_driver_handle_t driver = (rc522_driver_handle_t)trans->user;
    rc522_spi_meta_t *meta = (rc522_spi_meta_t *)(driver->meta);
    RC522_LOG_ON_ERROR(gpio_set_level(meta->cs_io_num, 0));
}

static void rc522_spi_transaction_post_cb(spi_transaction_t *trans)
{
    rc522_driver_handle_t driver = (rc522_driver_handle_t)trans->user;
    rc522_spi_meta_t *meta = (rc522_spi_meta_t *)(driver->meta);
    RC522_LOG_ON_ERROR(gpio_set_level(meta->cs_io_num, 1));
}
