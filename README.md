# esp-idf-rc522

C library for interfacing ESP32 with MFRC522 RFID card reader.

> Library currently just reads serial number of RFID tags, which is enough for most applications.

## How to use

This directory is an ESP-IDF component. Clone it (or add it as submodule) into `components` directory of the project.

## Example

This is basic example of scanning RFID tags.

```c
#include <esp_log.h>
#include <inttypes.h>
#include "rc522.h"

static const char* TAG = "rc522-demo";
static rc522_handle_t scanner;

static void rc522_handler(void* arg, esp_event_base_t base, int32_t event_id, void* event_data)
{
    rc522_event_data_t* data = (rc522_event_data_t*) event_data;

    switch(event_id) {
        case RC522_EVENT_TAG_SCANNED: {
                rc522_tag_t* tag = (rc522_tag_t*) data->ptr;
                ESP_LOGI(TAG, "Tag scanned (sn: %" PRIu64 ")", tag->serial_number);
            }
            break;
    }
}

void app_main()
{
    rc522_config_t config = {
        .spi.host = VSPI_HOST,
        .spi.miso_gpio = 25,
        .spi.mosi_gpio = 23,
        .spi.sck_gpio = 19,
        .spi.sda_gpio = 22,
    };

    rc522_create(&config, &scanner);
    rc522_register_events(scanner, RC522_EVENT_ANY, rc522_handler, NULL);
    rc522_start(scanner);
}
```

## FAQ

### **How to use I2C instead of SPI?**

Set the property `.transport` of the config structure to `RC522_TRANSPORT_I2C` and choose GPIOs for data (`.i2c.sda_gpio`) and clock (`.i2c.scl_gpio`):

```c
rc522_config_t config = {
    .transport = RC522_TRANSPORT_I2C,
    .i2c.sda_gpio = 18,
    .i2c.scl_gpio = 19,
};
```

### **How to use halfduplex in SPI transport?**

Set the `.spi.device_flags` property of the config to `SPI_DEVICE_HALFDUPLEX`. Other device flags (`SPI_DEVICE_*`) can be set here as well by chaining them with bitwise OR (`|`) operator.

```c
rc522_config_t config = {
    .spi.host = VSPI_HOST,
    .spi.miso_gpio = 25,
    .spi.mosi_gpio = 23,
    .spi.sck_gpio = 19,
    .spi.sda_gpio = 22,
    .spi.device_flags = SPI_DEVICE_HALFDUPLEX,
};
```

### **How to attach RC522 to existing SPI bus?**

Let's say that spi bus `VSPI_HOST` has been already initialized, and rc522 needs to be attached to that bus. That can be accomplished with the next configuration. Property `.spi.bus_is_initialized` is required to be set to `true` in order to inform library to not initialize spi bus again.

> NOTE: Property `.spi.bus_is_initialized` will be deprecated in the future once when [this issue](https://github.com/espressif/esp-idf/issues/8745) is resolved.

```c
rc522_config_t config = {
    .spi.host = VSPI_HOST,
    .spi.sda_gpio = 22,
    .spi.bus_is_initialized = true,
};
```

## Author

GitHub: [abobija](https://github.com/abobija)<br>
Homepage: [abobija.com](https://abobija.com)

## License

[MIT](LICENSE)
