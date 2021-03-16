# esp-idf-rc522

C library for interfacing ESP32 with MFRC522 RFID card reader

## Demo

[![How To Connect MFRC522 With ESP32 | ESP IDF Component](https://img.youtube.com/vi/IHaccsDMg9s/mqdefault.jpg)](https://www.youtube.com/watch?v=IHaccsDMg9s)

## How to use

This directory is an ESP-IDF component. Clone it (or add it as submodule) into `components` directory of the project.

## Example

```c
#include "esp_log.h"
#include "rc522.h"

static const char* TAG = "app";

void tag_handler(uint8_t* sn) { // serial number is always 5 bytes long
    ESP_LOGI(TAG, "Tag: %#x %#x %#x %#x %#x",
        sn[0], sn[1], sn[2], sn[3], sn[4]
    );
}

void app_main(void) {
    const rc522_start_args_t start_args = {
        .miso_io  = 25,
        .mosi_io  = 23,
        .sck_io   = 19,
        .sda_io   = 22,
        .callback = &tag_handler,

        // Uncomment next line for attaching RC522 to SPI2 bus. Default is VSPI_HOST (SPI3)
        //.spi_host_id = HSPI_HOST
    };

    rc522_start(start_args);
}
```

## Author

GitHub: [abobija](https://github.com/abobija)<br>
Homepage: [abobija.com](https://abobija.com)

## License

[MIT](LICENSE)