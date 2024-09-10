# esp-idf-rc522

[![Component Registry](https://components.espressif.com/components/abobija/rc522/badge.svg)](https://components.espressif.com/components/abobija/rc522)

This repository contains ESP-IDF library for communication with MFRC522 RFID card reader, packaged as ESP-IDF component.

> [!NOTE]
> Library currently just reads UID of RFID tags, which is enough for most applications.

## How to use

Run the following command in your ESP-IDF project to install this component:

```bash
idf.py add-dependency "abobija/rc522"
```

## Example

To run basic example, create it as follows:

```bash
idf.py create-project-from-example "abobija/rc522:basic"
```

Then build it as usual:

```bash
cd basic
idf.py build
```

And flash it to the board:

```bash
idf.py -p PORT flash monitor
```

The example uses [helpers/spi.h](include/rc522/helpers/spi.h) for wiring GPIOs.

## Author

GitHub: [abobija](https://github.com/abobija)<br>
Homepage: [abobija.com](https://abobija.com)

## License

[MIT](LICENSE)
