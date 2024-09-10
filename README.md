# esp-idf-rc522

[![Component Registry](https://components.espressif.com/components/abobija/rc522/badge.svg)](https://components.espressif.com/components/abobija/rc522)

This repository contains ESP-IDF library for communication with MFRC522 RFID card reader, packaged as ESP-IDF component. Library supports SPI and I2C protocols.

> [!NOTE]
> Component currently supports only reading of UIDs of RFID tags, which is enough for most applications. Feel free to open PR if you want to implement read/write data to the cards.

## How to use

Run the following command in your ESP-IDF project to install this component:

```bash
idf.py add-dependency "abobija/rc522"
```

## Example

> [!NOTE]
> To find more examples, go to [examples](examples) folder.

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

Find defined GPIO configuration in [basic.c](examples/basic/main/basic.c) file.

## License

[MIT](LICENSE)
