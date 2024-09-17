# esp-idf-rc522

[![Component Registry](https://components.espressif.com/components/abobija/rc522/badge.svg)](https://components.espressif.com/components/abobija/rc522)

This repository contains ESP-IDF library for communication with MFRC522 RFID card reader, packaged as ESP-IDF component. Library supports SPI and I2C protocols.

## How to use

Run the following command in your ESP-IDF project to install this component:

```bash
idf.py add-dependency "abobija/rc522"
```

## Example project

> [!TIP]
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

Find defined GPIO configuration in [basic.c](examples/basic/main/basic.c) file, while RST pin should be connected to 3.3V.

## Terms

ISO/IEC 14443 uses the following terms for components:

- PCD: proximity coupling device (the card reader)
- PICC: proximity integrated circuit card

## Additional resources

| Title | Description |
| ----- | ----------- |
| [ISO/IEC 14443](https://en.wikipedia.org/wiki/ISO/IEC_14443) | Identification cards -- Contactless integrated circuit cards |
| [ISO/IEC 14443-2](http://www.emutag.com/iso/14443-2.pdf) | Radio frequency power and signal interface |
| [ISO/IEC 14443-3](http://www.emutag.com/iso/14443-3.pdf) | Initialization and anticollision |
| [ISO/IEC 14443-4](http://www.emutag.com/iso/14443-4.pdf) | Transmission protocol |
| [MFRC522](https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf) | MFRC522 - Standard performance MIFARE and NTAG frontend |
| [MF1S50YYX_V1](https://www.nxp.com/docs/en/data-sheet/MF1S50YYX_V1.pdf) | MIFARE Classic EV1 1K |
| [MF1S70YYX_V1](https://www.nxp.com/docs/en/data-sheet/MF1S70YYX_V1.pdf) | MIFARE Classic EV1 4K |


## License

This component is provided under Apache 2.0 license, see [LICENSE](LICENSE) file for details.
