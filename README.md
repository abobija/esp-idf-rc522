# esp-idf-rc522

[![Component Registry](https://components.espressif.com/components/abobija/rc522/badge.svg)](https://components.espressif.com/components/abobija/rc522)

This repository contains ESP-IDF library (component) for communication with RFID cards using MFRC522 reader.

## Installation

To install latest version of this component to your project, run:

```bash
idf.py add-dependency "abobija/rc522"
```

## Support

- ESP-IDF version: ^5
- Communication protocols: SPI and I2C
- Cards: MIFARE 1K, MIFARE 4K and MIFARE Mini
- Card memory operations:
    - Read and Write ([example](examples/read_write))

## Run example

> [!TIP]
> To find more interesting examples (like [`memory_dump`](examples/memory_dump)), go to [examples](examples) folder.

To run [`basic`](examples/basic) example, create it as follows:

```bash
idf.py create-project-from-example "abobija/rc522:basic"
```

Then build and flash it as usual:

```bash
cd basic
idf.py build flash monitor
```

> [!NOTE]
> Basic example uses SPI communication. Find defined GPIO configuration in [basic.c](examples/basic/main/basic.c) file, while RST pin should be connected to 3.3V.

## Pin Layout

Pin layout is configurable by the user. Check `#define`s of [basic example](examples/basic/main/basic.c) to see how GPIOs are configured.

## Terms

ISO/IEC 14443 uses the following terms for components:
  - `PCD`: proximity coupling device (the card reader)
  - `PICC`: proximity integrated circuit card

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
