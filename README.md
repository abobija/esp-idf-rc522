# esp-idf-rc522

[![Component Registry](https://img.shields.io/github/v/release/abobija/esp-idf-rc522?sort=date&display_name=release&style=for-the-badge&logo=espressif&logoColor=white&label=Latest%20version)](https://components.espressif.com/components/abobija/rc522)

This repository contains [ESP-IDF](https://github.com/espressif/esp-idf) library (component) for communication with RFID cards using [MFRC522](https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf) reader.

## Installation

To install latest version of this component to your project, run:

```bash
idf.py add-dependency "abobija/rc522"
```

## Terms

| Term | Description |
| ---- | ----------- |
| PCD  | Proximity Coupling Device (the card reader). In our case this is MFRC522 module |
| PICC | Proximity Integrated Circuit Card (e.g: rfid card, tag, ...) |

## Support

- Cards: `MIFARE 1K`, `MIFARE 4K` and `MIFARE Mini`
- Card operations:
    - Read and write to memory blocks ([example](examples/read_write))
- Communication protocols: `SPI` and `I2C`
- ESP-IDF version: `^5`

## Example

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
> [`basic`](examples/basic) example uses SPI communication. Find defined GPIO configuration in [basic.c](examples/basic/main/basic.c) file.

## Pin Layout

Pin layout is configurable by the user. Check `#define`s of [basic example](examples/basic/main/basic.c) to see how GPIOs are configured. Library currently does not use RST pin so connect it to the 3.3V.

## Unit testing

To run unit tests, go to [`test`](test) directory and set target to `linux`:

```bash
cd test
idf.py --preview set-target linux
```

Then build the project and run tests:

```bash
idf.py build && ./build/test.elf
```


## Additional resources

| Title | Description |
| ----- | ----------- |
| [ISO/IEC 14443](https://en.wikipedia.org/wiki/ISO/IEC_14443) | Identification cards - Contactless integrated circuit cards |
| [ISO/IEC 14443-2](http://www.emutag.com/iso/14443-2.pdf) | Radio frequency power and signal interface |
| [ISO/IEC 14443-3](http://www.emutag.com/iso/14443-3.pdf) | Initialization and anticollision |
| [ISO/IEC 14443-4](http://www.emutag.com/iso/14443-4.pdf) | Transmission protocol |
| [MFRC522](https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf) | MFRC522 - Standard performance MIFARE and NTAG frontend |
| [MF1S50YYX_V1](https://www.nxp.com/docs/en/data-sheet/MF1S50YYX_V1.pdf) | MIFARE Classic EV1 1K |
| [MF1S70YYX_V1](https://www.nxp.com/docs/en/data-sheet/MF1S70YYX_V1.pdf) | MIFARE Classic EV1 4K |


## License

This component is provided under Apache 2.0 license, see [LICENSE](LICENSE) file for details.
