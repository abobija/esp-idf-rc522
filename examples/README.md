# Examples

In this directory you can find examples how to use [abobija/rc522](https://components.espressif.com/components/abobija/rc522) component inside of your project.

## How to run example

Each directory here represents one example. If you want to run [basic](basic) example, create it as follows:

```bash
idf.py create-project-from-example "abobija/rc522:basic"
```

> [!NOTE]
> Note that in previous command, instead of `basic` you can put name of any other example (e.g. `i2c`)

Then build it as usual

```bash
cd basic
idf.py build
```

And flash it to the board:

```bash
idf.py -p PORT flash monitor
```

Visit [basic](basic) example to check wiring RC522 with ESP chip for this particular example.
