# OreSat CFC Software

Software for Linux version of the CFC (Cirrus Flux Camera) card.

When this project is running on the real hardware it will required the
`prucam-pirt1280` kernel module.
See https://github.com/oresat/oresat-prucam-pirt1280 for more info.

## Quickstart

Install dependencies

```bash
pip install .[dev]
```

Generate Code

```bash
./gen.py
```

Run the CFC app

```bash
python -m oresat_cfc -i vcan0

```
Run the CFC app with mocked hardware

```bash
python -m oresat_cfc -m
```

## UI

The UI is an optional [Bottle]-based website for development, integration,
and testing. After installing bottle and starting the app, it can be found
at `http://localhost:8000`.

Install optional UI

```bash
pip3 install bottle
```

[Bottle]: https://bottlepy.org/docs/dev/
