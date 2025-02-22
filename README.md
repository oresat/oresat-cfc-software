# OreSat CFC Software

Software for Linux version of the CFC (Cirrus Flux Camera) card.

When this project is running on the real hardware it will required the
`prucam-pirt1280` kernel module.
See https://github.com/oresat/oresat-prucam-pirt1280 for more info.

## Quickstart

Install dependenies

```bash
$ pip3 install .
```
Make a virtual CAN bus

```bash
$ sudo ip link add dev vcan0 type vcan
$ sudo ip link set vcan0 up
```

Run the CFC app

```bash
$ python3 -m oresat_cfc
```

Can select the CAN bus to use (`vcan0`, `can0`, etc) with the `-b BUS` arg.

- The`--mock` or `-m` flag can be used to mock all hardware (the CAN bus is
  always required).

See other options with `-h` flag.

## UI

The UI is a optional [Bottle]-based website for development, integration,
and testing. After installing bottle and starting the app, it can can be found
at `http://localhost:8000`.

Install optional UI

```bash
$ pip3 install bottle
```

[Bottle]: https://bottlepy.org/docs/dev/
