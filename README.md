# OreSat CFC Software

Software for Linux version of the CFC (Cirrus Flux Camera) card.

Like all OreSat software projects it is built using OLAF (OreSat Linux App
Framework), which it built ontop of [CANopen for Python]. See the
[oresat-olaf repo] for more info about OLAF.

When this project is running on the real hardware it will required the
`prucam-pirt1280` kernel module.
See https://github.com/oresat/oresat-prucam-pirt1280 for more info.

## Quickstart

Install dependenies

```bash
$ pip3 install -r requirements.txt
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

Can mock hardware by using the `-m HARDWARE` flag.

- The`-m all` flag can be used to mock all hardware (CAN bus is always
required).
- The `-m camera` flag would only mock the camera
- The `-m tec` flag would only mock the TEC

See other options with `-h` flag.

A basic [Flask]-based website for development and integration can be found at
`http://localhost:8000` when the software is running.

[Flask]: https://flask.palletsprojects.com/en/latest/
[oresat-olaf repo]: https://github.com/oresat/oresat-olaf
[CANopen for Python]: https://github.com/christiansandberg/canopen
