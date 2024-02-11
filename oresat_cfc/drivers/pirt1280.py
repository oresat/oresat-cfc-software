"""
The PIRT1280 SWIR (short-wave infrared) camera driver.
"""

import io
import math as m
import os
import random
from enum import IntEnum
from time import sleep
from typing import Union

import numpy as np
from olaf import Adc, Gpio
from spidev import SpiDev  # pylint: disable=E0611


class Pirt1280Error(Exception):
    """An error with Pirt1280"""


class Pirt1280Register(IntEnum):
    """PIRT1280 register addresses"""

    COM = 1
    COFF0 = 2
    COFF1 = 3
    CWS0 = 4
    CWS1 = 5
    HB0 = 6
    HB1 = 7
    ROFF0 = 8
    ROFF1 = 9
    RWS0 = 10
    RWS1 = 11
    IT0 = 14
    IT1 = 15
    IT2 = 16
    IT3 = 17
    FT0 = 18
    FT1 = 19
    FT2 = 20
    FT3 = 21
    VHI = 26
    VLO = 28
    CONF0 = 38
    CONF1 = 39
    CONF2 = 40
    CONF3 = 41
    NCP = 46
    CONF4 = 49


class Pirt1280:
    """The PIRT1280 SWIR camera"""

    COLS = 1280
    ROWS = 1024
    BYTES_PER_PIXEL = 2
    PIXEL_BYTES = COLS * ROWS * BYTES_PER_PIXEL

    PRUCAM_PATH = "/dev/prucam"

    SPI_HZ = 100_000

    REG_WR = 0x40
    """OR with address to write"""

    READ_BACK_WAIT = 0.2
    """number of seconds to wait after writing a register before reading it back"""

    # refclk
    REFCLK = 16_000_000  # Hz
    REFCLK_CYCLE = 1 / REFCLK  # period

    # row params
    DEFAULT_CWS = 1400
    DUMMY_PIXELS = 8
    SYNC_PIXELS = 2
    PIX_CLKS_PER_REFCLK = 4  # 4 64MHz clocks per 16MHZ refclk

    # calculate the readout time
    READOUT_REFCLKS = (
        ((DEFAULT_CWS + DUMMY_PIXELS + SYNC_PIXELS) * BYTES_PER_PIXEL) * ROWS
    ) / PIX_CLKS_PER_REFCLK
    READOUT_MARGIN = 1.1  # scalar to increase readout time for safety

    R1 = 10_000
    """resistance of the upper resistor in the divider in ohms"""

    INTEGRATION_TIME_MAX_US = 80_000

    def __init__(
        self, spi_bus: int, spi_device: int, gpio_num: int, adc_pin: int, mock: bool = False
    ):
        self._gpio_num = gpio_num
        self._mock = mock
        self._adc = Adc(adc_pin, mock)
        self._gpio = Gpio(gpio_num, mock=mock)
        self._integration_time = -1  # reduce IO calls

        if mock:
            self._mock_regs = [0] * (list(Pirt1280Register)[-1].value + 1)
        else:
            self._spi = SpiDev()
            self._spi.open(spi_bus, spi_device)
            self._spi.max_speed_hz = self.SPI_HZ

        self._enabled = False

    def enable(self):
        """Enable the PIRT1280 (power it on)."""

        if self._enabled:
            return

        # set the enable GPIO high
        if not self._mock:
            self._gpio.high()

        self._enabled = True

        sleep(self.READ_BACK_WAIT)

        self._write_16b_reg(Pirt1280Register.ROFF0.value, 8)
        self._write_16b_reg(Pirt1280Register.COFF0.value, 8)
        self._write_16b_reg(Pirt1280Register.HB0.value, 32)
        self._write_8b_reg(Pirt1280Register.VHI.value, 244)
        self._write_8b_reg(Pirt1280Register.VLO.value, 68)

        # output mode 1
        read_value = self._read_8b_reg(Pirt1280Register.CONF1.value)
        self._write_8b_reg(Pirt1280Register.CONF1.value, read_value & 0x3F)

        self.integration_time = 1000

    def disable(self):
        """Disable the PIRT1280 (power it off)."""

        if not self._mock:
            self._gpio.low()

        self._enabled = False
        self._integration_time = -1

    @property
    def is_enabled(self) -> bool:
        """bool: Pirt1280 enabled."""

        return self._enabled

    def _read_8b_reg(self, reg: int) -> int:
        """Read a 8-bit int from a register."""

        if self._mock:
            return self._mock_regs[reg]

        self._spi.writebytes([reg])
        return self._spi.readbytes(1)[0]

    def _write_8b_reg(self, reg: int, value: int):
        """Write a 8-bit int to a register."""

        if self._mock:
            self._mock_regs[reg] = value
        else:
            self._spi.writebytes([reg | self.REG_WR, value])

        # wait a sec for it to apply
        sleep(self.READ_BACK_WAIT)

        value_read = self._read_8b_reg(reg)

        if value != value_read:
            raise Pirt1280Error(
                f"readback to regs {Pirt1280Register(reg).name} did not match "
                f"0x{value:X} vs 0x{value_read:X}"
            )

    def _write_16b_reg(self, reg: int, value: Union[int, list]):
        """Write a 16-bit int to a pair of registers."""

        # convert the value to little-endian bytes
        if isinstance(value, list):
            b = value
            value = int.from_bytes(bytes(value), "little")
        else:
            b = list(value.to_bytes(2, "little"))
        reg0 = reg
        reg1 = reg + 1

        # write the register
        if self._mock:
            self._mock_regs[reg0] = b[0]
            self._mock_regs[reg1] = b[1]
        else:
            self._spi.writebytes([reg0 | self.REG_WR, b[0]])
            self._spi.writebytes([reg1 | self.REG_WR, b[1]])

        # wait a sec for it to apply
        sleep(self.READ_BACK_WAIT)

        reg0_read = self._read_8b_reg(reg0)
        reg1_read = self._read_8b_reg(reg1)

        value_read = int.from_bytes(bytes([reg0_read, reg1_read]), "little")

        if value != value_read:
            raise Pirt1280Error(
                f"readback to regs {Pirt1280Register(reg).name} did not match "
                f"0x{value:04X} vs 0x{value_read:04X}"
            )

    def capture(self) -> bytes:
        """
        Capure a image as raw bytes.

        Returns
        -------
        bytes:
            The raw capture data.
        """

        if self._mock:
            return bytes([random.randint(0, 255) for i in range(self.PIXEL_BYTES)])

        # open the prucam char device
        fd = os.open(self.PRUCAM_PATH, os.O_RDWR)
        fio = io.FileIO(fd, closefd=False)

        # allocate buffer to read frame into
        imgbuf = bytearray(self.PIXEL_BYTES)

        # read from prucam into buffer
        fio.readinto(imgbuf)

        # close the char device
        os.close(fd)

        return bytes(imgbuf)

    @property
    def integration_time(self) -> int:
        """int: The integration time in microseconds."""

        if self.is_enabled:
            return max(self._integration_time, 0)  # max removes the possible -1 flag value
        return 0

    @integration_time.setter
    def integration_time(self, value: int):
        if not self.is_enabled or value == self._integration_time:
            return  # nothing todo

        if value < 0 or value > self.INTEGRATION_TIME_MAX_US:
            raise ValueError(
                f"integration time must be between 1 and {self.INTEGRATION_TIME_MAX_US} "
                "microseconds"
            )

        # from the specified number of integration_time, get the number of integration_time
        # refclks, rounding down the float
        intr_refclks = int((value / 1_000_000) / self.REFCLK_CYCLE)

        # valueidate the integration_time time
        intr_refclks = max(intr_refclks, 513)

        # calculate the number of refclks in a frame by adding the number of refclks
        # of integration_time plus the number of refclks it takes to read out, with a little
        # bit of margin
        frame_refclks = int(intr_refclks + (self.READOUT_REFCLKS * self.READOUT_MARGIN))

        # convert the frame time and integration_time time refclks value to little-endian
        # bytes, so the first byte is the lowest byte, which is how it should be written
        # to the register

        # frb == frame refclk bytes
        frb = frame_refclks.to_bytes(4, "little")

        # irb == integration_time reflcks bytes
        irb = intr_refclks.to_bytes(4, "little")

        # write the frame time registers
        self._write_16b_reg(Pirt1280Register.FT0.value, [frb[0], frb[1]])
        self._write_16b_reg(Pirt1280Register.FT2.value, [frb[2], frb[3]])

        # write the integration_time time registers
        self._write_16b_reg(Pirt1280Register.IT0.value, [irb[0], irb[1]])
        self._write_16b_reg(Pirt1280Register.IT2.value, [irb[2], irb[3]])

        self._integration_time = value

        # wait a sec for it to apply
        sleep(self.READ_BACK_WAIT)

        frb0 = self._read_8b_reg(Pirt1280Register.FT0.value)
        frb1 = self._read_8b_reg(Pirt1280Register.FT1.value)
        frb2 = self._read_8b_reg(Pirt1280Register.FT2.value)
        frb3 = self._read_8b_reg(Pirt1280Register.FT3.value)

        irb0 = self._read_8b_reg(Pirt1280Register.IT0.value)
        irb1 = self._read_8b_reg(Pirt1280Register.IT1.value)
        irb2 = self._read_8b_reg(Pirt1280Register.IT2.value)
        irb3 = self._read_8b_reg(Pirt1280Register.IT3.value)

        frb_read = int.from_bytes(bytes([frb0, frb1, frb2, frb3]), "little")
        irb_read = int.from_bytes(bytes([irb0, irb1, irb2, irb3]), "little")

        if frame_refclks != frb_read:
            raise Pirt1280Error(
                f"readback to FT regs did not match 0x{frame_refclks:X} vs " f"0x{frb_read:X}"
            )
        if intr_refclks != irb_read:
            raise Pirt1280Error(
                f"readback to IT regs did not match 0x{intr_refclks:X} vs " f"0x{irb_read:x}"
            )

    def _get_temp(self) -> float:
        """Get the raw temperature of the sensor."""

        vout = self._adc.value

        # The 10k NTC is part of a voltage divider with a 10k resistor between
        # 1.8v and Vout and the NTC between Vout and ground. Thus, the equation is:
        #
        # Vout = 1.8v * (NTC / (10k + NTC))
        #
        # We know volts and need to solve for NTC resistance, which is:
        #
        # NTC = (10k * Vout) / (1.8v - Vout)
        res = (self.R1 * vout) / (self._adc.ADC_VIN - vout)

        # Per the steinhart/hart equation where A-D are the steinhart coefficients,
        # R25 is the NTC resistance at 25C(10k), and RT is the NTC resistance
        # T=1/(A1+B1*LN(RT/R25)+C1*LN(RT/R25)^2+D1*LN(RT/R25)^3)-273.15

        # steinhart coefficients per Vishay website for part NTCS0805E3103FMT
        # NOTE: don't take the equations off the site it self, export/download the
        # data for the specific part as there are more specifics there.
        # https://www.vishay.com/thermistors/ntc-rt-calculator/
        sh_a = 0.003354016434680530000
        sh_b = 0.000286451700000000000
        sh_c = 0.000003252255000000000
        sh_d = 0.000000045945010000000

        tmp = m.log(res / self.R1)
        temp_k = 1 / (sh_a + (sh_b * tmp) + (sh_c * tmp**2) + (sh_d * m.pow(res / self.R1, 3)))
        temp_c = temp_k - 273.15

        return temp_c

    @property
    def temperature(self) -> float:
        """float: The temperature of the sensor."""

        samples = 10

        samples_sum = 0.0
        for _ in range(samples):
            samples_sum += self._get_temp()

        return samples_sum / samples


def pirt1280_raw_to_numpy(raw: bytes) -> np.ndarray:
    """Convert the raw capture from Pirt1280 to a numpy array."""

    return np.frombuffer(raw, dtype=np.uint16).reshape(Pirt1280.ROWS, Pirt1280.COLS)
