'''
The PIRT1280 camera driver.
'''

import io
import os
import random
from time import sleep
from enum import IntEnum

import gpio
import numpy as np
from spidev import SpiDev


class Pirt1280Error(Exception):
    '''An error with Pirt1280'''


class Pirt128Register(IntEnum):
    '''PIRT1280 register addresses'''

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
    VLO = 27
    CONF0 = 38
    CONF1 = 39
    CONF2 = 40
    CONF3 = 41
    NCP = 46
    CONF4 = 49


class Pirt1280OutputMode(IntEnum):

    ONE = 0x00  # or 0x03 ; bits 0 and 1 must be the same
    TWO = 0x02


class Pirt1280:
    '''The PIRT1280 camera'''

    COLS = 1280
    ROWS = 1024
    BYTES_PER_PIXEL = 2
    PIXEL_BYTES = COLS * ROWS * BYTES_PER_PIXEL

    PRUCAM_PATH = '/dev/prucam'

    SPI_HZ = 100_000  # SPI 100KHz

    # OR with address to write
    REG_WR = 0x40

    # number of seconds to wait after writing a register before reading it back
    READ_BACK_WAIT = 0.1

    # refclk
    REFCLK = 16_000_000  # 16MHz
    REFCLK_CYCLE = 1 / REFCLK  # 16MHz period

    # row params
    DEFAULT_CWS = 1400
    DUMMY_PIXELS = 8
    SYNC_PIXELS = 2
    PIX_CLKS_PER_REFCLK = 4  # 4 64MHz clocks per 16MHZ refclk

    # calculate the readout time
    READOUT_REFCLKS = (((DEFAULT_CWS + DUMMY_PIXELS + SYNC_PIXELS) * BYTES_PER_PIXEL) * ROWS) \
        / PIX_CLKS_PER_REFCLK
    READOUT_MARGIN = 1.1  # scalar to increase readout time for safety

    def __init__(self, spi_bus: int, spi_device: int, gpio_num: int, mock: bool = False):

        self._gpio_num = gpio_num
        self._mock = mock

        if mock:
            self._mock_regs = [0] * (list(Pirt128Register)[-1].value + 1)
        else:
            self._spi = SpiDev()
            self._spi.open(spi_bus, spi_device)
            self._spi.max_speed_hz = self.SPI_HZ

    def __del__(self):

        self.disable()

    def enable(self):
        '''Enable the PIRT1280 (power it on).'''

        # set the enable GPIO high
        if not self._mock:
            gpio.setup(self._gpio_num, gpio.OUT)
            gpio.set(self._gpio_num, gpio.HIGH)

        sleep(self.READ_BACK_WAIT)

        # TODO set ROFF register to 8 to start at row 8
        # TODO I might be able to turn down CWS or use defauth of 1279

        self._write_16b_reg(8, Pirt128Register.ROFF0, Pirt128Register.ROFF1)
        self._write_16b_reg(8, Pirt128Register.COFF0, Pirt128Register.COFF1)
        self._write_16b_reg(32, Pirt128Register.HB0, Pirt128Register.HB1)

        self.output_mode = Pirt1280OutputMode.ONE
        self.integration_time = 0.001

    def disable(self):
        '''Disable the PIRT1280 (power it off).'''

        if not self._mock:
            gpio.setup(self._gpio_num, gpio.OUT)
            gpio.set(self._gpio_num, gpio.LOW)

    def _read_8b_reg(self, reg: Pirt128Register) -> int:
        '''Read a 8-bit int from a register.'''

        if self._mock:
            return self._mock_regs[reg.value]

        self._spi.writebytes([reg.value])
        return self._spi.readbytes(1)[0]

    def _write_8b_reg(self, value: int, reg: Pirt128Register):
        '''Write a 8-bit int to a register.'''

        if self._mock:
            self._mock_regs[reg.value] = value
        else:
            self._spi.writebytes([reg.value | self.REG_WR, value])

        # wait a sec for it to apply
        sleep(self.READ_BACK_WAIT)

    def _write_16b_reg(self, value: int, reg0: Pirt128Register, reg1: Pirt128Register):
        '''Write a 16-bit int to a pair of registers.'''

        # convert the value to little-endian bytes
        b = value.to_bytes(2, 'little')

        # write the register
        if self._mock:
            self._mock_regs[reg0.value] = b[0]
            self._mock_regs[reg1.value] = b[1]
        else:
            self._spi.writebytes([reg0.value | self.REG_WR, b[0]])
            self._spi.writebytes([reg1.value | self.REG_WR, b[1]])

        # wait a sec for it to apply
        sleep(self.READ_BACK_WAIT)

    def capture(self) -> bytes:
        '''
        Capure a image as raw bytes.

        Returns
        -------
        bytes:
            The raw capture data.
        '''

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
    def output_mode(self) -> Pirt1280OutputMode:
        '''Pirt1280OutputMode: The number of outputs enabled.'''

        value = self._read_8b_reg(Pirt128Register.CONF1)
        value >>= 6
        value &= 0x3

        if value in [0x3, 0x0]:
            return Pirt1280OutputMode.ONE
        elif value == 0x02:
            return Pirt1280OutputMode.TWO
        else:
            raise Pirt1280Error('Invalueid Output Mode of 0x3')

    @output_mode.setter
    def output_mode(self, mode: Pirt1280OutputMode):

        mode_raw = mode.value << 6

        # read current CONF1 value
        read = self._read_8b_reg(Pirt128Register.CONF1)

        # override bits 6 & 7 of CONF1 and write it back
        new = read & 0x3F
        new |= mode_raw
        self._write_8b_reg(new, Pirt128Register.CONF1)

    @property
    def integration_time(self) -> float:
        '''float: The integration time in seconds.'''

        raw = bytes([
            self._read_8b_reg(Pirt128Register.IT0),
            self._read_8b_reg(Pirt128Register.IT1),
            self._read_8b_reg(Pirt128Register.IT2),
            self._read_8b_reg(Pirt128Register.IT3),
        ])

        intr_refclks = int.from_bytes(raw, 'little')

        return intr_refclks * self.REFCLK_CYCLE

    @integration_time.setter
    def integration_time(self, value: float):

        # from the specified number of integration_time, get the number of integration_time
        # refclks, rounding down the float
        intr_refclks = int(value / self.REFCLK_CYCLE)

        # valueidate the integration_time time
        if intr_refclks < 513:
            intr_refclks = 513

        # calculate the number of refclks in a frame by adding the number of refclks
        # of integration_time plus the number of refclks it takes to read out, with a little
        # bit of margin
        frame_refclks = int(intr_refclks + (self.READOUT_REFCLKS * self.READOUT_MARGIN))

        # convert the frame time and integration_time time refclks value to little-endian
        # bytes, so the first byte is the lowest byte, which is how it should be written
        # to the register

        # frb == frame refclk bytes
        frb = frame_refclks.to_bytes(4, 'little')

        # irb == integration_time reflcks bytes
        irb = intr_refclks.to_bytes(4, 'little')

        # write the frame time register
        self._write_8b_reg(frb[0], Pirt128Register.FT0)
        self._write_8b_reg(frb[1], Pirt128Register.FT1)
        self._write_8b_reg(frb[2], Pirt128Register.FT2)
        self._write_8b_reg(frb[3], Pirt128Register.FT3)

        # write the integration_time time register
        self._write_8b_reg(irb[0], Pirt128Register.IT0)
        self._write_8b_reg(irb[1], Pirt128Register.IT1)
        self._write_8b_reg(irb[2], Pirt128Register.IT2)
        self._write_8b_reg(irb[3], Pirt128Register.IT3)

        # wait a sec for it to apply
        sleep(self.READ_BACK_WAIT)


def pirt1280_raw_to_numpy(raw: bytes) -> np.ndarray:
    return np.frombuffer(raw, dtype=np.uint16).reshape(Pirt1280.ROWS, Pirt1280.COLS)
