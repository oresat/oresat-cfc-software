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


class Pirt128Register(IntEnum):
    '''PIRT1280 register addresses'''

    COM = 0x01
    COFF0 = 0x02
    COFF1 = 0x03
    CWS0 = 0x04
    cWS1 = 0x05
    HB0 = 0x06
    HB1 = 0x07
    ROFF0 = 0x08
    ROFF1 = 0x09
    RWS0 = 0x0a
    RWS1 = 0x0b
    IT0 = 0x0e
    IT1 = 0x0f
    IT2 = 0x10
    IT3 = 0x11
    FT0 = 0x12
    FT1 = 0x13
    FT2 = 0x14
    FT3 = 0x15
    VHI = 0x1a
    VLO = 0x1c
    CONF0 = 0x26
    CONF1 = 0x27
    CONF2 = 0x28
    CONF3 = 0x29
    NCP = 0x2e
    CONF4 = 0x31


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

    def __init__(self, gpio_num: int, mock: bool = False):

        self._gpio_num = gpio_num
        self._mock = mock  # TODO

        # init SPI
        if not mock:
            self._spi = SpiDev()
            self._spi.open(1, 1)  # /dev/spidev1.1
            self._spi.max_speed_hz = self.SPI_HZ

    def __del__(self):

        self.disable()

    def enable(self):
        '''
        Enable the PIRT1280 (power it on).
        '''

        # set the enable GPIO high
        if not self._mock:
            gpio.setup(self._gpio_num, gpio.OUT)
            gpio.set(self._gpio_num, 1)

        sleep(self.READ_BACK_WAIT)

        # TODO set ROFF register to 8 to start at row 8
        # TODO I might be able to turn down CWS or use defauth of 1279

        self._set_16b_reg(8, Pirt128Register.ROFF0, Pirt128Register.ROFF1)
        self._set_16b_reg(8, Pirt128Register.COFF0, Pirt128Register.COFF1)
        self._set_16b_reg(32, Pirt128Register.HB0, Pirt128Register.HB1)
        self.set_single_lane()
        self.set_integration(0.001)

    def disable(self):
        '''
        Disable the PIRT1280 (power it off).
        '''

        if not self._mock:
            gpio.setup(self._gpio_num, gpio.OUT)
            gpio.set(self._gpio_num, 0)

    def set_single_lane(self):

        if self._mock:
            return

        # read current CONF1 value
        read = self._read_reg(Pirt128Register.CONF1)

        # clear bits 6 & 7 of CONF1 and write it back
        new = read[0] & 0x3F
        self._set_8b_reg(new, Pirt128Register.CONF1)

        # wait a sec for it to apply
        sleep(self.READ_BACK_WAIT)

        # read back CONF1
        read = self._read_reg(Pirt128Register.CONF1)

    def _set_16b_reg(self, val: int, reg0: Pirt128Register, reg1: Pirt128Register):
        '''
        Write a 16-bit int to a pair of registers.
        '''

        # convert the value to little-endian bytes
        b = val.to_bytes(2, 'little')

        # write the register
        if not self._mock:
            self._spi.writebytes([reg0.value | self.REG_WR, b[0]])
            self._spi.writebytes([reg1.value | self.REG_WR, b[1]])

        # wait a sec for it to apply
        sleep(self.READ_BACK_WAIT)

    def _set_8b_reg(self, val: int, reg: Pirt128Register):
        '''
        Write a 8-bit int to a register.
        '''

        if not self._mock:
            self._spi.writebytes([reg.value | self.REG_WR, val])

    def set_integration(self, intr_seconds: float):
        '''
        Set the integration time.

        Parameters
        ----------
        intr: float
            Integration time in seconds to use while capturing.
        '''

        # from the specified number of integration, get the number of integration
        # refclks, rounding down the float
        intr_refclks = int(intr_seconds / self.REFCLK_CYCLE)

        # validate the integration time
        if intr_refclks < 513:
            intr_refclks = 513

        # calculate the number of refclks in a frame by adding the number of refclks
        # of integration plus the number of refclks it takes to read out, with a little
        # bit of margin
        frame_refclks = int(intr_refclks + (self.READOUT_REFCLKS * self.READOUT_MARGIN))

        # convert the frame time and integration time refclks value to little-endian
        # bytes, so the first byte is the lowest byte, which is how it should be written
        # to the register

        # frb == frame refclk bytes
        frb = frame_refclks.to_bytes(4, 'little')

        # irb == integration reflcks bytes
        irb = intr_refclks.to_bytes(4, 'little')

        # write the frame time register
        self._set_8b_reg(frb[0], Pirt128Register.FT0)
        self._set_8b_reg(frb[1], Pirt128Register.FT1)
        self._set_8b_reg(frb[2], Pirt128Register.FT2)
        self._set_8b_reg(frb[3], Pirt128Register.FT3)

        # write the integration time register
        self._set_8b_reg(irb[0], Pirt128Register.IT0)
        self._set_8b_reg(irb[1], Pirt128Register.IT1)
        self._set_8b_reg(irb[2], Pirt128Register.IT2)
        self._set_8b_reg(irb[3], Pirt128Register.IT3)

        # wait a sec for it to apply
        sleep(self.READ_BACK_WAIT)

    def _read_reg(self, reg: int) -> bytes:

        if self._mock:
            return b'\x00'

        self._spi.writebytes([reg])
        return self._spi.readbytes(1)

    def capture(self, intr: float = 0.0) -> bytes:
        '''
        Capure a image as raw bytes.

        Parameters
        ----------
        intr: float
            Optional integration time in seconds to use while capturing.

        Returns
        -------
        bytes:
            The raw capture data.
        '''

        if self._mock:
            return bytes([random.randint(0, 255) for i in range(self.PIXEL_BYTES)])

        if intr != 0.0:
            # set the integration for this frame
            self.set_integration(intr)

        # wait a moment for settings to apply
        sleep(0.05)

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

    def capture_as_np_array(self, intr: float = 0.0) -> np.ndarray:
        '''
        Capure a image as numpy array.

        Parameters
        ----------
        intr: float
            Optional integration time in seconds to use while capturing.

        Returns
        -------
        np.ndarray:
            The data as a numpy array.
        '''

        raw = self.capture(intr)
        data = np.frombuffer(raw, dtype=np.uint16).reshape(self.ROWS, self.COLS)

        return data
