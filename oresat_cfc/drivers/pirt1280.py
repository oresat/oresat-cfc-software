import io
import os
import random
import logging
from time import sleep
from enum import IntEnum

import spidev
import gpio
import numpy as np

# fix gpio enabling all logging for other modules
logging.getLogger().removeHandler(logging.getLogger().handlers[0])

sensor_enable_gpio = 86

# SPI 100KHz
spi_hz = 100000


class Register(IntEnum):
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


# OR with address to write
reg_wr = 0x40

# number of seconds to wait after writing a register before reading it back
read_back_wait = 0.1

# refclk
refclk = 16000000  # 16MHz
refclk_cycle = 1 / refclk  # 16MHz period

# row params
default_cws = 1400
dummy_pixels = 8
sync_pixels = 2
bytes_per_pixel = 2
rows = 1024
pix_clks_per_refclk = 4  # 4 64MHz clocks per 16MHZ refclk

# calculate the readout time
readout_refclks = (((default_cws + dummy_pixels + sync_pixels) * bytes_per_pixel) * rows) \
    / pix_clks_per_refclk
readout_margin = 1.1  # scalar to increase readout time for safety


class PIRT1280:
    '''The PIRT1280 camera'''

    COLS = 1280
    ROWS = 1024
    PIXEL_BYTES = COLS * ROWS * 2
    PRUCAM_PATH = '/dev/prucam'

    def __init__(self, mock: bool = False):

        self.mock = mock  # TODO

        # init SPI
        if not mock:
            self.spi = spidev.SpiDev()
            self.spi.open(1, 1)  # /dev/spidev1.1
            self.spi.max_speed_hz = spi_hz

    def __del__(self):

        self.disable()

    def enable(self):

        # set the enable GPIO high
        if not self.mock:
            gpio.setup(sensor_enable_gpio, gpio.OUT)
            gpio.set(sensor_enable_gpio, 1)

        sleep(read_back_wait)

        # TODO set ROFF register to 8 to start at row 8
        # TODO I might be able to turn down CWS or use defauth of 1279

        self.set_16b_reg(8, Register.ROFF0, Register.ROFF1)
        self.set_16b_reg(8, Register.COFF0, Register.COFF1)
        self.set_16b_reg(32, Register.HB0, Register.HB1)
        self.set_single_lane()
        self.set_integration(0.001)

    def disable(self):

        if not self.mock:
            gpio.setup(sensor_enable_gpio, gpio.OUT)
            gpio.set(sensor_enable_gpio, 0)

    def set_single_lane(self):

        if self.mock:
            return

        # read current CONF1 value
        read = self._read_reg(Register.CONF1)

        # clear bits 6 & 7 of CONF1 and write it back
        new = read[0] & 0x3F
        self.set_8b_reg(new, Register.CONF1)

        # wait a sec for it to apply
        sleep(read_back_wait)

        # read back CONF1
        read = self._read_reg(Register.CONF1)

    def set_16b_reg(self, val: int, reg0: Register, reg1: Register):
        '''
        set_16b_reg writes the passed value to the passed registers as a 16 bit
        little-endian integer.
        '''

        # convert the value to little-endian bytes
        b = val.to_bytes(2, 'little')

        # write the register
        if not self.mock:
            self.spi.writebytes([reg0.value | reg_wr, b[0]])
            self.spi.writebytes([reg1.value | reg_wr, b[1]])

        # wait a sec for it to apply
        sleep(read_back_wait)

    def set_8b_reg(self, val: int, reg: Register):

        if not self.mock:
            self.spi.writebytes([reg.value | reg_wr, val.to_bytes(1, 'little')])

    def set_integration(self, intr_seconds):

        # from the specified number of integration, get the number of integration
        # refclks, rounding down the float
        intr_refclks = int(intr_seconds / refclk_cycle)

        # validate the integration time
        if intr_refclks < 513:
            intr_refclks = 513

        # calculate the number of refclks in a frame by adding the number of refclks
        # of integration plus the number of refclks it takes to read out, with a little
        # bit of margin
        frame_refclks = int(intr_refclks + (readout_refclks * readout_margin))

        # convert the frame time and integration time refclks value to little-endian
        # bytes, so the first byte is the lowest byte, which is how it should be written
        # to the register

        # frb == frame refclk bytes
        frb = frame_refclks.to_bytes(4, 'little')

        # irb == integration reflcks bytes
        irb = intr_refclks.to_bytes(4, 'little')

        # write the frame time register
        self.set_8b_reg(frb[0], Register.FT0)
        self.set_8b_reg(frb[1], Register.FT0)
        self.set_8b_reg(frb[2], Register.FT0)
        self.set_8b_reg(frb[3], Register.FT0)

        # write the integration time register
        self.set_8b_reg(irb[0], Register.IT0)
        self.set_8b_reg(irb[1], Register.IT1)
        self.set_8b_reg(irb[2], Register.IT2)
        self.set_8b_reg(irb[3], Register.IT3)

        # wait a sec for it to apply
        sleep(read_back_wait)

    def _read_reg(self, reg: int) -> bytes:

        if self.mock:
            return b'\x00'

        self.spi.writebytes([reg])
        return self.spi.readbytes(1)

    def capture(self, intr: float = 0.0) -> bytes:

        if self.mock:
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
        imgbuf = bytes(self.PIXEL_BYTES)

        # read from prucam into buffer
        fio.readinto(imgbuf)

        # close the char device
        os.close(fd)

        return imgbuf

    def capture_as_np_array(self, intr: float = 0.0) -> np.ndarray:

        raw = self.capture(intr)
        data = np.frombuffer(raw, dtype=np.uint16).reshape(self.ROWS, self.COLS)
        return data
