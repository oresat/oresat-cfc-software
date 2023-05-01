import io
import os
from time import sleep

import spidev
import gpio

sensor_enable_gpio = 86

# SPI 100KHz
spi_hz = 100000

# register addresses
com = 0x01
coff0 = 0x02
coff1 = 0x03
cws0 = 0x04
cws1 = 0x05
hb0 = 0x06
hb1 = 0x07
roff0 = 0x08
roff1 = 0x09
rws0 = 0x0a
rws1 = 0x0b
it0 = 0x0e
it1 = 0x0f
it2 = 0x10
it3 = 0x11
ft0 = 0x12
ft1 = 0x13
ft2 = 0x14
ft3 = 0x15
vhi = 0x1a
vlo = 0x1c
conf0 = 0x26
conf1 = 0x27
conf2 = 0x28
conf3 = 0x29
ncp = 0x2e
conf4 = 0x31

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
readout_refclks = (((default_cws + dummy_pixels + sync_pixels) *
                    bytes_per_pixel) * rows) / pix_clks_per_refclk
readout_margin = 1.1  # scalar to increase readout time for safety


class PIRT1280:
    '''The PIRT1280 camera'''

    COLS = 1280
    ROWS = 1024
    PIXEL_BYTES = COLS * ROWS * 2
    PRUCAM_PATH = '/dev/prucam'

    def __init__(self, mock: bool = False):

        self.mock = mock

        # init SPI
        self.spi = spidev.SpiDev()
        self.spi.open(1, 1)  # /dev/spidev1.1
        self.spi.max_speed_hz = spi_hz

    def __del__(self):
        self.disable()

    def enable(self):

        # set the enable GPIO high
        gpio.setup(sensor_enable_gpio, gpio.OUT)
        gpio.set(sensor_enable_gpio, 1)

        sleep(read_back_wait)

        # TODO set ROFF register to 8 to start at row 8
        # TODO I might be able to turn down CWS or use defauth of 1279

        self.set_roff(8)
        self.set_coff(8)
        self.set_hb(32)
        self.set_single_lane()
        self.set_integration(0.001)

    def disable(self):

        gpio.setup(sensor_enable_gpio, gpio.OUT)
        gpio.set(sensor_enable_gpio, 0)

    def set_single_lane(self):
        # read current CONF1 value
        read = self._read_reg(conf1)

        # clear bits 6 & 7 of CONF1 and write it back
        new = read[0] & 0x3F
        self.set_conf1(new)

        # wait a sec for it to apply
        sleep(read_back_wait)

        # read back CONF1
        read = self._read_reg(conf1)

    def set_com(self, val):
        self.spi.writebytes([com | reg_wr, val])

    def set_coff(self, coff):
        '''set_coff sets the column offset registers'''
        self.set_16b_reg('COFF', coff, coff0, coff1)

    def set_cws(self, cws):
        '''set_cws sets the column window size'''
        self.set_16b_reg('CWS', cws, cws0, cws1)

    def set_hb(self, hb):
        '''set_hb sets the horizontal blanking registers'''
        self.set_16b_reg('HB', hb, hb0, hb1)

    def set_roff(self, roff):
        '''set_roff sets the row offset register'''
        self.set_16b_reg('ROFF', roff, roff0, roff1)

    def set_rws(self, rws):
        '''set_rws sets the row window size register'''
        self.set_16b_reg('RWS', rws, rws0, rws1)

    def set_16b_reg(self, name, val, reg0, reg1):
        '''
        set_16b_reg writes the passed value to the passed registers as a 16 bit
        little-endian integer.
        '''

        # convert the value to little-endian bytes
        b = val.to_bytes(2, 'little')

        # write the register
        self.spi.writebytes([reg0 | reg_wr, b[0]])
        self.spi.writebytes([reg1 | reg_wr, b[1]])

        # wait a sec for it to apply
        sleep(read_back_wait)

    def set_conf0(self, val):
        self.spi.writebytes([conf0 | reg_wr, val])

    def set_conf1(self, val):
        self.spi.writebytes([conf1 | reg_wr, val])

    def set_conf2(self, val):
        self.spi.writebytes([conf2 | reg_wr, val])

    def set_conf3(self, val):
        self.spi.writebytes([conf3 | reg_wr, val])

    def set_conf4(self, val):
        self.spi.writebytes([conf4 | reg_wr, val])

    def set_vhi(self, val):
        self.spi.writebytes([vhi | reg_wr, val])

    def set_vlo(self, val):
        self.spi.writebytes([vlo | reg_wr, val])

    def set_ncp(self, val):
        self.spi.writebytes([ncp | reg_wr, val])

    def _write_reg_raw(self, reg: int, val: bytes):
        self.spi.writebytes([reg, val])

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
        self.spi.writebytes([ft0 | reg_wr, frb[0]])
        self.spi.writebytes([ft1 | reg_wr, frb[1]])
        self.spi.writebytes([ft2 | reg_wr, frb[2]])
        self.spi.writebytes([ft3 | reg_wr, frb[3]])

        # write the integration time register
        self.spi.writebytes([it0 | reg_wr, irb[0]])
        self.spi.writebytes([it1 | reg_wr, irb[1]])
        self.spi.writebytes([it2 | reg_wr, irb[2]])
        self.spi.writebytes([it3 | reg_wr, irb[3]])

        # wait a sec for it to apply
        sleep(read_back_wait)

    def _read_reg(self, reg: int) -> bytes:
        self.spi.writebytes([reg])
        return self.spi.readbytes(1)

    def capture(self, intr: float = 0.0) -> bytearray:

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

        return imgbuf
