'''
The PIRT1280 camera driver.
'''

import io
import os
import random
import math as m
from time import sleep
from enum import IntEnum

import cv2
import gpio
import numpy as np
from spidev import SpiDev
from olaf import Adc


class Pirt1280Error(Exception):
    '''An error with Pirt1280'''


class Pirt1280Register(IntEnum):
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


class Pirt1280:
    '''The PIRT1280 camera'''

    COLS = 1280
    ROWS = 1024
    BYTES_PER_PIXEL = 2
    PIXEL_BYTES = COLS * ROWS * BYTES_PER_PIXEL

    PRUCAM_PATH = '/dev/prucam'

    SPI_HZ = 100_000

    REG_WR = 0x40
    '''OR with address to write'''

    READ_BACK_WAIT = 0.1
    '''number of seconds to wait after writing a register before reading it back'''

    # refclk
    REFCLK = 16_000_000  # Hz
    REFCLK_CYCLE = 1 / REFCLK  # period

    # row params
    DEFAULT_CWS = 1400
    DUMMY_PIXELS = 8
    SYNC_PIXELS = 2
    PIX_CLKS_PER_REFCLK = 4  # 4 64MHz clocks per 16MHZ refclk

    # calculate the readout time
    READOUT_REFCLKS = (((DEFAULT_CWS + DUMMY_PIXELS + SYNC_PIXELS) * BYTES_PER_PIXEL) * ROWS) \
        / PIX_CLKS_PER_REFCLK
    READOUT_MARGIN = 1.1  # scalar to increase readout time for safety

    R1 = 10_000
    '''resistance of the upper resistor in the divider in ohms'''

    def __init__(self, spi_bus: int, spi_device: int, gpio_num: int, adc_pin: int,
                 mock: bool = False):

        self._gpio_num = gpio_num
        self._mock = mock
        self._adc = Adc(adc_pin, mock=True)

        if mock:
            self._mock_regs = [0] * (list(Pirt1280Register)[-1].value + 1)
        else:
            self._spi = SpiDev()
            self._spi.open(spi_bus, spi_device)
            self._spi.max_speed_hz = self.SPI_HZ

        self._enable = False

    def __del__(self):

        self.disable()

    def enable(self):
        '''Enable the PIRT1280 (power it on).'''

        # set the enable GPIO high
        if not self._mock:
            gpio.setup(self._gpio_num, gpio.OUT)
            gpio.set(self._gpio_num, gpio.HIGH)

        self._enable = True

        sleep(self.READ_BACK_WAIT)

        self._write_16b_reg(Pirt1280Register.ROFF0.value, 8)
        self._write_16b_reg(Pirt1280Register.COFF0.value, 8)
        self._write_16b_reg(Pirt1280Register.HB0.value, 32)

        # output mode 1
        read_value = self._read_8b_reg(Pirt1280Register.CONF1.value)
        self._write_8b_reg(Pirt1280Register.CONF1.value, read_value & 0x3F)

        self.integration_time = 1

    def disable(self):
        '''Disable the PIRT1280 (power it off).'''

        if not self._mock:
            gpio.setup(self._gpio_num, gpio.OUT)
            gpio.set(self._gpio_num, gpio.LOW)

        self._enable = False

    @property
    def is_enabled(self) -> bool:
        '''bool: Pirt1280 enabled.'''

        return self._enable

    def _read_8b_reg(self, reg: int) -> int:
        '''Read a 8-bit int from a register.'''

        if self._mock:
            return self._mock_regs[reg]

        self._spi.writebytes([reg])
        return self._spi.readbytes(1)[0]

    def _write_8b_reg(self, reg: int, value: int):
        '''Write a 8-bit int to a register.'''

        if self._mock:
            self._mock_regs[reg] = value
        else:
            self._spi.writebytes([reg | self.REG_WR, value])

        # wait a sec for it to apply
        sleep(self.READ_BACK_WAIT)

    def _write_16b_reg(self, reg: int, value: int):
        '''Write a 16-bit int to a pair of registers.'''

        # convert the value to little-endian bytes
        b = value.to_bytes(2, 'little')
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
    def integration_time(self) -> float:
        '''float: The integration time in milliseconds.'''

        raw = bytes([
            self._read_8b_reg(Pirt1280Register.IT0),
            self._read_8b_reg(Pirt1280Register.IT1),
            self._read_8b_reg(Pirt1280Register.IT2),
            self._read_8b_reg(Pirt1280Register.IT3),
        ])

        intr_refclks = int.from_bytes(raw, 'little')

        return intr_refclks * self.REFCLK_CYCLE * 1000

    @integration_time.setter
    def integration_time(self, value: float):

        # from the specified number of integration_time, get the number of integration_time
        # refclks, rounding down the float
        intr_refclks = int((value / 1000) / self.REFCLK_CYCLE)

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
        self._write_8b_reg(Pirt1280Register.FT0.value, frb[0])
        self._write_8b_reg(Pirt1280Register.FT1.value, frb[1])
        self._write_8b_reg(Pirt1280Register.FT2.value, frb[2])
        self._write_8b_reg(Pirt1280Register.FT3.value, frb[3])

        # write the integration_time time register
        self._write_8b_reg(Pirt1280Register.IT0.value, irb[0])
        self._write_8b_reg(Pirt1280Register.IT1.value, irb[1])
        self._write_8b_reg(Pirt1280Register.IT2.value, irb[2])
        self._write_8b_reg(Pirt1280Register.IT3.value, irb[3])

        # wait a sec for it to apply
        sleep(self.READ_BACK_WAIT)

    def _get_temp(self) -> float:
        '''Get the raw temperature of the sensor.'''

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
        temp_k = 1 / (sh_a + (sh_b * tmp) + (sh_c * tmp ** 2) + (sh_d * m.pow(res / self.R1, 3)))
        temp_c = temp_k - 273.15

        return temp_c

    @property
    def temperature(self) -> float:
        '''float: The temperature of the sensor.'''

        samples: int = 10

        samples_sum = 0
        for i in range(samples):
            samples_sum += self._get_temp()

        return samples_sum / samples


def pirt1280_raw_to_numpy(raw: bytes) -> np.ndarray:
    '''Convert the raw capture from Pirt1280 to a numpy array.'''

    return np.frombuffer(raw, dtype=np.uint16).reshape(Pirt1280.ROWS, Pirt1280.COLS)


def pirt1280_make_display_image(raw: bytes, ext: str = '.jpg', sat_percent: int = 0,
                                downscale_factor: int = 1) -> bytes:
    '''
    Generate an image to send to UI (heavy manipulated image for UI display).

    Parameters
    ----------
    raw: bytes
        The raw PIRT1280 data.
    ext: str
        The extension of the file to generate.
    sat_percent: int
        Option to color pixel above a saturation percentage red. Set to 0 to disable.
    downscale_factor: int
        Downscale factor size in both row and columns. Set to 1 or less to not downscale.
    '''

    data = pirt1280_raw_to_numpy(raw)

    # convert single pixel value int 3 values for BGR format (BGR values are all the same)
    tmp = np.zeros((data.shape[0], data.shape[1], 3), dtype=data.dtype)
    for i in range(3):
        tmp[:, :, i] = data[:, :]
    data = tmp

    # flip the image because it is read upside down
    data = cv2.flip(data, 0)
    data = cv2.flip(data, 1)

    # manipulate image for displaying
    data //= 64  # scale 14-bits to 8-bits
    data = data.astype(np.uint8)  # imencode wants uint8 or uint64
    data = np.invert(data)  # invert black/white values for displaying

    # downscale image
    if downscale_factor > 1:
        data = np.copy(data[::downscale_factor, ::downscale_factor])

    # color saturate pixel red
    if sat_percent > 0:
        sat_value = (255 * sat_percent) // 100
        sat_pixels = np.where(data[:, :] >= [sat_value, sat_value, sat_value])
        data[sat_pixels[0], sat_pixels[1]] = [0, 0, 255]  # red

    ok, encoded = cv2.imencode(ext, data)
    if not ok:
        raise ValueError(f'{ext} encode error')

    return bytes(encoded)
