from enum import IntEnum
from time import time

import cv2
import tifffile
import numpy as np
from olaf import Service, logger, new_oresat_file

from .. import __version__
from ..drivers.pirt1280 import Pirt1280, pirt1280_raw_to_numpy


class CfcState(IntEnum):
    OFF = 0x1
    STANDBY = 0x2
    CAPTURE = 0x3


STATE_TRANSMISSIONS = {
    CfcState.OFF: [CfcState.OFF, CfcState.STANDBY],
    CfcState.STANDBY: [CfcState.OFF, CfcState.STANDBY, CfcState.CAPTURE],
    CfcState.CAPTURE: [CfcState.OFF, CfcState.STANDBY],
}


class Pirt1280Service(Service):
    '''Service for camera and capture state machine'''

    def __init__(self, pirt1280: Pirt1280):
        super().__init__()

        self._state = CfcState.OFF

        self._pirt1280 = pirt1280

        self.state_index = 0x6000
        self.camera_index = 0x6001
        self.test_camera_index = 0x7000

    def on_start(self):

        self.capture_delay_obj = self.node.od[self.state_index][2]
        self.capture_count_obj = self.node.od[self.state_index][3]

        self.node.add_sdo_read_callback(self.state_index, self.on_state_read)
        self.node.add_sdo_write_callback(self.state_index, self.on_state_write)

        self.node.add_sdo_read_callback(self.camera_index, self.on_camera_read)
        self.node.add_sdo_write_callback(self.camera_index, self.on_camera_write)

        self.node.add_sdo_read_callback(self.test_camera_index, self.on_test_camera_read)
        self.node.add_sdo_write_callback(self.test_camera_index, self.on_test_camera_write)

    def on_end(self):

        self._pirt1280.disable()

    def state_machine_loop(self):

        try:
            self._state_machine_loop
        except Exception as e:
            logger.error(e)
            self.tec_disable()
            self._pirt1280.disable()
            self.state = CfcState.OFF

    def _state_machine_transittion(self, new_state: CfcState or int):

        if isinstance(new_state, int):
            new_state = CfcState(new_state)

        if new_state not in STATE_TRANSMISSIONS[self._state]:
            return

        if new_state == CfcState.OFF:
            self.tec_disable()
            self.camera_disable()
        elif new_state in list(CfcState):
            self.camera_enable()
            self.count = 0

        self.state = new_state

    def _state_machine_loop(self):

        self.count = 0

        while not self._event.is_set():
            if self._state == CfcState.OFF:
                self._event.wait(1)
            elif self._state == CfcState.STANDBY:
                self._event.wait(1)
            elif self._state == CfcState.CAPTURE:
                self._count += 1
                self._capture()
                if self._capture_count_obj.value != 0 and \
                        self._count > self._capture_count_obj.value:
                    self._state_machine_transittion(CfcState.STANDBY)
                else:
                    self._event.wait(self.capture_delay_obj.value)
            else:
                self._state = CfcState.OFF

    def _capture(self, count: int = 1):
        '''Capture x raw images in a row with no delay and save them to fread cache'''

        dt = time()
        raw = self._pirt1280.capture()
        metadata = {
            'sw_version': __version__,
            'time': dt,
            'temperature': self._pirt1280.temperature,
            'integration_time': self._pirt1280.integration_time,
        }

        file_name = '/tmp/' + new_oresat_file('capture', date=dt, ext='.tiff')
        data = pirt1280_raw_to_numpy(raw)

        tifffile.imwrite(
            file_name,
            data,
            dtype=data.dtype,
            metadata=metadata,
            photometric='miniswhite',
            compression='zstd',
            compressionargs={'level': 1}  # images do not compress well
        )

        self.fread_cache.add(file_name, consume=True)

    def on_state_read(self, index: int, subindex: int):
        '''SDO read on the state machine obj'''

        if index == self.state_index and subindex == 0x1:
            return self._state.value

    def on_state_write(self, index: int, subindex: int, value):
        '''SDO read on the state machine obj'''

        if index == self.camera_index and subindex == 0x1:
            self._state_machine_transittion(value)

    def on_camera_read(self, index: int, subindex: int):
        '''SDO read on the camera obj'''

        if index != self.camera_index:
            return

        ret = None

        if subindex == 0x4:
            ret = self._pirt1280.is_enabled
        elif subindex == 0x5:
            ret = int(self._pirt1280.temperature)
        elif subindex == 0x7:
            ret = self._pirt1280.integration_time

        return ret

    def on_camera_write(self, index: int, subindex: int, value):
        '''SDO read on the camera obj'''

        if index != self.camera_index:
            return

        if subindex == 0x4:
            if value:
                self._pirt1280.enable()
            else:
                self._pirt1280.disable()
        elif subindex == 0x7:
            self._pirt1280.integration_time = value

    def on_test_camera_read(self, index: int, subindex: int):
        '''SDO read on the test camera obj'''

        if index != self.test_camera_index:
            return

        ret = None

        if subindex == 0x1:
            ret = self._last_capture
        elif subindex == 0x2 and self._state != CfcState.OFF:
            self._last_capture = self._pirt1280.capture()
            ret = make_display_image(self._last_capture, downscale_factor=2)
        elif subindex == 0x3:
            ret = self._pirt1280.is_enabled

        return ret

    def on_test_camera_write(self, index: int, subindex: int, value):
        '''SDO write on the test camera obj'''

        if index != self.test_camera_index and subindex != 0x3:
            return

        if value:
            self.camera.enable()
        else:
            self.camera.disable()


def make_display_image(raw: bytes, ext: str = '.jpg', sat_percent: int = 0,
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
