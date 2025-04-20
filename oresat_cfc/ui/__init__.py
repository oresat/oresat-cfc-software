import base64
import os
from threading import Event

import cv2
import numpy as np
from oresat_canopend import NodeClient

from ..__init__ import __version__
from ..drivers.pirt1280 import Pirt1280, pirt1280_raw_to_numpy
from ..gen.cfc_od import CfcEntry
from ..services.camera import CameraService

try:
    from bottle import TEMPLATE_PATH

    DIR_PATH = os.path.dirname(os.path.abspath(__file__))
    TEMPLATE_PATH.append(DIR_PATH)
except ImportError:
    pass


class Ui:
    def __init__(self, node: NodeClient, camera: CameraService):
        self.node = node
        self.camera = camera
        try:
            from bottle import Bottle

            self.app = Bottle()

            self.app.route("/", "GET", self.get_index)
            self.app.route("/image", "GET", self.get_image)
            self.app.route("/image/raw", "GET", self.get_image_raw)
            self.app.route("/data", "GET", self.get_data)
            self.app.route("/data", "PUT", self.put_data)
        except ImportError:
            self.app = None

    def run(self):
        if self.app:
            self.app.run(port=8000, quiet=True)
        else:
            while True:
                Event().wait()

    def get_index(self):
        from bottle import template

        return template("./index.tpl", version=__version__)

    def get_image(self) -> dict:
        if self.camera.last_capture:
            raw = self.camera.last_capture
            img = make_display_image(raw, sat_percent=95, downscale_factor=2)
        else:
            raw = b"\x00" * Pirt1280.PIXEL_BYTES
            img = make_display_image(raw, sat_percent=0, downscale_factor=2)
        return {"image": base64.encodebytes(img).decode("utf-8")}

    def get_image_raw(self) -> dict:
        raw = self.camera.last_capture
        return {"image": base64.encodebytes(raw).decode("utf-8")}

    def get_data(self) -> dict:
        return {
            "camera": {
                "status": self.node.od_read(CfcEntry.CAMERA_STATUS, use_enum=False),
                "capture_delay": self.node.od_read(CfcEntry.CAMERA_CAPTURE_DELAY),
                "number_to_capture": self.node.od_read(CfcEntry.CAMERA_NUMBER_TO_CAPTURE),
                "save_captures": self.node.od_read(CfcEntry.CAMERA_SAVE_CAPTURES),
                "integration_time": self.node.od_read(CfcEntry.CAMERA_INTEGRATION_TIME),
                "temperature": self.node.od_read(CfcEntry.CAMERA_TEMPERATURE),
                "last_capture_time": self.camera.last_capture_time,
            },
            "tec": {
                "status": self.node.od_read(CfcEntry.TEC_STATUS, use_enum=False),
                "saturated": self.node.od_read(CfcEntry.TEC_SATURATED),
                "setpoint": self.node.od_read(CfcEntry.TEC_PID_SETPOINT),
            },
        }

    def put_data(self):
        from bottle import request

        if "camera" in request.json:
            camera_data = request.json["camera"]
            if "capture_delay" in camera_data:
                self.node.od_write(CfcEntry.CAMERA_CAPTURE_DELAY, camera_data["capture_delay"])
            if "number_to_capture" in camera_data:
                self.node.od_write(
                    CfcEntry.CAMERA_NUMBER_TO_CAPTURE, camera_data["number_to_capture"]
                )
            if "save_captures" in camera_data:
                self.node.od_write(CfcEntry.CAMERA_SAVE_CAPTURES, camera_data["save_captures"])
            if "integration_time" in camera_data:
                self.node.od_write(
                    CfcEntry.CAMERA_INTEGRATION_TIME, camera_data["integration_time"]
                )
        if "tec" in request.json:
            tec_data = request.json["tec"]
            if "status" in tec_data:
                self.node.od_write(CfcEntry.TEC_STATUS, tec_data["status"])
            if "setpoint" in tec_data:
                self.node.od_write(CfcEntry.TEC_PID_SETPOINT, tec_data["setpoint"])

        # do this last
        if "camera" in request.json and "status" in request.json["camera"]:
            self.camera._set_state(request.json["camera"]["status"])


def make_display_image(raw: bytes, sat_percent: int = 0, downscale_factor: int = 1) -> bytes:
    data = pirt1280_raw_to_numpy(raw)

    # convert single pixel value int 3 values for BGR format (BGR values are all the same)
    tmp = np.zeros((data.shape[0], data.shape[1], 3), dtype=data.dtype)
    for i in range(3):
        tmp[:, :, i] = data[:, :]
    data = tmp

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

    ok, encoded = cv2.imencode(".jpg", data)  # pylint: disable=E1101
    if not ok:
        raise ValueError("jpg encode error")

    return bytes(encoded)
