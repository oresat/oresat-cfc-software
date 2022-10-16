from enum import IntEnum

from olaf import Resource, logger

from .pirt1280 import PIRT1280


class State(IntEnum):
    OFF = 0x0
    STANDBY = 0x1
    CAPTURE = 0x2
    SATURATED = 0x3
    ERROR = 0x4


class CFCResource(Resource):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.delay = 1
        self._state = State.STANDBY

        self._cam = PIRT1280()
        self._cam.set_integration(0.05)

    def on_start(self):

        logger.info('enabling camera')
        self._cam.enable()

    def on_loop(self):

        self._cam.capture()

    def on_end(self):

        logger.info('disabling camera')
        self._cam.disable()
