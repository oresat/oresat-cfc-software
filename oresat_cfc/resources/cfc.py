from enum import IntEnum

from olaf import Resource, logger, TimerLoop

from ..drivers.pirt1280 import PIRT1280


class State(IntEnum):
    OFF = 0x0
    STANDBY = 0x1
    CAPTURE = 0x2
    SATURATED = 0x3
    ERROR = 0x4


class CFCResource(Resource):

    def __init__(self, camera: PIRT1280):
        super().__init__()

        self._state = State.STANDBY

        self._cam = camera
        self._cam.set_integration(0.05)

        self.timer_loop = TimerLoop('cfc resource', self._loop, 1000)

    def on_start(self):

        logger.info('enabling camera')
        self._cam.enable()

    def on_end(self):

        logger.info('disabling camera')
        self._cam.disable()

    def _loop(self) -> bool:

        if self._state == State.CAPTURE:
            self._cam.capture()  # TODO save data

        return True
