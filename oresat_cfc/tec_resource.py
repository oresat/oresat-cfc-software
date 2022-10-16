from olaf import Resource, logger

from .tec import TEC
from .tec_controller import TecController


class TECResource(Resource):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.delay = 0.25

        self._tec = TEC()
        self._tec_ctrl = TecController(self._tec)

    def on_start(self):

        logger.info('startng TEC watchdog')
        self._tec_ctrl.enable()

    def on_loop(self):

        self._tec.loop()

    def on_end(self):

        logger.info('stopping TEC watchdog')
        self._tec_ctrl.disable()
