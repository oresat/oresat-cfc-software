from datetime import datetime

from olaf import Resource, logger, TimerLoop

from ..drivers.tec import TEC
from ..tec_controller import TecController, WATCHDOG_FILE


class TECResource(Resource):

    TIMEOUT_S = 5
    '''if timestamp older than 5 seconds, disable the TEC'''

    def __init__(self, tec: TEC):
        super().__init__()

        self._tec = tec
        self._tec_ctrl = TecController(self._tec)

        self.tec_timer_loop = TimerLoop('tec', self._tec_loop, 500)
        self.watchdog_timer_loop = TimerLoop('tec watchdog', self._watchdog_loop, 1000)

    def on_start(self):

        self.tec_timer_loop.start()
        logger.info('startng TEC watchdog')
        self.watchdog_timer_loop.start()

    def on_end(self):

        self.tec_timer_loop.stop()
        logger.info('stopping TEC watchdog')
        self.watchdog_timer_loop.stop()

    def _tec_loop(self) -> True:

        self._tec_ctrl.loop()

        return True

    def _watchdog_loop(self) -> True:
        '''on an interval, check the timestamp and disable the GPIO on any error'''

        try:
            with open(WATCHDOG_FILE, 'r') as f:
                ts_str = f.read()
                ts = int(ts_str)

                logger.debug(f'last TEC controller timestamp: {ts}')
                last = datetime.utcfromtimestamp(ts)

                if (datetime.now() - last).total_seconds() > self.TIMEOUT_S:
                    raise ValueError('TEC timestamp is stale!')
        except Exception as e:
            logger.warning(f'error checking TEC timestamp, disabling: {e}')
            self._tec.disable()

        return True
