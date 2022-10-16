from datetime import datetime

from olaf import Resource, logger

from .tec import TEC
from .tec_controller import WATCHDOG_FILE

# if timestamp older than 5 seconds, disable the TEC
TIMEOUT_S = 5


class TECWatchdogResource(Resource):
    '''
    Used to make sure the TEC controller is not hung by checkinmg the timestamp it updates
    frequently
    '''

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._tect = TEC()
        self.delay = TIMEOUT_S

    def on_loop(self):
        '''on an interval, check the timestamp and disable the GPIO on any error'''

        try:
            self._check_timestamp()
        except Exception as e:
            logger.warning(f'error checking TEC timestamp, disabling: {e}')
            self._tec.disable()

    def _check_timestamp():

        with open(WATCHDOG_FILE, 'r') as f:
            ts_str = f.read()
            ts = int(ts_str)

            logger.debug(f'last TEC controller timestamp: {ts}')
            last = datetime.utcfromtimestamp(ts)

            if (datetime.now() - last).total_seconds() > TIMEOUT_S:
                raise Exception('TEC timestamp is stale!')
