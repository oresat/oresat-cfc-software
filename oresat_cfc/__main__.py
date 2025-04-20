import logging
from argparse import ArgumentParser

from oresat_canopend import NodeClient

from .drivers.pirt1280 import Pirt1280
from .drivers.rc625 import Rc625
from .gen.cfc_od import CfcEntry
from .services.camera import CameraService
from .services.tec_controller import TecControllerService
from .ui import Ui

PIRT1280_SPI = (1, 1)  # bus, device
PIRT1280_ENABLE_GPIO = (2, 22)  # chip, line
PIRT1280_ADC_PIN = 0
TEC_ENABLE_GPIO = (2, 24)  # chip, line


def main():
    parser = ArgumentParser()
    parser.add_argument("-m", "--mock-hw", action="store_true", help="mock hardware")
    parser.add_argument("-v", "--verbose", action="store_true", help="verbose logging")
    args = parser.parse_args()

    LOG_FMT = "%(levelname)s: %(filename)s:%(lineno)s - %(message)s"
    logging.basicConfig(format=LOG_FMT)
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    else:
        logging.getLogger().setLevel(logging.INFO)

    node = NodeClient(CfcEntry)

    pirt1280 = Pirt1280(PIRT1280_SPI, PIRT1280_ENABLE_GPIO, PIRT1280_ADC_PIN, args.mock_hw)
    rc625 = Rc625(TEC_ENABLE_GPIO, args.mock_hw)

    camera_service = CameraService(node, pirt1280)
    tec_service = TecControllerService(node, pirt1280, rc625)
    ui = Ui(node, camera_service)

    tec_service.run(thread=True)
    camera_service.run(thread=True)
    try:
        ui.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
