"""CFC OLAF app main"""

import os

from olaf import app, olaf_run, olaf_setup, render_olaf_template, rest_api

from . import __version__
from .drivers.pirt1280 import Pirt1280
from .drivers.rc625 import Rc625
from .services.camera import CameraService
from .services.tec_controller import TecControllerService


@rest_api.app.route("/cfc")
def camera_template():
    """Render the cfc template."""
    return render_olaf_template("cfc.html", name="CFC (Cirrus Flux Camera)")


@rest_api.app.route("/pid-graph")
def tec_template():
    """Render the tec template."""
    return render_olaf_template("pid.html", name="TEC PID")


def main():
    """Main for cfc olaf app."""

    path = os.path.dirname(os.path.abspath(__file__))

    args, _ = olaf_setup("cfc_processor")
    mock_args = [i.lower() for i in args.mock_hw]
    mock_camera = "camera" in mock_args or "all" in mock_args
    mock_tec = "tec" in mock_args or "all" in mock_args

    camera_spi_bus = 1
    camera_spi_device = 1
    camera_enable_pin = "SENSOR_ENABLE"
    camera_adc_num = 0
    tec_enable_pin = "TEC_ENABLE"

    app.od["versions"]["sw_version"].value = __version__

    pirt1280 = Pirt1280(
        camera_spi_bus, camera_spi_device, camera_enable_pin, camera_adc_num, mock_camera
    )
    rc6_25 = Rc625(tec_enable_pin, mock_tec)

    app.add_service(CameraService(pirt1280))
    app.add_service(TecControllerService(pirt1280, rc6_25))

    rest_api.add_template(f"{path}/templates/cfc.html")
    rest_api.add_template(f"{path}/templates/pid.html")

    olaf_run()


if __name__ == "__main__":
    main()
