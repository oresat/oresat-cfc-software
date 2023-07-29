import os

from olaf import app, rest_api, olaf_setup, olaf_run, render_olaf_template

from .drivers.pirt1280 import Pirt1280
from .drivers.rc625 import Rc625
from .services.camera import CameraService
from .services.tec import TecService


@rest_api.app.route('/cfc')
def camera_template():
    return render_olaf_template('cfc.html', name='CFC')


def main():
    path = os.path.dirname(os.path.abspath(__file__))

    args = olaf_setup(f'{path}/data/oresat_cfc.dcf')
    mock_args = [i.lower() for i in args.mock_hw]
    mock_camera = 'camera' in mock_args or 'all' in mock_args
    mock_tec = 'tec' in mock_args or 'all' in mock_args

    # get configs form OD
    camera_spi_bus = app._od['Camera']['SPI bus'].value
    camera_spi_device = app._od['Camera']['SPI device'].value
    camera_enable_pin = app._od['Camera']['Enable gpio pin'].value
    camera_adc_num = app._od['Camera']['ADC pin'].value
    tec_enable_pin = app._od['TEC']['Enable gpio pin'].value

    pirt1280 = Pirt1280(camera_spi_bus, camera_spi_device, camera_enable_pin, camera_adc_num,
                        mock_camera)
    rc6_25 = Rc625(tec_enable_pin, mock_tec)

    app.add_service(CameraService(pirt1280))
    app.add_service(TecService(pirt1280, rc6_25))

    rest_api.add_template(f'{path}/templates/cfc.html')

    olaf_run()


if __name__ == '__main__':
    main()
