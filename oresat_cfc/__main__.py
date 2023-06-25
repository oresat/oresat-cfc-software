import os

from olaf import app, rest_api, olaf_setup, olaf_run, render_olaf_template

from .drivers.tec import Tec
from .drivers.pirt1280 import Pirt1280
from .cfc_resource import CFCResource


@rest_api.app.route('/cfc')
def camera_template():
    return render_olaf_template('cfc.html', name='CFC')


def main():
    path = os.path.dirname(os.path.abspath(__file__))

    args = olaf_setup(f'{path}/data/oresat_cfc.dcf')
    mock_args = [i.lower() for i in args.mock_hw]
    mock_camera = 'camera' in mock_args or 'all' in mock_args
    mock_tec = 'tec' in mock_args or 'all' in mock_args

    # TODO get these from OD
    camera_enable_pin = 86
    tec_enable_pin = 88

    pirt1280 = Pirt1280(camera_enable_pin, mock_camera)
    tec = Tec(tec_enable_pin, mock_tec)

    app.add_resource(CFCResource(pirt1280, tec))

    rest_api.add_template(f'{path}/templates/cfc.html')

    olaf_run()


if __name__ == '__main__':
    main()
