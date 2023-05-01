import os

from olaf import app, rest_api, olaf_setup, olaf_run, render_olaf_template

from .drivers.tec import TEC
from .drivers.pirt1280 import PIRT1280
from .resources.cfc import CFCResource
from .resources.tec import TECResource


@rest_api.app.route('/cfc')
def camera_template():
    return render_olaf_template('cfc.html', name='CFC')


def main():
    path = os.path.dirname(os.path.abspath(__file__))

    args = olaf_setup(f'{path}/data/oresat_cfc.dcf')
    mock_args = [i.lower() for i in args.mock_hw]
    mock_camera = 'camera' in mock_args or 'all' in mock_args
    mock_tec = 'tec' in mock_args or 'all' in mock_args

    pirt1280 = PIRT1280(mock_camera)
    tec = TEC(mock_tec)

    app.add_resource(CFCResource(pirt1280))
    app.add_resource(TECResource(tec))

    rest_api.add_template(f'{path}/templates/cfc.html')

    olaf_run()


if __name__ == '__main__':
    main()
