from os.path import dirname, abspath
from argparse import ArgumentParser

from olaf import app_args_parser, parse_app_args, App

from .cfc_resource import CFCResource


def main():
    parser = ArgumentParser(parents=[app_args_parser])
    args = parser.parse_args()
    parse_app_args(args)

    eds = f'{dirname(abspath(__file__))}/data/oresat_cfc.dcf'
    app = App(eds, args.bus, args.node_id, args.mock_hw)

    app.add_resource(CFCResource)

    app.run()


if __name__ == '__main__':
    main()
