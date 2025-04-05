#!/usr/bin/env python3

import shutil
from argparse import ArgumentParser

from oresat_configs import gen_canopend_files, gen_dbc_node

OD_CONFIG_PATH = "od.yaml"
GEN_DIR_PATH = "oresat_cfc/gen"

parser = ArgumentParser()
parser.add_argument("gen", nargs="?", choices=["code", "dbc", "clean"], default="code")
args = parser.parse_args()

if args.gen == "code":
    gen_canopend_files(OD_CONFIG_PATH, GEN_DIR_PATH)
elif args.gen == "dbc":
    gen_dbc_node(OD_CONFIG_PATH)
elif args.gen == "clean":
    shutil.rmtree(GEN_DIR_PATH, ignore_errors=True)
