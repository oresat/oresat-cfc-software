#!/usr/bin/env python3

import os
from oresat_configs import write_canopend_od, gen_od, OdConfig

GEN_DIR = "oresat_cfc/gen"
os.makedirs(GEN_DIR, exist_ok=True)

init_file = os.path.join(GEN_DIR, "__init__.py")
if not os.path.isfile(init_file):
    open(init_file, "w").close()

od_config = OdConfig.from_yaml("od.yaml")
od = gen_od([od_config])
write_canopend_od("cfc", od, GEN_DIR)
