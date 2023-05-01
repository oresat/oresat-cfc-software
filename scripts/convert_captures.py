#!/usr/bin/python3

import os
import sys
import gzip

import numpy as np
import cv2

cols = 1280
rows = 1024

capt_dir = sys.argv[1]

# read all the files in the capture directory
capts = os.listdir(capt_dir)

for capt in capts:
    # only try to convert gzip'd files
    if not capt.endswith(".gz"):
        continue

    # create the full file path
    path = capt_dir + "/" + capt
    print(path)

    # open the file
    with gzip.open(path, 'rb') as f:
        print(path)
        # read the decompressed bytes
        buf = f.read()

        if len(buf) == 0:
            continue

        # read image bytes into ndarray
        img = np.frombuffer(buf, dtype=np.uint16).reshape(rows, cols)

        # flip the image because it is read upside down
        img = cv2.flip(img, 0)
        img = cv2.flip(img, 1)

        # invert image colors to look normal
        img = cv2.bitwise_not(img)

        # try to encode the image as png
        ok, encoded = cv2.imencode(".png", img, params=[cv2.CV_16U])
        if not ok:
            raise Exception("error encoding image")

        # write the file as a png
        png_path = path.replace(".gz", ".png")
        # png_path = path.replace(".bin", ".png")
        with open(png_path, 'wb') as png:
            png.write(encoded.tobytes())
