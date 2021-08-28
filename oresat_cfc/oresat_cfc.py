#!/usr/bin/python3
"""Main for the OreSat CFC daemon."""

import time
import gzip
import numpy as np
import sys
from pydbus import SystemBus
import os
import io
import logging
from gi.repository import GLib
import threading
import cv2

log = logging.getLogger()
log.setLevel(logging.DEBUG)

# TODO remove this when on octavo
try:
    from cfc_tec import ctrl as tec
except:
    pass
from pirt1280 import pirt1280

DBUS_INTERFACE_NAME = "org.OreSat.CFC"

# TODO confirm this
tec_setpoint = -6

# constants
cols = 1280
rows = 1024
pixel_bytes = cols * rows * 2
prucam_path = "/dev/prucam"

capture_dir = "/home/rew/cfc_captures"

# instantiate the PIRT once here. 
# TODO there is nothing wrong with doing this multiple times elsewhere
# TODO remove 'try:' when on octavo
try:
    pirt = pirt1280()
except:
    pass

def run_cfc_test_forever():
    try: # TODO remove on octavo
        pirt.enable()
    except:
        pass

    # wait a sec
    time.sleep(1)
    
    try: # TODO remove on octavo
        tec.start(tec_setpoint)
    except:
        pass

    while True:
        log.info("capturing...")
        
        get_frame_with_integration(0.01)

        time.sleep(1)

def get_frame_with_integration(intr):
    try: # TODO remove on octavo
        pirt.set_integration(intr)
    except:
        pass
    
    # wait a moment for settings to apply
    time.sleep(0.05)
    
    # open the prucam char device
    fd = os.open(prucam_path, os.O_RDWR)
    fio = io.FileIO(fd, closefd = False)

    # allocate buffer to read frame into
    imgbuf = bytearray(pixel_bytes)

    # read from prucam into buffer
    fio.readinto(imgbuf)

    # close the char device
    os.close(fd)

    # TODO make filename with time, integration time, and ....?
    
    temp = -12.1
    try:
        temp = tec.get_temp()
    except:
        pass

    # replace decimal point with 'p' to not mess up extension
    temp_str = str(temp).replace(".","p")
    
    filename = "capt_" + str(intr) + "_" + temp_str + "C_" + str(int(time.time())) + ".gz"
   
    # TODO test image
    # TODO compress image?

    # write raw buffer out to file gzip'd
    with gzip.open(capture_dir + "/" + filename, 'wb') as out:
        out.write(imgbuf)


def main():
    """The main for the oresat CFC daemon"""

    try:
        # run test in separate thread
        threading.Thread(target=run_cfc_test_forever).start()

        # set up dbus wrapper
        cfc = DBusServer()
        bus = SystemBus()
        bus.publish(DBUS_INTERFACE_NAME, cfc)

        loop = GLib.MainLoop()

        try:
            loop.run()
        except KeyboardInterrupt:
            loop.quit()
        except Exception as e:
            loop.quit()

    except Exception as e:
        print("exception:", e)

        return 0
    return 1

# TODO
# - endpoint for diskspace
# - errors endpoint


class DBusServer():

    # D-Bus interface definition
    dbus = """
    <node>
        <interface name="org.OreSat.CFC">
            <method name='EnableTec'>
                <arg type='b' name='enable' direction='in'/>
            </method>
            <property name="TecSaturated" type="b" access="read" />
            <property name="TecEnabled" type="b" access="read" />
            <property name="TecTemp" type="d" access="read" />
            <property name="CaptureCount" type="u" access="read" />
        </interface>
    </node>
    """
   
    # need to test these
    @property
    def TecSaturated(self) -> bool:
        log.debug("TecSaturated")
        return True
        return tec.saturated

    @property
    def TecEnabled(self) -> bool:
        log.debug("TecEnabled")
        return True
        return tec.enabled

    @property
    def TecTemp(self) -> float:
        log.debug("TecTemp")
        return 15.0
        return tec.get_temp()

    @property
    def CaptureCount(self) -> int:
        log.debug("CaptureCount")
        
        # return the number of files in the capture directory
        files = os.listdir(capture_dir)
        return len(files)

    def EnableTec(self, enable):
        log.debug("Enable")
        """D-Bus Method to enable/disable TEC"""

        if enable:
            log.info("enabling TEC")
            try:
                tec.start(tec_setpoint)
            except Exception as e:
                log.error("error enabling TEC: ", e)
        else:
            log.info("disabling TEC")
            try:
                tec.stop()
            except Exception as e:
                log.error("error edisabling TEC: ", e)

