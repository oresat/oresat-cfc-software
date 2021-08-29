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
from cfc_tec import ctrl as tec
from pirt1280 import pirt1280

log = logging.getLogger()
log.setLevel(logging.DEBUG)

DBUS_INTERFACE_NAME = "org.OreSat.CFC"

# TODO confirm this
tec_setpoint = -6

# constants
cols = 1280
rows = 1024
pixel_bytes = cols * rows * 2
prucam_path = "/dev/prucam"

capture_dir = "/home/oresat/cfc_captures"

# instantiate the PIRT once here. 
# TODO there is nothing wrong with doing this multiple times elsewhere
pirt = pirt1280()

def run_cfc_test_forever():
    try:
        # enable the image sensor
        pirt.enable()

        # wait a sec
        time.sleep(1)
        
        # set the TEC temperature
        tec.start(tec_setpoint)

        while True:
            log.info("capturing...")
            
            get_frame_with_integration(0.01)

            time.sleep(1)
    except Exception as e:
        log.error("error running cfc:", e)
        time.sleep(10)
        sys.exit(1)

def get_frame_with_integration(intr):
    # set the integration for this frame
    pirt.set_integration(intr)
    
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

    # get the current temperature 
    temp = tec.get_temp()

    # replace decimal point with 'p' to not mess up extension
    temp_str = str(temp).replace(".","p")
    
    # make the filename
    filename = "capt_" + str(intr) + "_" + temp_str + "C_" + str(int(time.time())) + ".gz"
   
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
        return tec.saturated

    @property
    def TecEnabled(self) -> bool:
        log.debug("TecEnabled")
        return tec.enabled

    @property
    def TecTemp(self) -> float:
        log.debug("TecTemp")
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

