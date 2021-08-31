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

pru0_path = "/sys/class/remoteproc/remoteproc1/"
pru1_path = "/sys/class/remoteproc/remoteproc2/"
pru0_fw = "prucam_pru0_fw.out"
pru1_fw = "prucam_pru1_fw.out"

def init_prus():
    # stop the PRUs. This can throw as error if they are already stopped, so ignore
    try:
        with open(pru0_path + "state", 'w') as f:
            f.write("stop")
    except Exception as e:
        log.warning(e)
    try:
        with open(pru1_path + "state", 'w') as f:
            f.write("stop")
    except Exception as e:
        log.warning(e)

    # first, write the firmware name
    with open(pru0_path + "firmware", 'w') as f:
        f.write(pru0_fw)
    with open(pru1_path + "firmware", 'w') as f:
        f.write(pru1_fw)
    
    # finally, start the PRUs
    with open(pru0_path + "state", 'w') as f:
        f.write("start")
    with open(pru1_path + "state", 'w') as f:
        f.write("start")


def run_cfc_test_forever():
    try:
        # initialize the PRUs
        init_prus()

        # enable the image sensor
        pirt.enable()

        # wait a sec
        time.sleep(1)
        
        # set the TEC temperature
        tec.start(tec_setpoint)

        while True:
            log.info("capturing...")
            
            get_frame_with_integration(0.01)

            time.sleep(180)
    except Exception as e:
        log.error("error running cfc: " + str(e))
        time.sleep(10)
        # use os._exit because sys.exit does not work from a thread
        # TODO OMG UNCOMMENT THIS FOR FLIGHT, leave like this only for testing!
        #os._exit(1)

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
                log.error("error enabling TEC: " + str(e))
        else:
            log.info("disabling TEC")
            try:
                tec.stop()
            except Exception as e:
                log.error("error disabling TEC: " + str(e))

