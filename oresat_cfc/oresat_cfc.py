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
from datetime import datetime
import threading
from cfc_tec import ctrl as tec
from pirt1280 import pirt1280
import atexit
import random
import string
from pathlib import Path

log = logging.getLogger()
log.setLevel(logging.DEBUG)

DBUS_INTERFACE_NAME = "org.OreSat.CFC"

# TODO change this back to -6
#tec_setpoint = -6
tec_setpoint = 5

# constants
cols = 1280
rows = 1024
pixel_bytes = cols * rows * 2
prucam_path = "/dev/prucam"

capture_dir = "/home/oresat/cfc_captures"

intr_times = [0.0025, 0.005, 0.01, 0.02]

# instantiate the PIRT once here. 
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
        # TODO test me
        Path("/home/oresat/cfc_captures").mkdir(parents=True, exist_ok=True)

        # initialize the PRUs
        init_prus()
        # init the pins, sorry this is jank
        os.system("sudo config-pin P1_36 pruin")
        os.system("sudo config-pin P1_33 pruin")
        os.system("sudo config-pin P2_32 pruin")
        os.system("sudo config-pin P2_30 pruin")
        os.system("sudo config-pin P1_31 pruin")
        os.system("sudo config-pin P2_34 pruin")
        os.system("sudo config-pin P2_28 pruin")
        os.system("sudo config-pin P1_20 pruin")
        os.system("sudo config-pin P1_29 pruout")
        os.system("sudo config-pin P2_24 pruout")
        os.system("sudo config-pin P2_33 pruout")

        # enable the image sensor
        pirt.enable()

        # wait a sec
        time.sleep(2)

        # how long for temperature to equalize after the TEC is saturated
        tec_sat_timeout = 5 * 60 # 5 minutes in seconds
        
        # maximum temperature at which we will start the TEC
        tec_max_start_temp = 36 # TODO reduce to 32

        # record the start time
        start_time = datetime.utcnow()
        tec_start_wait = 60 * 10

        while True:
            log.info("capturing...")

            temp = tec.get_temp()

            # handle enabling the TEC

            # holy fuck, somehow we hit 40C, turn the TEC off immediately
            if temp >= 40:
                tec.stop()
                log.warn("TEC reached {}C, disabling!".format(temp))

            # wait 'tec_start_wait' before ever starting the TEC
            # TODO ADD BACK
            elif (datetime.utcnow() - start_time).seconds < tec_start_wait:
                log.info("TEC off for {} more seconds".format(tec_start_wait - (datetime.utcnow() - start_time).seconds))
                tec.stop()

            # If the TEC is not saturated, not enabled, and below the threshold 
            # to start, enable this. This will catch the initial turn on state 
            # because it starts as not enabled or satureated, and hopefully 
            # cold enough to use
            elif temp <= tec_max_start_temp and not tec.enabled and not tec.saturated:
                log.info("starting TEC b/c it is at {}C, and not enabled or saturated".format(temp))
                tec.start(tec_setpoint)

            # if the TEC is/was saturated, but we have waited long enough since 
            # saturation, and it is below the threshold, start it again. This 
            # handles the case where the TEC was saturated and disabled, and we 
            # waited a while for it to cool off
            elif tec.saturated and ((datetime.utcnow() - tec.saturation_time).seconds > tec_sat_timeout) and temp <= tec_max_start_temp:
                log.info("restarting TEC b/c is is at {}C, and was previously saturated but has cooled".format(temp))
                tec.start(tec_setpoint)
                    
            # capture frames at all integration times
            for intr in intr_times:
                get_frame_with_integration(intr)
            
            # wait a while before trying again
            time.sleep(15)

    except Exception as e:
        log.error("error running cfc: " + str(e))
        time.sleep(10)
        # use os._exit because sys.exit does not work from a thread
        os._exit(1)


def randomword(length):
    letters = string.ascii_lowercase
    return ''.join(random.choice(letters) for i in range(length))

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

    # convert the temperature to mC so we dont need a decimal point
    temp_hundreths_of_degree = int(temp * 100)

    # convert the integration time to uS so we dont need a decimal point
    intr_us = int(intr * 1000000)
    
    # make the filename
    filename = str(int(time.time())) + "_" + str(intr_us) + "_" + str(temp_hundreths_of_degree) + "mC" + "_" + randomword(8) + ".gz"
   
    # write raw buffer out to file gzip'd
    with gzip.open(capture_dir + "/" + filename, 'wb', compresslevel=3) as out:
        out.write(imgbuf)

def main():
    """The main for the oresat CFC daemon"""

    try:
        atexit.register(tec.stop)

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

