#!/usr/bin/env python
# -*- coding: utf-8 -*-
import zmq
from zmq.eventloop import ioloop as ioloop_mod
import zmqdecorators

import time
import re
import yaml
import glob

if os.name == 'posix' and sys.version_info[0] < 3:
    import subprocess32 as subprocess
else:
    import subprocess
import multiprocessing

SERVICE_NAME = "urpobot.launcher"
SERVICE_PORT = 7579
SIGNALS_PORT = 7580

class my_launcher(zmqdecorators.service):
    subprocesses = []

    def __init__(self, service_name, service_port):
        super(my_launcher, self).__init__(service_name, service_port)

    def test_port(self, serial_device):
        """Tests a given device for a board and if found will spin off a service object for it"""
        try:
            port = serial.Serial(serial_device, self.config['speed'], xonxoff=False, timeout=0.01)
            # PONDER: are these the right way around...
            port.setDTR(False) # Reset the arduino by driving DTR for a moment (RS323 signals are active-low)
            time.sleep(0.050)
            port.setDTR(True)
            in_buffer = ""
            started = time.time()
            while True:
                data = port.read(1)
                in_buffer += data
                match = self.board_ident_regex.search(in_buffer)
                if not match:
                    # Timeout, abort
                    if ((time.time() - started) > self.board_ident_timeout):
                        print "Could not find board in %s in %d seconds" % (serial_device, self.board_ident_timeout)
                        print "buffer: %s" % repr(in_buffer)
                        port.close()
                        return False
                    # Otherwise go back to reading data
                    continue
                # Got a match, continue by setting up a new service object
                port.close() # Free the port
                device_name = match.group(1)
                if (self.start_board(serial_device, device_name)):
                    print "Found board %s in %f seconds" % (device_name, time.time() - started)
                    return True
                #otherwise init failed
                return False
        except serial.SerialException, e:
            # Problem with port
            print "Got an exception from port %s: %s" % (serial_device, repr(e))
            return False
        # Something weird happened, we should not drop this far
        print False

    @zmqdecorators.method()
    def start_board(self, serial_device, device_name):
        """Spins up an interface object for given board"""
        if device_name == 'motorctrl':
            self.subprocesses.append(multiprocessing.Process(target=subprocess.call, args=('python motorctrl.py %s' % serial_device,), kwargs={'shell': True}))
            self.subprocesses[-1].start()
            return True
        if device_name == 'MinIMU9AHRS_XV':
            self.subprocesses.append(multiprocessing.Process(target=subprocess.call, args=('python navidata.py %s' % serial_device,), kwargs={'shell': True}))
            self.subprocesses[-1].start()
            return True
        print "Board %s is not known" % device_name
        return False

    @zmqdecorators.method()
    def scan(self):
        """Scans the configured serial devices for boards"""
        for filespec in self.config['search_ports']:
            for comport in glob.glob(filespec):
                self.test_port(comport)

    def cleanup(self):
        print("Cleanup called")
        for p in self.subprocesses
            p.join()
        self.subprocesses = []



if __name__ == "__main__":
    import serial
    import sys,os
    instance = myserver(SERVICE_NAME, SERVICE_PORT)
    print("Starting")
    instance.run()
