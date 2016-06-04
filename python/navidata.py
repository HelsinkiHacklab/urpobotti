#!/usr/bin/env python
# -*- coding: utf-8 -*-
import zmq
from zmq.eventloop import ioloop as ioloop_mod
import zmqdecorators
import re

SERVICE_NAME = "urpobot.pings"
SERVICE_PORT = 7577
SIGNALS_PORT = 7578

lidarinfo = re.compile('(\d+): (\d+) \((\d+)\)')


class navidataserver(zmqdecorators.service):
    def __init__(self, service_name, service_port, serialport):
        super(navidataserver, self).__init__(service_name, service_port)
        self.serial_port = serialport
        self.input_buffer = ""
        self.evthandler = ioloop_mod.IOLoop.instance().add_handler(self.serial_port.fileno(), self.handle_serial_event, ioloop_mod.IOLoop.instance().READ)

    def run(self):
        print("Starting navidataserver")
        # Setup for ASCII output
        self.serial_port.write("\r\n")
        self.serial_port.write("HideRaw\r\n")
        self.serial_port.write("ShowDist\r\n")
        # Start motor
        self.serial_port.write("MotorOn\r\n")
        super(navidataserver, self).run()

    def handle_serial_event(self, fd, events):
        # Copied from arbus that was thread based
        if not self.serial_port.inWaiting():
            # Don't try to read if there is no data, instead sleep (yield) a bit
            time.sleep(0)
            return
        data = self.serial_port.read(1)
        if len(data) == 0:
            return
        #print("DEBUG: data=%s" % data)

        # Put the data into inpit buffer and check for CRLF
        self.input_buffer += data
        # Trim prefix NULLs and linebreaks
        self.input_buffer = self.input_buffer.lstrip(chr(0x0) + "\r\n")
        #print "input_buffer=%s" % repr(self.input_buffer)
        if (    len(self.input_buffer) > 1
            and self.input_buffer[-2:] == "\r\n"):
            # Got a message, parse it (sans the CRLF) and empty the buffer
            self.message_received(self.input_buffer[:-2])
            self.input_buffer = ""

    def message_received(self, message):
        try:
            match = lidarinfo.search(message)
            if match:
                self.lidar(*match.group(1,2,3))
            if (len(message) > 5 and message[:5] == '!ANG:'):
                self.attitude(*message[5:].split(','))

        except Exception as e:
            print "message_received exception: Got exception %s" % repr(e)
            # Ignore indexerrors, they just mean we could not parse the command
            pass
        pass

    @zmqdecorators.signal(SERVICE_NAME, SIGNALS_PORT)
    def lidar(self, angle, distance_mm, quality):
        #print("DEBUG: reported lidar: %s,%s,%s" % (angle, distance_mm, quality))
        pass

    @zmqdecorators.signal(SERVICE_NAME, SIGNALS_PORT)
    def attitude(self, roll, pitch, yaw):
        #print("DEBUG: reported attitude: %s,%s,%s" % (roll, pitch, yaw))
        pass

    def cleanup(self):
        print("Cleanup called")
        # Stop motor
        self.serial_port.write("MotorOff\r\n")



if __name__ == "__main__":
    import serial
    import sys,os
    port = serial.Serial(sys.argv[1], 115200, xonxoff=False, timeout=0.01)
    instance = navidataserver(SERVICE_NAME, SERVICE_PORT, port)
    instance.run()
