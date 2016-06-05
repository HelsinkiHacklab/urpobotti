#!/usr/bin/env python
# -*- coding: utf-8 -*-
import zmq
from zmq.eventloop import ioloop as ioloop_mod
import zmqdecorators
import re
import time
import json

SERVICE_NAME = "urpobot.pings"
SERVICE_PORT = 7577
SIGNALS_PORT = 7578

lidarinfo = re.compile('(\d+): (\d+) \((\d+)\)')

AHRS_BUFFER_SIZE = 5
ANGLE_ADJUST = 180 # Add and modulo to adjust the reported angle so sensor orientation matches robot

class navidataserver(zmqdecorators.service):
    def __init__(self, service_name, service_port, serialport):
        super(navidataserver, self).__init__(service_name, service_port)
        self.serial_port = serialport
        self.input_buffer = ""
        self.socket.setsockopt(zmq.SNDHWM, 10)
        self.AHRS_BUFFER = [ (0,0,0) for x in range(AHRS_BUFFER_SIZE) ]
        self.AHRS_BUFFER_I = 0
        self.LIDAR_BUFFER = [ (0,0) for x in range(360) ]
        

    def run(self):
        print("Starting navidataserver")
        time.sleep(0.1)
        # Setup for ASCII output
        self.serial_port.write("\r\n")
        self.serial_port.write("HideRaw\r\n")
        self.serial_port.write("ShowDist\r\n")
        # Start motor
        time.sleep(0.1)
        self.serial_port.write("MotorOn\r\n")
        self.evthandler = ioloop_mod.IOLoop.instance().add_handler(self.serial_port.fileno(), self.handle_serial_event, ioloop_mod.IOLoop.instance().READ)
        super(navidataserver, self).run()

    def handle_serial_event(self, fd, events):
        bytesToRead = self.serial_port.inWaiting()
        if not bytesToRead:
            # Don't try to read if there is no data, instead sleep (yield) a bit
            time.sleep(0)
            return
        data = self.serial_port.read(bytesToRead)
        if len(data) == 0:
            return
        #print("DEBUG: data=%s" % data)

        # TODO: do this more sanely
        for char in data:
            # Put the data into inpit buffer and check for CRLF
            self.input_buffer += char
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
                angle = int(match.group(1))
                if (   angle < 0 
                    or angle > 359):
                    # Broken data
                    return
                angle = (angle + ANGLE_ADJUST) % 360
                self.LIDAR_BUFFER[angle] = [ int(x) for x in match.group(2,3) ]
                #print("DEBUG: angle=%d" % angle)
                if angle == 359:
                    self.lidar(json.dumps(self.LIDAR_BUFFER))
            if (len(message) > 5 and message[:5] == '!ANG:'):
                self.AHRS_BUFFER[self.AHRS_BUFFER_I] = [ float(x) for x in message[5:].split(',') ]
                if self.AHRS_BUFFER_I == AHRS_BUFFER_SIZE - 1:
                    self.attitude(json.dumps(self.AHRS_BUFFER))
                    self.AHRS_BUFFER_I = 0
                self.AHRS_BUFFER_I += 1
                    

        except Exception as e:
            print "message_received exception: Got exception %s" % repr(e)
            # Ignore indexerrors, they just mean we could not parse the command
            pass
        pass

    @zmqdecorators.signal(SERVICE_NAME, SIGNALS_PORT)
    def lidar(self, jsondata):
        #print("DEBUG: reported lidar: %s" % repr(json.loads(jsondata)))
        pass

    @zmqdecorators.signal(SERVICE_NAME, SIGNALS_PORT)
    def attitude(self, jsondata):
        #print("DEBUG: reported attitude: %s" % repr(json.loads(jsondata)))
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
