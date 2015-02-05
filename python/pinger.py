#!/usr/bin/env python
# -*- coding: utf-8 -*-
import zmq
from zmq.eventloop import ioloop as ioloop_mod
import zmqdecorators

SERVICE_NAME = "urpobot.pings"
SERVICE_PORT = 7577
SIGNALS_PORT = 7578


class myserver(zmqdecorators.service):
    def __init__(self, service_name, service_port, serialport):
        super(myserver, self).__init__(service_name, service_port)
        self.serial_port = serialport
        self.input_buffer = ""
        # TODO: start serial reader thread and give out the pulses as signals

        self.evthandler = ioloop_mod.IOLoop.instance().add_handler(self.serial_port.fileno(), self.handle_serial_event, ioloop_mod.IOLoop.instance().READ)

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
        #print("DEBUG: msg=%s" % message)
        try:
            #!PPS:0,0
            if (message[:6] == '!PING:'):
                (sensor, distance) = message[6:].split(',')
                self.pingreport(sensor, distance)

        except Exception,e:
            print "message_received: Got exception %s" % e
            # Ignore indexerrors, they just mean we could not parse the command
            pass
        pass

    @zmqdecorators.signal(SERVICE_NAME, SIGNALS_PORT)
    def pingreport(self, sensor, distance):
        print("DEBUG: reported ping: %s,%s" % (sensor, distance))

    def cleanup(self):
        print("Cleanup called")

    @zmqdecorators.method()
    def setspeeds(self, resp, m1speed, m2speed):
        self.port.write("SPDS:%d,%d\n" % (int(m1speed), int(m2speed)))
        # TODO: actually handle ACK/NACK somehow (we need to read it from the serialport but we can't block while waiting for it...)
        resp.send("ACK")




if __name__ == "__main__":
    import serial
    import sys,os
    port = serial.Serial(sys.argv[1], 115200, xonxoff=False, timeout=0.01)
    instance = myserver(SERVICE_NAME, SERVICE_PORT, port)
    print("Starting")
    instance.run()
