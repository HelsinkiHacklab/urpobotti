#!/usr/bin/env python
# -*- coding: utf-8 -*-
import zmq
from zmq.eventloop import ioloop as ioloop_mod
import zmqdecorators
import time

SERVICE_NAME = "urpobot.motor"
SERVICE_PORT = 7575
SIGNALS_PORT = 7576

# How long to wait for new commands before stopping automatically
COMMAND_GRACE_TIME = 0.250


class motorserver(zmqdecorators.service):
    def __init__(self, service_name, service_port, serialport):
        super(motorserver, self).__init__(service_name, service_port)
        self.serial_port = serialport
        self.input_buffer = ""
        self.evthandler = ioloop_mod.IOLoop.instance().add_handler(self.serial_port.fileno(), self.handle_serial_event, ioloop_mod.IOLoop.instance().READ)
        self.last_command_time = time.time()
        self.pcb = ioloop_mod.PeriodicCallback(self.check_data_reveived, COMMAND_GRACE_TIME)
        self.pcb.start()

    def check_data_reveived(self, *args):
        if (time.time() - self.last_command_time > COMMAND_GRACE_TIME):
            self._setspeeds(0,0)

    def _setspeeds(self, m1speed, m2speed):
        self.serial_port.write("S%04X%04X\n" % ((m1speed & 0xffff), (m2speed & 0xffff)))

    @zmqdecorators.method()
    def setspeeds(self, resp, m1speed, m2speed):
        self.last_command_time = time.time()
        #print("Got speeds %s,%s" % (m1speed, m2speed))
        self._setspeeds(m1speed, m2speed)
        # TODO: actually handle ACK/NACK somehow (we need to read it from the serialport but we can't block while waiting for it...)
        resp.send("ACK")

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
            # Currently we have no incoming messages from this board
            pass
        except Exception as e:
            print "message_received exception: Got exception %s" % repr(e)
            # Ignore indexerrors, they just mean we could not parse the command
            pass
        pass

    def cleanup(self):
        print("Cleanup called")
        self._setspeeds(0,0)

    def run(self):
        print("Starting motorserver")
        super(motorserver, self).run()



if __name__ == "__main__":
    import serial
    import sys,os
    port = serial.Serial(sys.argv[1], 115200, xonxoff=False, timeout=0.01)
    instance = motorserver(SERVICE_NAME, SERVICE_PORT, port)
    instance.run()
