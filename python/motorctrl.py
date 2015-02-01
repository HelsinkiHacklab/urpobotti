#!/usr/bin/env python
# -*- coding: utf-8 -*-
import zmq
import zmqdecorators

SERVICE_NAME = "urpobot.motor"
SERVICE_PORT = 7575
SIGNALS_PORT = 7576


class myserver(zmqdecorators.service):
    def __init__(self, service_name, service_port, serialport):
        super(myserver, self).__init__(service_name, service_port)
        self.port = serialport
        # TODO: start serial reader thread and give out the pulses as signals

    def cleanup(self):
        print("Cleanup called")

    @zmqdecorators.signal(SERVICE_NAME, SIGNALS_PORT)
    def testsignal(self):
        print("Sending testsignal")
        pass

    @zmqdecorators.method()
    def setspeeds(self, resp, m1speed, m2speed):
        self.port.write("SPDS:%d,%d\n" % (int(m1speed), int(m2speed)))
        # TODO: actually handle ACK/NACK
        resp.send("ACK")

    @zmqdecorators.method()
    def food(self, resp, arg, arg2):
        print "Sending noms as reply"
        # Remember, ZMQ only deals in strings, so typecast everything (JSON is a good idea too)
        resp.send("Here's %s for the noms (for %s ppl)", str(arg), str(arg2))


if __name__ == "__main__":
    import serial
    import sys,os
    port = serial.Serial(sys.argv[1], 115200, xonxoff=False, timeout=0.01)
    instance = myserver(SERVICE_NAME, SERVICE_PORT, port)
    print("Starting")
    instance.run()
