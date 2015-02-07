#!/usr/bin/env python
# -*- coding: utf-8 -*-
import zmq
from zmq.eventloop import ioloop as ioloop_mod
import zmqdecorators

SERVICE_NAME = "urpobot.pixyblocks"
SERVICE_PORT = 7579
SIGNALS_PORT = 7580

from pixy import *
from ctypes import *

# I have no idea what this is for, it's from the pixy get_blocks.py example
class Blocks (Structure):
  _fields_ = [ ("type", c_uint),
               ("signature", c_uint),
               ("x", c_uint),
               ("y", c_uint),
               ("width", c_uint),
               ("height", c_uint),
               ("angle", c_uint) ]

BLOCKS = Block()

class myserver(zmqdecorators.service):
    def __init__(self, service_name, service_port):
        super(myserver, self).__init__(service_name, service_port)


        self.pcb = ioloop_mod.PeriodicCallback(self.check_pixy_blocks_are_new, 20)
        self.pcb.start()

    def check_pixy_blocks_are_new(self):
        #if pixy_blocks_are_new():
        #    self.get_and_report_blocks()
        self.get_and_report_blocks()

    def get_and_report_blocks(self):
        count = pixy_get_blocks(1, BLOCKS)
        if count < 1:
            return
        if BLOCKS.type == 0: #TYPE_NORMAL:
            self.block("%d" % BLOCKS.signature,
                "%3d" % BLOCKS.x,
                "%3d" % BLOCKS.y,
                "%3d" % BLOCKS.width,
                "%3d" % BLOCKS.height
                )
            pass
        if BLOCKS.type == 1: #TYPE_COLOR_CODE:
            self.ccblock("%d" % BLOCKS.signature,
                "%3d" % BLOCKS.x,
                "%3d" % BLOCKS.y,
                "%3d" % BLOCKS.width,
                "%3d" % BLOCKS.height,
                "%3d" % BLOCKS.angle
                )
            pass

    @zmqdecorators.signal(SERVICE_NAME, SIGNALS_PORT)
    def block(self, sig, x, y, w, h):
        print("DEBUG: reported normal block: sig=%s, x=%s, y=%s, w=%s h=%s" % (sig, x, y, w, h))
        pass

    @zmqdecorators.signal(SERVICE_NAME, SIGNALS_PORT)
    def ccblock(self, sig, x, y, w, h, angle):
        print("DEBUG: reported CC block: sig=%s, x=%s, y=%s, w=%s h=%s, ang=%s" % (sig, x, y, w, h, angle))
        pass

    def run(self):
        # Initialize Pixy Interpreter thread #
        pixy_init()
        # And start the eventloop
        return super(myserver, self).run()


if __name__ == "__main__":
    import sys,os
    instance = myserver(SERVICE_NAME, SERVICE_PORT)
    print("Starting")
    instance.run()
