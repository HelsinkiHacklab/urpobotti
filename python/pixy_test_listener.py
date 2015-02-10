#!/usr/bin/env python
# -*- coding: utf-8 -*-
import zmq
from zmq.eventloop import ioloop as ioloop_mod
import zmqdecorators

class myclient(zmqdecorators.client):
    def __init__(self):
        super(myclient, self).__init__()
        zmqdecorators.subscribe_topic(("urpobotti.local", 7580), 'CC', self.cc_callback)
        zmqdecorators.subscribe_topic(("urpobotti.local", 7580), 'NORM', self.norm_callback)

    def cc_callback(self, *args):
        print "Got CC: %s" % repr(args)

    def norm_callback(self, *args):
        print "Got NORM: %s" % repr(args)

        
if __name__ == "__main__":
    instance = myclient()
    print("Starting")
    instance.run()

