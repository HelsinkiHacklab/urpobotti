#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division, print_function

import os
import sys

import zmq
import math


class foo(object):
    def __init__(self, zmq_ctx):
        self.navidata = zmq_ctx.socket(zmq.SUB)
        self.navidata.connect("tcp://sonofurpo.local:7578")
        self.navidata.setsockopt_string(zmq.SUBSCRIBE, u'')

        # Get them pollers
#        self.poller = zmq.Poller()
#        self.poller.register(self.navidata, zmq.POLLIN)

    def run(self):
        try:
            while(True):
#                socks = dict(self.poller.poll(1))
#                if self.navidata in socks and socks[self.navidata] == zmq.POLLIN:
#                    msg = self.navidata.recv_multipart()
#

                msg = self.navidata.recv_multipart()

                if msg[0] == 'lidar':
                    print("Got lidar data %s" % repr(msg[1:]))
                    pass

                if msg[0] == 'attitude':
                    print("Got attitude data %s" % repr(msg[1:]))
                    pass

        except KeyboardInterrupt:
            pass

if __name__ == '__main__':

    zmq_ctx = zmq.Context()

    ls = foo(zmq_ctx)
    ls.run()
