#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys,os
import zmq
from zmq.eventloop import ioloop
from zmq.eventloop.zmqstream import ZMQStream
ioloop.install()
import itertools
import random
import time

import zmqdecorators

service_name="urpobot.motor"
resp = zmqdecorators.call_sync(service_name, "setspeeds", sys.argv[1], sys.argv[1])
print "got %s" % (repr(resp))