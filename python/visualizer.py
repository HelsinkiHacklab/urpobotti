#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division, print_function

import os
import sys

import visual
import zmq
import math

DEBUG = False

# All units in mm

class tank(object):
    size = (335, 185, 95)

    def __init__(self, **kwargs):
        self.container = visual.frame(**kwargs)
        self.box = visual.box(frame=self.container, length=self.size[0], width=self.size[1], height=self.size[2], color=(0,0,0.7))
        self.lidarframe = visual.frame(frame=self.container, pos=(self.size[0]/2.0, self.size[1]/2.0, self.size[2]))
        self.lidarpoints = visual.points(pos=[(0,0,0) for i in range(360)], frame=self.lidarframe, size=5, color=(0 , 1, 0))
        print(repr(self.lidarpoints))

    def update_lidar_point(self, angle, dist_mm, quality):
        if (   angle < 0
            or angle > 360):
                print("ERROR: Got invalid angle %d" % angle)
                return False
        angle_rad = angle * math.pi / 180.0
        c = math.cos(angle_rad)
        s = -math.sin(angle_rad)
        dist_x = dist_mm * c
        dist_y = dist_mm * s
        self.lidarpoints.pos[angle] = visual.vector(dist_x, 0, dist_y)


class myscene(object):

    def __init__(self, zmq_ctx):
        self.zmq_ctx = zmq_ctx

        visual.scene.width = 1280
        visual.scene.height = 768
        visual.scene.autoscale = True
        visual.scene.title = "Visualizing tank ZMQ ouput"
        self.tank = tank()

        self.navidata = zmq_ctx.socket(zmq.SUB)
        self.navidata.connect("tcp://sonofurpo.local:7578")
        self.navidata.setsockopt_string(zmq.SUBSCRIBE, u'')

        # Get them pollers
        self.poller = zmq.Poller()
        self.poller.register(self.navidata, zmq.POLLIN)




    def frame_recv(self, stream, msg):
        pass

    def run(self):
        print("Running")
        try:
            visual.scene.visible = True
            visual.scene.autoscale = False
            visual.scene.userzoom = True
#            ioloop.IOLoop.instance().start()
            while(True):
                socks = dict(self.poller.poll(30))
                if self.navidata in socks and socks[self.navidata] == zmq.POLLIN:
                    msg = self.navidata.recv_multipart()
                    
                    if msg[0] == 'lidar':
                        #print("Got lidar data %s" % repr(msg[1:]))
                        self.tank.update_lidar_point(int(msg[1]), int(msg[2]), int(msg[3]))
                        pass
    
                    if msg[0] == 'attitude':
                        print("Got attitude data %s" % repr(msg[1:]))
                        pass

        except KeyboardInterrupt:
            self.quit()

    def quit(self):
        print("Quitting")
#        ioloop.IOLoop.instance().stop()


if __name__ == '__main__':
    DEBUG = bool(int(os.environ.get('DEBUG', '0')))

    zmq_ctx = zmq.Context()

    ls = myscene(zmq_ctx)
    ls.run()
