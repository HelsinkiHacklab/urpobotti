#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division, print_function

import os
import sys

import visual
import zmq
import math
import json

DEBUG = False

# REMEMBER: Z depth not up...
# All units in mm

class tank(object):
    size_lwh = visual.vector(335, 185, 95)

    def __init__(self, **kwargs):
        self.size = visual.vector(self.size_lwh[0],self.size_lwh[2],self.size_lwh[1]) # VPython coordinate system, Z is depth aka width
        self.box_bl = visual.vector(-self.size_lwh[0]/2.0,-self.size_lwh[2]/2.0,-self.size_lwh[1]/2.0)
        self.container = visual.frame(**kwargs)
        self.box = visual.box(frame=self.container, length=self.size_lwh[0], width=self.size_lwh[1], height=self.size_lwh[2], color=(0,0,0.7))
        self.boxpoints = visual.points(pos=[self.box_bl, self.box_bl+self.size], frame=self.container, size=5, color=(1 , 0, 1))
        self.boxpoints.color[0] = (1,0,0)
        self.lidarframe = visual.frame(frame=self.container, pos=(0, self.size_lwh[2]/2.0+1, 0))
        self.lidarpoints = visual.points(pos=[(0,0,0) for i in range(360)], frame=self.lidarframe, size=5, color=(0 , 0.5, 0))
        for angle in range(360):
            self.update_lidar_point(angle, 20, None)

    def update_lidar_point(self, angle, dist_mm, quality):
        if (   angle < 0
            or angle > 359):
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
        self.scene = visual.scene # shortcut

        self.scene.width = 1280
        self.scene.height = 768
        self.scene.autoscale = True
        self.scene.title = "Visualizing tank ZMQ ouput"
        self.tank = tank()

        try:
            self.navidata = zmq_ctx.socket(zmq.SUB)
            self.navidata.connect("tcp://sonofurpo.local:7578")
            self.navidata.setsockopt_string(zmq.SUBSCRIBE, u'')

            # Get them pollers
            self.poller = zmq.Poller()
            self.poller.register(self.navidata, zmq.POLLIN)
        except zmq.error.ZMQError as e:
            self.poller = None
            print("Got exception '%s' from ZMQ" % e)

    def run(self):
        print("Running")
        try:
            self.scene.up = (0,1,0) # REMEMBER: Z depth not up...
            self.scene.visible = True
            self.scene.autoscale = False
            self.scene.range = self.tank.size*2
            self.scene.userzoom = True
            self.scene.forward = visual.vector(0.80, -0.60, 0.03)
            while(True):
                if self.poller:
                    socks = dict(self.poller.poll())
                    if self.navidata in socks and socks[self.navidata] == zmq.POLLIN:
                        msg = self.navidata.recv_multipart()

                        if msg[0] == 'lidar':
                            #print("Got lidar data %s" % repr(msg[1:]))
                            data = json.loads(msg[1])
                            for angle in range(360):
                                self.tank.update_lidar_point(angle, data[angle][0], data[angle][1])
                            pass

                        if msg[0] == 'attitude':
                            print("Got attitude data %s" % repr(msg[1:]))
                            pass
                # Poll for events (the callback is in newer version)
                if self.scene.mouse.events:
                    self.mbinteraction(self.scene.mouse.getevent())
                elif self.scene.kb.keys:
                    self.kbinteraction(self.scene.kb.getkey())


        except KeyboardInterrupt:
            self.quit()

    def mbinteraction(self, event):
        print("Got event %s" % repr(event))
        print("Forward %s up %s" % (repr(self.scene.forward),  repr(self.scene.up)))

    def kbinteraction(self, event):
        print("Got event %s" % repr(event))
        if event == '+':
            self.scene.autoscale = False
            self.scene.scale *= 1.5
        if event == '-':
            self.scene.autoscale = False
            self.scene.scale /= 1.5

    def quit(self):
        print("Quitting")
#        ioloop.IOLoop.instance().stop()


if __name__ == '__main__':
    DEBUG = bool(int(os.environ.get('DEBUG', '0')))

    zmq_ctx = zmq.Context()

    ls = myscene(zmq_ctx)
    ls.run()
