#!/usr/bin/env python
# -*- coding: utf-8 -*-
import zmq
from zmq.eventloop import ioloop as ioloop_mod
import zmqdecorators
import json
import time
import collections

class ZMQLidar(zmqdecorators.client):
    lidar_data = [(0,0) for x in range(360)]
    _last_lidar_time = None
    speed_rpm = 300
    angle_adjust = 90

    def __init__(self, service_name):
        super(ZMQLidar, self).__init__()
        zmqdecorators.subscribe_topic(service_name, 'lidar', self.lidar_callback)
        # TODO: check how old our data is and null the array if too old
        zmqdecorators.subscribe_topic(service_name, 'lidar_rpm', self.rpm_callback)

    def lidar_callback(self, jsondata, timestamp):
        # TODO:reject too old data ?
        self.lidar_data = json.loads(jsondata)
        if self.angle_adjust != 0:
            dq = collections.deque(self.lidar_data)
            dq.rotate(self.angle_adjust)
            self.lidar_data = list(dq)
            pass
        self._last_lidar_time = time.time()

    def rpm_callback(self, rpm, timestamp):
        # TODO:reject too old data ?
        self.speed_rpm = float(rpm)

    def getScan(self):
        return self.lidar_data

    def getRPM(self):
        return self.speed_rpm

