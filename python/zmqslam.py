#!/usr/bin/env python
import zmq
from zmq.eventloop import ioloop as ioloop_mod
import zmqdecorators


MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 10
LIDAR_SERVICE            = 'urpobot.pings'

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.components import XVLidar as LaserModel

from zmqlidar import ZMQLidar as Lidar


class slamthing(object):
    def __init__(self):
        # Connect to Lidar unit
        self.lidar = Lidar(LIDAR_SERVICE)
    
        # Create an RMHC SLAM object with a laser model and optional robot model
        self.slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

        # Initialize an empty trajectory
        self.trajectory = []
    
        # Initialize empty map
        self.mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

        # Iterate at about 11Hz (double the 5.5Hz for the XV lidar)
        self.pcb = ioloop_mod.PeriodicCallback(self._iterate, 90)
        self.pcb.start()

    def _iterate(self):
        # Update SLAM with current Lidar scan, using first element of (scan, quality) pairs
        self.slam.update([pair[0] for pair in lidar.getScan()])

        # Get current robot position
        self.x, self.y, self.theta = self.slam.getpos()

        # Get current map bytes as grayscale
        self.slam.getmap(self.mapbytes)

        self.trajectory.append((x,y))


if __name__ == '__main__':
    m = slamthing()
    m.run()
