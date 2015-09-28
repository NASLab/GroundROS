#!/usr/bin/env python

from tracker import PlanarTracker
from communication import HuskeyConnect
from actuate import ROS2DimActuate
from numpy import pi
from path_generator import PathGenerator


connection = HuskeyConnect()
act = ROS2DimActuate()
tracker = PlanarTracker(act.actuate, connection.getStates)

for i in range(10):
    tracker.goToPoint(1, 0)
    tracker.faceDirection(pi/2)
    path = PathGenerator(path_type='circle', speed=.3)
    tracker.followPath(path)
