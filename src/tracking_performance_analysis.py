#!/usr/bin/env python

import rospy
from time import sleep

from sys import path
path.append('Modules/')
from tracker import PlanarTracker
from communication import LabNavigation
from actuate import ROS2DimActuate
from numpy import pi
from path_generator import PathGenerator


connection = LabNavigation()
act = ROS2DimActuate()
tracker = PlanarTracker(act.actuate, connection.getStates)
tracker.setID(0)
path = PathGenerator(path_type='two_lines', speed=.2)


sleep(2)

# tracker.goToPoint(0, -2)
#
# tracker.faceDirection(-pi/4)

start_location = path.getBeginning()
tracker.goToPoint(start_location[0]+.3, start_location[1])
sleep(1)
tracker.faceDirection(pi / 2)
sleep(1)
path.startNow()
_, x, y, _, _, _, _ = connection.getStates(0)
tracker.followPath(path, x - start_location[0], y - start_location[1])

# tracker.goToPoint(-2.5/10, -2.5)
# tracker.faceDirection(pi / 2)
# path.setVelocity = .3
# sleep(1)
# path.startNow()
# tracker.followTrajectory(path)
