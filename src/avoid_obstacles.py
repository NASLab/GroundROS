#!/usr/bin/env python

import rospy
from numpy import arctan2, sqrt, arange, pi
from sys import path
path.append('Modules/')
from communication import HuskeyConnect
from path_planning import GapFinder
from sensor_msgs.msg import LaserScan
from actuate import ROS2DimActuate
from tracker import PlanarTracker


def move(data):
    distances = data
    path_planner.filterReadings(distances, angles)
    x, y, theta = connection.getStates()
    diff_x = target_x - x
    diff_y = target_y - y
    distance = sqrt(diff_x**2 + diff_y**2)
    if distance < .1:
        print 'ARRIVED SUCCESSFULLY !!!!!!!'
        return
    angle = arctan2(diff_y, diff_x) - theta
    subgoal_distance, subgoal_angle = path_planner.planPath(distance, angle)
    tracker.moveTowardsDynamicPoint(subgoal_distance, subgoal_angle)


if __name__ == "__main__":
    connection = HuskeyConnect()
    path_planner = GapFinder(1.2)
    act = ROS2DimActuate()
    tracker = PlanarTracker(act.actuate)
    target_x = 1
    target_y = 1
    increment = pi * .5 / 180
    angles = arange(-3 * pi / 4, 3 * pi / 4 + increment, increment)

    if len(angles) != 541:
        print -3 * pi / 4, 3 * pi / 4 + increment, increment, len(angles)
        raise Exception('bad angle range definition')

    rospy.init_node('path_planner', anonymous=False)
    rospy.Subscriber('/scan', LaserScan, move)
    rospy.spin()
