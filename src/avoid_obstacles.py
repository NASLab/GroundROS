#!/usr/bin/env python

import rospy
from numpy import arctan2, sqrt, arange, pi
from sys import path
path.append('Modules/')
from communication import NaslabNetwork
from sensor_msgs.msg import LaserScan
from actuate import ROS2DimActuate
from tracker import PlanarTracker
from time import sleep
from path_planning import GapFinder

every_other = 3


def move(data):
    # print 'lidar data',data.angle_min*180/pi,data.angle_max*180/pi,data.angle_increment*180/pi
    distances = list(data.ranges)[0::every_other]
    if len(distances) != len(angles):
        raise Exception('lenght mismatch')
    path_planner.filterReadings(distances, angles)
    x, y, theta = connection.getStates()
    diff_x = target_x - x
    diff_y = target_y - y
    distance = sqrt(diff_x**2 + diff_y**2)
    if distance < .1:
        print 'ARRIVED MOTHERFUCKER!!!!!!!!!!!'
        subscriber.unregister()
    angle = arctan2(diff_y, diff_x) - theta
    subgoal_distance, subgoal_angle = path_planner.obstacleAvoidance(distance, -angle)
    # subgoal_x = x+subgoal_distance*cos(theta-subgoal_angle)
    # subgoal_y = y+subgoal_distance*sin(theta-subgoal_angle)

    # print subgoal_x,subgoal_y
    # print 'subgoal:',subgoal_x, subgoal_y
    # tracker.goToPoint(subgoal_x,subgoal_y)\

    # print theta+subgoal_angle,theta,subgoal_angle
    tracker.moveTowardsDynamicPoint(subgoal_distance, -subgoal_angle)


if __name__ == "__main__":
    sleep(7)
    # countDown()
    connection = NaslabNetwork()
    path_planner = GapFinder(.6)
    act = ROS2DimActuate()
    tracker = PlanarTracker(act.actuate, connection.getStates)
    target_x = 0
    target_y = 2
    increment = pi * .5 / 180
    angles = arange(-3 * pi / 4, 3 * pi / 4 + increment, increment)[0::every_other]

    # if len(angles) != 541:
    #     print -3 * pi / 4, 3 * pi / 4 + increment, increment, len(angles)
    #     raise Exception('bad angle range definition')

    # rospy.init_node('path_planner', anonymous=False)
    subscriber = rospy.Subscriber('/scan', LaserScan, move, queue_size=1)
    rospy.spin()
