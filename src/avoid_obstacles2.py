#!/usr/bin/env python

import rospy
from numpy import arctan2, sqrt, arange, pi, sin, cos
from sys import path
path.append('Modules/')
from communication import NaslabNetwork
from sensor_msgs.msg import LaserScan
from actuate import ROS2DimActuate
from tracker import PlanarTracker
from time import sleep
from path_planning import GapFinder

every_other = 3
increment = pi * .5 / 180
angles = arange(-3 * pi / 4, 3 * pi / 4 + increment, increment)[0::every_other]


class Navigation(object):

    def __init__(self):
        sleep(5)
        self.distance = []
        self.connection = NaslabNetwork()
        self.path_planner = GapFinder(.5)
        self.actuation = ROS2DimActuate()
        self.tracker = PlanarTracker(self.actuation.actuate, self.connection.getStates)
        self.tracker.setID(1)
        self.target_body_number = 3
        self.space = .9
        # self.direction = 3
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.move, queue_size=1)
        rospy.spin()

    # def start(self):

    def move(self, data):
        distances = list(data.ranges)[0::every_other]
        self.path_planner.filterReadings(distances, angles)

        x, y, theta = self.connection.getStates(1)
        target_x, target_y, target_theta = self.connection.getStates(self.target_body_number)
        target_x = target_x + self.space * cos(target_theta)
        target_y = target_y + self.space * sin(target_theta)
        diff_x = target_x - x
        diff_y = target_y - y
        self.distance = sqrt(diff_x**2 + diff_y**2)

        if self.distance < .03:

            if self.target_body_number == 3:
                self.tracker.faceDirection(target_theta+pi)
                print 'ARRIVED!!!!!!!!!!!'
                self.subscriber.unregister()
                return

            # if self.target_body_number == 2:
            #     self.tracker.faceDirection(pi+target_theta)
            #     self.target_body_number = 1
            #     self.space = -1.1
            #     print 'ARRIVED!!!!!!!!!!!'
            #     sleep(4)

        angle = arctan2(diff_y, diff_x) - theta
        subgoal_distance, subgoal_angle = self.path_planner.planPath(self.distance, -angle)
        self.tracker.moveTowardsDynamicPoint(subgoal_distance, -subgoal_angle)



if __name__ == "__main__":
    nav = Navigation()
