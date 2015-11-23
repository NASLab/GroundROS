#!/usr/bin/env python

import rospy
from numpy import arctan2, sqrt, arange, pi
from sys import path
path.append('Modules/')
from communication import LabNavigation
from sensor_msgs.msg import LaserScan
from actuate import ROS2DimActuate
from tracker import PlanarTracker
from time import sleep, time
from path_planning import GapFinder

every_other = 3
increment = pi * .5 / 180
angles = arange(-3 * pi / 4, 3 * pi / 4 + increment, increment)[0::every_other]
kp = .4 / 1
kd = .3


class Navigation(object):

    def __init__(self):

        self.connection = LabNavigation()
        self.path_planner = GapFinder(.7)
        self.actuation = ROS2DimActuate()
        self.tracker = PlanarTracker(self.actuation.actuate, self.connection.getStates)

        self.tracker.setID(1)
        self.space = .9
        self.target_x = 0
        self.target_y = 2

        sleep(5)
        self.distance = []
        self.prev_closest_reading = 0.0
        self.prev_time = time()
        self.crash_avert_velocity=0.0

        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.move, queue_size=1)
        rospy.spin()

    def move(self, data):
        distances = list(data.ranges)[0::every_other]
        self.path_planner.filterReadings(distances, angles)
        closest_reading = self.path_planner.getMinimumReading()
        time_now = time()
        self.crash_avert_velocity = (self.crash_avert_velocity+(closest_reading - self.prev_closest_reading) * kd / (time() - self.prev_time))/2
        self.crash_avert_velocity = min(0.0,self.crash_avert_velocity)
        controlled_velocity = closest_reading * kp + self.crash_avert_velocity
        controlled_velocity = max(0.0,min(controlled_velocity,1))
        print 'Controlled Velocity:', controlled_velocity,
        print 'closest_reading:',closest_reading,
        print 'Crash avert velocity:',self.crash_avert_velocity
        self.actuation.setTangentialVelocityLimit(controlled_velocity)

        agent_id, x, y, z, yaw, pitch, roll = self.connection.getStates(1)
        # print 'agent location (x,y):', x, y, yaw

        diff_x = self.target_x - x
        diff_y = self.target_y - y
        self.distance = sqrt(diff_x**2 + diff_y**2)

        if self.distance < .03:
            self.target_y = self.target_y * -1

        angle = arctan2(diff_y, diff_x) - yaw
        subgoal_distance, subgoal_angle = self.path_planner.planPath(self.distance, -angle)
        self.tracker.moveTowardsDynamicPoint(subgoal_distance, -subgoal_angle)
        self.prev_closest_reading = closest_reading
        self.prev_time = time_now


if __name__ == "__main__":
    nav = Navigation()
