#!/usr/bin/env python

import rospy
from numpy import arctan2, sqrt, arange, pi, sign
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

        self.gap = .9
        # self.space = .9
        self.target_x = 0
        self.target_y = 2
        self.agent_id = 1




        self.connection = LabNavigation()
        self.path_planner = GapFinder(self.gap)
        self.actuation = ROS2DimActuate()
        self.tracker = PlanarTracker(self.actuation.actuate, self.connection.getStates)

        self.tracker.setID(self.agent_id)

        sleep(5)
        self.distance = []
        self.prev_closest_reading = 0.0
        self.prev_time = time()
        self.crash_avert_velocity = 0.0

        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.move, queue_size=1)
        rospy.spin()

    def move(self, data):
        distances = list(data.ranges)[0::every_other]
        self.path_planner.filterReadings(distances, angles)
        closest_reading,closest_reading_angle = self.path_planner.getMinimumReading()
        closest_reading = min(closest_reading,2*self.gap)
        time_now = time()
        self.crash_avert_velocity = (self.crash_avert_velocity + (closest_reading - self.prev_closest_reading) * kd / (time() - self.prev_time)) / 2
        self.crash_avert_velocity = min(0.0, self.crash_avert_velocity)
        # print 'Crash avert velocity:% 4.2f'%self.crash_avert_velocity
        controlled_velocity = (closest_reading) * kp + self.crash_avert_velocity
        controlled_velocity = max(0.0, min(controlled_velocity, 1.0))
        # print 'Controlled Velocity:', controlled_velocity,
        # print 'closest_reading:',closest_reading,
        # print 'Crash avert velocity:',self.crash_avert_velocity
        self.actuation.setTangentialVelocityLimit(min(1, controlled_velocity))

        agent_id, x, y, z, yaw, pitch, roll = self.connection.getStates(self.agent_id)
        # print 'agent location (x,y):', x, y, yaw

        diff_x = self.target_x - x
        diff_y = self.target_y - y
        self.distance = sqrt(diff_x**2 + diff_y**2)
        # print 'distance',self.distance
        if self.distance < .1:
            print 'ARRIVED!!!!!!!!!!'
            self.target_y = self.target_y * -1

        angle = arctan2(diff_y, diff_x) - yaw
        subgoal_distance, subgoal_angle = self.path_planner.planPath(self.distance, -angle)
        subgoal_angle2 = -subgoal_angle
        # print angle,subgoal_angle2
        # faz = 1
        # var = min(max(0,self.gap*(1+faz)-closest_reading),faz)
        # offset = var*pi/faz/4
        # subgoal_angle2 = subgoal_angle2+offset*sign(subgoal_angle2-(-closest_reading_angle))
        # print '% 4.2f, % 4.2f, % 4.2f' % (var, offset,offset*sign(subgoal_angle2-(-closest_reading_angle)))
        self.tracker.moveTowardsDynamicPoint(subgoal_distance, subgoal_angle2)
        # print 'target angle:',yaw+subgoal_angle2
        self.prev_closest_reading = closest_reading
        self.prev_time = time_now
        # print "TarX:% 4.2f, TarY:% 4.2f, PosX:% 4.2f, PosY:% 4.2f, SubAng:% 4.2f, SubAng+Yaw:% 4.2f" %(self.target_x,self.target_y,x,y,subgoal_angle,subgoal_angle+yaw)


if __name__ == "__main__":
    nav = Navigation()
