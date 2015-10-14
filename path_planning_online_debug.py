#!/usr/bin/env python

import matplotlib.pyplot as plt
import rospy
# import numpy as np
from numpy import arange, pi, sqrt, arctan2, sin, cos
# from time import time, sleep
# from datetime import datetime
# import logging
from sensor_msgs.msg import LaserScan  # Messages from Lidar System
# import sys
from sys import path
path.append('src/Modules/')
from path_planning import GapFinder
from communication import HuskeyConnect

every_other = 3
increment = pi * .5 / 180
angles = arange(-3 * pi / 4, 3 * pi / 4 + increment, increment)[0::every_other]

# logging.basicConfig(filename='control.log', format=50 * '=' +
# '\n%(asctime)s %(message)s', level=logging.DEBUG)  # set up debug logging


class LidarLogger(object):

    def __init__(self):
        self.path_planner = GapFinder(1)
        rospy.init_node('lidarLogger')  # start the control node
        self.logger = []
        self.connection = HuskeyConnect()
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.readDistance)
        self.rate = rospy.Rate(10)  # set rate to 10 Hz
        # read = 0
        while not rospy.is_shutdown() and len(self.logger) < 1:
            self.rate.sleep()
            # rospy.spin()

        # try:
        #     pass
        # self.__run__() #start motion
        # except Exception, e:
        #     print e
        #     print "Error during runtime. Exiting."
        #     logging.exception(e)
        # finally:
        #     print "Run complete."
        # print self.logger  # REMOVE LATER
        #     np.save('scan' + str(datetime.now()), self.logger)

    # def __run__(self):
    # sleep(5) #delay before startup
    #     while not rospy.is_shutdown():
    #         self.rate.sleep()

    def readDistance(self, data):
        print 'here'
        self.subscriber.unregister()
        distances = list(data.ranges)[0::every_other]
        self.path_planner.filterReadings(distances, angles)
        x, y = self.path_planner.polarToCartesian()
        robot_x, robot_y, robot_theta = self.connection.getStates()
        target_x = 0
        target_y = -2
        diff_x = target_x - robot_x
        diff_y = target_y - robot_y
        distance = sqrt(diff_x**2 + diff_y**2)
        if distance < .1:
            # print '.'
            return
        print 'here'
        angle = arctan2(diff_y, diff_x) - robot_theta
        subgoal_distance, subgoal_angle = self.path_planner.obstacleAvoidance(distance, -angle)
        # scan = data.ranges
        # print scan
        # self.logger = scan
        print "Lidar Scan Recieved. Logging data..."
        f0 = plt.figure()
        ax0 = f0.add_subplot(111)
        nums = len(self.path_planner.possible_travel)
        # for i in range(nums):
        # ax0.plot(x, y, 'r.')
        ax0.plot(robot_x, robot_y, 'ko', markersize=5)
        reading_x = [0] * nums
        reading_y = [0] * nums
        for i in range(len(self.path_planner.possible_travel)):
            # print i,len(x)
            # print self.path_planner.possible_travel
            reading_x[i] = robot_x + self.path_planner.readings_polar[i][0] * cos(robot_theta - self.path_planner.readings_polar[i][1])
            reading_y[i] = robot_y + self.path_planner.readings_polar[i][0] * sin(robot_theta - self.path_planner.readings_polar[i][1])
            x[i] = robot_x + self.path_planner.possible_travel[i] * cos(-self.path_planner.readings_polar[i][1] + robot_theta)
            y[i] = robot_y + self.path_planner.possible_travel[i] * sin(-self.path_planner.readings_polar[i][1] + robot_theta)
            if i in self.path_planner.subgoals:
                ax0.plot(x[i], y[i], 'mo', markersize=10)
        # if environment_state is 'not_safe':
        ax0.plot(reading_x, reading_y, 'r.')
        ax0.plot(robot_x + subgoal_distance * cos(robot_theta - subgoal_angle),
                 robot_y + subgoal_distance * sin(robot_theta - subgoal_angle), 'mo', markersize=20)
        # elif environment_state is 'safe':
        #     ax0.plot(target_distance * cos(target_angle),
        #              target_distance * sin(target_angle), 'go', markersize=20)
        # elif environment_state is 'close_to_obstacle':
        #     ax0.plot(target_distance * cos(target_angle),
        #              target_distance * sin(target_angle), 'ro', markersize=20)
        ax0.plot(target_x, target_y, 'cx', markersize=20, linewidth=10)
        ax0.plot(x, y, 'b.')
        ax0.axis('equal')
        plt.draw()
        plt.pause(.1)
        raw_input("<Hit Enter To Close>")
        plt.close(f0)


if __name__ == "__main__":
    try:
        lidarLog = LidarLogger()
    except rospy.ROSInterruptException, e:
        raise e
