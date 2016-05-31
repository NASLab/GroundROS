#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from numpy import arctan2, sqrt, arange, pi, sin, cos
from sys import path
path.append('Modules/')
from sensor_msgs.msg import LaserScan
from time import time
from path_planning import GapFinder
import matplotlib.pyplot as plt


every_other = 3
increment = pi * .5 / 180
angles = arange(-3 * pi / 4, 3 * pi / 4 + increment, increment)[0::every_other]
kp = .4 / 1
kd = .3

# targets = [[2, [1, -.3]], [1, [1, -.3]]]


def withDistance(x, y, theta, distance):
    new_x = x + distance * cos(theta)
    new_y = y + distance * sin(theta)
    return new_x, new_y


class Navigation(object):

    def __init__(self):

        self.gap = .6
        self.position = []
        self.target = []
        self.stage = 0
        self.substage = 0

        self.path_planner = GapFinder(self.gap)
        rospy.init_node('Navigation')
        self.distance = []
        self.prev_closest_reading = 0.0
        self.prev_time = time()
        self.crash_avert_velocity = 0.0

        print 'Starting the Navigation'

        rospy.Subscriber('/husky/position', Pose2D, self.updatePosition, queue_size=1)
        rospy.Subscriber('/husky/target', Pose2D, self.updateTarget, queue_size=1)
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.move, queue_size=1)

        rospy.spin()

    def updatePosition(self, data):
        self.position = [data.x, data.y, data.theta]
        # print 'Robot: ' + '%.2f, ' * 3 % (self.position[0], self.position[1], self.position[2] * 180 / 3.1415)

    def updateTarget(self, data):
        self.target = [data.x, data.y, data.theta]
        # print 'target is at: ' + '%.2f, ' * 3 % (self.target[0], self.target[1], self.target[2] * 180 / 3.1415)

    def move(self, data):
        plt.ion()
        if not self.position or not self.target:
            return
        # target = withDistance(self.target[0], self.target[1], self.target[2], targets[self.stage][1][0])
        target = self.target
        # print 'Target is at', target
        plt.clf()
        plt.plot(self.position[0], self.position[1], 'ms', markersize=10, label='Robot')
        plt.arrow(self.position[0], self.position[1], self.gap * cos(self.position[2]), self.gap * sin(self.position[2]))
        plt.plot(self.target[0], self.target[1], 'cs', markersize=10, label='Destination')
        plt.plot(target[0], target[1], 'cs', markersize=5)
        robot_x, robot_y, robot_yaw = self.position
        distances = list(data.ranges)[0::every_other]

        if self.substage == 0:
            self.path_planner.filterReadings(distances, angles)
            closest_reading, closest_reading_angle = self.path_planner.getMinimumReading()

            # dynamic obstacle collision avoidance
            closest_reading = min(closest_reading, 2 * self.gap)
            time_now = time()
            self.crash_avert_velocity = (self.crash_avert_velocity + (closest_reading - self.prev_closest_reading) * kd / (time() - self.prev_time)) / 2
            self.crash_avert_velocity = min(0.0, self.crash_avert_velocity)

            # set velocity based on dynamic obstacle movement
            controlled_velocity = (closest_reading) * kp + self.crash_avert_velocity
            controlled_velocity = max(0.0, min(controlled_velocity, 1.0))
            target_x = target[0]
            target_y = target[1]
            diff_x = target_x - robot_x
            diff_y = target_y - robot_y
            self.distance = sqrt(diff_x**2 + diff_y**2)

            # plan path to the target
            angle = arctan2(diff_y, diff_x) - robot_yaw  # find direction towards target in robots coordinate frame
            subgoal_distance, subgoal_angle = self.path_planner.planPath(self.distance, -angle)
            # print 'Relative target is at', subgoal_distance, -subgoal_angle * 180 / 3.1415
            plt.plot(robot_x + subgoal_distance * cos(robot_yaw - subgoal_angle),
                     robot_y + subgoal_distance * sin(robot_yaw - subgoal_angle), 'g+', markersize=20, label='Best Subgoal')
            nums = len(self.path_planner.possible_travel)
            reading_x = [0] * nums
            reading_y = [0] * nums
            possible_travel_plot_x, possible_travel_plot_y = self.path_planner.polarToCartesian()
            for i in range(len(self.path_planner.possible_travel)):
                reading_x[i] = robot_x + self.path_planner.readings_polar[i][0] * cos(robot_yaw - self.path_planner.readings_polar[i][1])
                reading_y[i] = robot_y + self.path_planner.readings_polar[i][0] * sin(robot_yaw - self.path_planner.readings_polar[i][1])
                possible_travel_plot_x[i] = robot_x + self.path_planner.possible_travel[i] * cos(-self.path_planner.readings_polar[i][1] + robot_yaw)
                possible_travel_plot_y[i] = robot_y + self.path_planner.possible_travel[i] * sin(-self.path_planner.readings_polar[i][1] + robot_yaw)
            plt.plot(possible_travel_plot_x, possible_travel_plot_y, 'b.', markersize=10, label='Possible Travel')
            plt.plot(reading_x, reading_y, 'r.')
            if subgoal_distance < 0:
                print 'NOT MOVING FORWARD'

            # See if reached the destination
            if self.distance < .1:
                print '\033[92m' + '\033[1m' + 'ARRIVED TO GATE' + '\033[0m'
                return

            # save some of the variable needed for next iteration
            self.prev_closest_reading = closest_reading
            self.prev_time = time_now

        # elif self.substage == 1:
        #     front_travel = self.path_planner.getFrontTravel(distances, angles)
        #     # front_error = front_travel - targets[self.stage][1][1]
        #     # print front_travel, front_error
        #     # if abs(front_error) < .03:
        #         print 'BREAKINGGGGGGGGGGGGGGGGGGGG'
        #         return

        # elif self.substage == 2:
        #     front_travel = self.path_planner.getFrontTravel(distances, angles)
        #     # front_error = front_travel - targets[self.stage][1][1] - .2
        #     print front_travel, front_error
        #     if abs(front_error) < .03:
        #         self.stage += 1
        #         print 'BREAKINGGGGGGGGGGGGGGGGGGGG'
        #         return

        else:
            print 'stupid fuck'

        # axes = plt.gca()
        # axes.set_xlim([-15,15])
        # axes.set_ylim([-15,15])
        plt.axes().set_aspect('equal', 'datalim')
        plt.pause(.001)

if __name__ == "__main__":
    nav = Navigation()
