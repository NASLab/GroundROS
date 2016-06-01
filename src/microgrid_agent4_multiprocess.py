#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from numpy import arctan2, sqrt, arange, pi, sin, cos, save
from sys import path
path.append('Modules/')
from rotation import rotate2DimFrame, wrapAnglePi
from communication import LabNavigation, MessageVerification
from sensor_msgs.msg import LaserScan
from actuate import ROS2DimActuate
from tracker import PlanarTracker
from time import sleep, time
from path_planning import GapFinder
from multiprocessing import Process, Value
# from datetime import datetime

every_other_lidar_reading = 1
data_log = 0
dynamic_env = 1
use_message = 0
targets = [[2, [1, -.2]], [1, [1, -.2]]]

if data_log:
    pose = Pose2D()

increment = pi * .5 / 180
angles = arange(-3 * pi / 4, 3 * pi / 4 + increment, increment)[0::every_other_lidar_reading]
kp = .4 / 1
kd = .3


def withDistance(x, y, theta, distance):
    new_x = x + distance * cos(theta)
    new_y = y + distance * sin(theta)
    return new_x, new_y


class Navigation(object):

    def __init__(self):

        self.gap = .6
        self.agent_id = 0
        self.stage = 0
        self.substage = 0

        self.go_to_target = Value('b', False)
        self.subgoal_x = Value('d', 0.0)
        self.subgoal_y = Value('d', 0.0)
        self.max_velocity = Value('d', 0.0)

        if data_log:
            self.position_publisher = rospy.Publisher('husky/position', Pose2D, queue_size=1)
            self.target_publisher = rospy.Publisher('husky/target', Pose2D, queue_size=1)

        if use_message:
            self.messaging = MessageVerification(True)
            self.messaging.connectToClient('192.168.4.11')
        self.path_planner = GapFinder(self.gap)
        # self.actuation = ROS2DimActuate()

        sleep_time = 1
        while sleep_time > 0:
            print "Mission starts in:", sleep_time
            sleep_time -= 1
            sleep(1)
        self.distance = []
        self.prev_closest_reading = 0.0
        self.prev_time = time()
        self.crash_avert_velocity = 0.0

        print 'Starting the Navigation'
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.move, queue_size=1)

        self.connection = LabNavigation()
        process = Process(target=self.goTo)
        process.start()

        # self.tracker = PlanarTracker(self.actuation.actuate, self.connection.getStates)
        # self.tracker.setID(self.agent_id)
        # rospy.init_node()
        # rospy.init_node('Navigation')
        # rospy.spin()

    def goTo(self):
        self.actuation2 = ROS2DimActuate()
        self.connection2 = LabNavigation()
        self.tracker2 = PlanarTracker(self.actuation2.actuate, self.connection2.getStates)
        self.tracker2.setID(self.agent_id)
        while not rospy.is_shutdown():
            # start_time = time()
            for i in range(64):
                if self.go_to_target.value:
                    # print 'GOING TO', self.subgoal_x.value, self.subgoal_y.value, 'with velocity', self.max_velocity.value
                    self.actuation2.setTangentialVelocityLimit(self.max_velocity.value)
                    self.tracker2.moveTowardsGlobalPoint(self.subgoal_x.value, self.subgoal_y.value)
                    # sleep(.03)
            # print 'goTo Hz is:',64/(time()-start_time)

    def move(self, data):
        agent_id, x, y, z, yaw, pitch, roll = self.connection.getStates(self.agent_id)
        if data_log:
            pose.x = x
            pose.y = y
            pose.theta = yaw
            self.position_publisher.publish(pose)

        # extract distance data and analyze them
        distances = list(data.ranges)[0::every_other_lidar_reading]

        if self.substage == 0:
            self.path_planner.filterReadings(distances, angles)
            closest_reading, closest_reading_angle = self.path_planner.getMinimumReading()

            # dynamic obstacle collision avoidance
            closest_reading = min(closest_reading, 2 * self.gap)
            if dynamic_env:
                time_now = time()
                self.crash_avert_velocity = (self.crash_avert_velocity + (closest_reading - self.prev_closest_reading) * kd / (time() - self.prev_time)) / 2
                self.crash_avert_velocity = min(0.0, self.crash_avert_velocity)

                # set velocity based on dynamic obstacle movement
                controlled_velocity = (closest_reading) * kp + self.crash_avert_velocity
                controlled_velocity = max(0.0, min(controlled_velocity, 1.0))
                self.max_velocity.value = min(1, controlled_velocity)

            # find destination and analyze it
            target_object = self.connection.getStates(targets[self.stage][0])
            if data_log:
                pose.x = target_object[1]
                pose.y = target_object[2]
                pose.theta = target_object[4]
                self.target_publisher.publish(pose)

            target = withDistance(target_object[1], target_object[2], target_object[4], targets[self.stage][1][0])
            target_x = target[0]
            target_y = target[1]
            diff_x = target_x - x
            diff_y = target_y - y
            self.distance = sqrt(diff_x**2 + diff_y**2)

            # plan path to the target
            angle = arctan2(diff_y, diff_x) - yaw  # find direction towards target in robots coordinate frame
            subgoal_distance, subgoal_angle = self.path_planner.planPath(self.distance, -angle)
            subgoal_angle2 = -subgoal_angle
            print 'SUBGOAAAAAAAAAAAAAL',subgoal_distance,subgoal_angle2*180/pi
            self.go_to_target.value = True
            # temp_x,temp_y= rotate2DimFrame(subgoal_distance,0,yaw+subgoal_angle2)
            # self.subgoal_x.value =temp_x+x
            # self.subgoal_y.value = temp_y+y
            self.subgoal_x.value = x + subgoal_distance * cos(yaw + subgoal_angle2)
            self.subgoal_y.value = y + subgoal_distance * sin(yaw + subgoal_angle2)

            # go to the point designated by path planner
            # self.tracker.moveTowardsDynamicPoint(subgoal_distance, subgoal_angle2)

            # See if reached the destination
            if self.distance < .1:
                print '\033[92m' + '\033[1m' + 'ARRIVED TO GATE' + '\033[0m'

                # face direction
                if targets[self.stage][1][0] < 0:
                    desired_facing = self.connection.getStates(targets[self.stage][0])[4]
                else:
                    desired_facing = pi + self.connection.getStates(targets[self.stage][0])[4]
                # self.tracker.faceDirection(desired_facing)
                self.stage += 1
                # self.substage = 1
                sleep(1)

            if dynamic_env:
                # save some of the variable needed for next iteration
                self.prev_closest_reading = closest_reading
                self.prev_time = time_now

        elif self.substage == 1:
            self.go_to_target.value = False
            front_travel = self.path_planner.getFrontTravel(distances, angles)
            front_error = front_travel - targets[self.stage][1][1]
            if abs(front_error) < .03:
                self.substage = 2
                print 'Reached Connection point.'
                if use_message:
                    self.messaging.sendMessage("make connection")
                    while not self.messaging.verifyMessage("connection done"):
                        sleep(.5)
                else:
                    sleep(5)
            # self.actuation.actuate(.5 * front_error, 0)

        elif self.substage == 2:
            front_travel = self.path_planner.getFrontTravel(distances, angles)
            front_error = front_travel - .1
            if abs(front_error) < .03:
                self.stage += 1
                self.substage = 0
                print 'Departed from connection point.'
            # self.actuation.actuate(.5 * front_error, 0)

        else:
            print 'stupid fuck did somethinng weird'

        if self.stage == len(targets):
            # self.tracker.saveLog()
            # save('/home/administrator/barzin_catkin_ws/src/path_tracking/scripts/experimental_results/planner_of_agent_' + str(self.agent_id), log)
            self.subscriber.unregister()
            print '\033[92m' + '\033[1m' + 'AND DONE' + '\033[0m'
        elif self.stage > len(targets):  # just do nothing after that
            print "Stupid shit didn't unregister"
            sleep(100)


if __name__ == "__main__":
    nav = Navigation()
