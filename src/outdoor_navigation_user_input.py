#!/usr/bin/env python

# Algorith to drive robot from one point to another while avoiding obstacles


import rospy
from geometry_msgs.msg import Pose2D
from numpy import arctan2, sqrt, arange, pi
from sys import path, argv
path.append(path[0]+'/Modules/')
from actuate import ROS2DimActuate
from gpsLocalization import gpsLocalization
from sensor_msgs.msg import LaserScan
from tracker import PlanarTracker
from time import sleep, time
from path_planning import GapFinder


every_other_lidar_reading = 3
increment = pi * .5 / 180
angles = arange(-3 * pi / 4, 3 * pi / 4 + increment, increment)[0::every_other_lidar_reading]
kp = .4 / 1
kd = .3
pose = Pose2D()

class Navigation(object):

    def __init__(self, target_x, target_y):

        # set parameters
        self.gap = .7  # space needed to pass through
        self.target_x = target_x  # destination coordinates
        self.target_y = target_y
        self.agent_id = 0

        rospy.init_node('navigation',anonymous=True)
        self.connection = gpsLocalization()  # Connection which will give current position
        self.path_planner = GapFinder(self.gap)  # Finds gaps that the robot can enter
        self.actuation = ROS2DimActuate()  # Controls the motion of the robot
        self.actuation.setAngularVelocityLimit(.5)  # Sets the maximum velocity
        # Create a tracker which knows how to move the robot and get it's position
        self.tracker = PlanarTracker(self.actuation.actuate, self.connection.getStates)
        # Tell the tracker which robot to command
        self.tracker.setID(self.agent_id)

        self.distance = []
        self.prev_closest_reading = 0.0
        self.prev_time = time()
        self.crash_avert_velocity = 0.0
        self.position_publisher = rospy.Publisher('husky/position', Pose2D, queue_size=1)
        self.target_publisher = rospy.Publisher('husky/target', Pose2D, queue_size=1)
        print 'Starting the Navigation'
        sleep(2)
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.move, queue_size=1)  # Call move for each laser scan

        rospy.spin()

    def move(self, data):
        if not self.position or not self.target:
            return
        agent_id, x, y, z, yaw, pitch, roll = self.connection.getStates(self.agent_id)  # Get localization info
        pose.x = x
        pose.y = y
        pose.theta = yaw
        self.position_publisher.publish(pose)
        pose.x = self.target_x
        pose.y = self.target_y
        self.target_publisher.publish(pose)

        print '-----------------------------'
        distances = list(data.ranges)[0::every_other_lidar_reading]  # store the range readings from the lidar
        self.path_planner.filterReadings(distances, angles)  # filter the results

        closest_reading, closest_reading_angle = self.path_planner.getMinimumReading()
        closest_reading = min(closest_reading, 2 * self.gap)
        time_now = time()
        self.crash_avert_velocity = (self.crash_avert_velocity + (closest_reading - self.prev_closest_reading) * kd / (time() - self.prev_time)) / 2
        self.crash_avert_velocity = min(0.0, self.crash_avert_velocity)

        controlled_velocity = (closest_reading) * kp + self.crash_avert_velocity
        controlled_velocity = max(0.0, min(controlled_velocity, 1.0))

        self.actuation.setTangentialVelocityLimit(min(.2, controlled_velocity))

        diff_x = self.target_x - x
        diff_y = self.target_y - y
        self.distance = sqrt(diff_x**2 + diff_y**2)

        if self.distance < .1:
            print 'ARRIVED!!!!!!!!!!'
            self.subscriber.unregister()
            rospy.signal_shutdown('Navigation has reached destination.')

        angle = arctan2(diff_y, diff_x) - yaw
        print 'dist: ', self.distance, 'angle: ', -angle
        subgoal_distance, subgoal_angle = self.path_planner.planPath(self.distance, -angle)
        subgoal_angle2 = -subgoal_angle

        self.tracker.moveTowardsDynamicPoint(subgoal_distance, subgoal_angle2)

        self.prev_closest_reading = closest_reading
        self.prev_time = time_now


if __name__ == "__main__":
    print 'Number of arguments:', len(argv), 'arguments.'
    print 'Argument List:', str(argv)
    try:
        target_x = float(argv[1])
        target_y = float(argv[2])
    except IndexError:
        print '-------------'
        print 'ERROR: You need to add 2 arguments for destination coordinates'
        print '-------------'
        raise
    nav = Navigation(target_x, target_y)
