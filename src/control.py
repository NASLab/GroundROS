#!/usr/bin/env python

import rospy
import numpy as np
import pathGenerator as pthgen
from geometry_msgs.msg import Pose2D, Twist
import socket
import sys
import struct
from math import pi, sin, cos
from time import time, sleep
import PID
import logging
from datetime import datetime

logging.basicConfig(filename='control.log', format=50 * '=' +
                    '\n%(asctime)s %(message)s', level=logging.DEBUG)


class navigation_control(object):

    def __init__(self):
        self.logger = [[0, 0, 0, 0, 0, 0, 0, 0]]
        # kp_longitudinal = float(raw_input('Set longitudinal KP:'))
        self.longitudinal_pid = PID.PID(4, 0, 0)  # 1.6, 1.70484816196, 1.00106666667
        # kp_lateral = float(raw_input('Set lateral KP:'))
        self.lateral_pid = PID.PID(4, 0, 0)  # 1.6, 2.13, 0.8
        # kp_angle = float(raw_input('Set lateral KP:'))
        self.angle_pid = PID.PID(4, 0, 0)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(1)
        ip_address = '192.168.0.25'
        server_address = (ip_address, 1895)
        print >>sys.stderr, 'Connecting To %s Port %s' % server_address
        self.sock.connect(server_address)

        rospy.init_node('navigation_control')
        self.pub_pose = rospy.Publisher('qualisys_pose', Pose2D, queue_size=1)
        self.pub_cmd = rospy.Publisher('/husky/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(40)

        self.pose_msg = Pose2D()
        self.pose_msg.x = float('nan')
        self.pose_msg.y = float('nan')
        self.pose_msg.theta = float('nan')
        self.twist_msg = Twist()

        print 'Connection Established.'
        print 'Starting To Recieve And Publish Pose Data.'
        try:
            self.__run__()
        except Exception, e:
            np.save('long ' + str(self.longitudinal_pid) + ' lat ' +
                    str(self.lateral_pid) + str(datetime.now()), self.logger)
            logging.exception(e)
            raise
        finally:
            self.sock.close()
            np.save('long ' + str(self.longitudinal_pid) + ' lat ' +
                    str(self.lateral_pid) + ' ' + str(datetime.now()), self.logger)

    def __run__(self):
        pth = pthgen.PathGenerator(path_type='infinity', speed=.3)
        sleep(5)
        degree_to_rad = pi / 180
        reference_x, reference_y = pth.getPosition()
        while not rospy.is_shutdown():
            # get position
            check = struct.unpack('<B', self.sock.recv(1))[0]
            if check is 2:
                self.pub_pose.publish(self.pose_msg)
                self.rate.sleep()
            else:
                print 'Warning: Bad Qualisys Packet'
                continue
            recieved_data = self.sock.recv(4096)
            if len(recieved_data) < 12:
                print 'bad 2'
                self.pub_cmd.publish(Twist())
                continue
            self.pose_msg.x = struct.unpack('<f', recieved_data[:4])[0]
            self.pose_msg.y = struct.unpack('<f', recieved_data[4:8])[0]
            self.pose_msg.theta = struct.unpack('<f', recieved_data[8:12])[0] * degree_to_rad
            if self.pose_msg.x == float('nan'):
                print 'Recieved "NaN" For Pose'
                self.pub_cmd.publish(Twist())
                continue

            # calculate error
            reference_x_temp, reference_y_temp = pth.getPosition()

            reference_angle = np.arctan2(reference_y_temp - reference_y, reference_x_temp - reference_x)
            angle_error = reference_angle - self.pose_msg.theta
            angle_error_wrapped = (angle_error + np.pi) % (2 * np.pi) - np.pi
            print self.pose_msg.theta, angle_error_wrapped
            reference_x = reference_x_temp
            reference_y = reference_y_temp
            diff_x = reference_x - self.pose_msg.x / 1000
            diff_y = reference_y - self.pose_msg.y / 1000
            longitudinal_error = cos(self.pose_msg.theta) * diff_x + sin(self.pose_msg.theta) * diff_y
            lateral_error = cos(self.pose_msg.theta) * diff_y - sin(self.pose_msg.theta) * diff_x
            feedback_linear = self.longitudinal_pid.calculate(longitudinal_error)
            feedback_angular = self.lateral_pid.calculate(lateral_error) + self.angle_pid.calculate(angle_error_wrapped)
            # print feedback_linear, feedback_angular

            # Calculate actoator command
            limit = 1
            feedback_linear = min(limit, max(feedback_linear, -limit))
            limit = 2
            feedback_angular = min(limit, max(feedback_angular, -limit))
            self.twist_msg.linear.x = feedback_linear
            self.twist_msg.angular.z = feedback_angular
            self.pub_cmd.publish(self.twist_msg)
            self.logger = np.append(self.logger, [[longitudinal_error, lateral_error,
                                                   reference_x, reference_y,
                                                   self.pose_msg.x, self.pose_msg.y,
                                                   self.pose_msg.theta, time()]], axis=0)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        nav = navigation_control()
    except rospy.ROSInterruptException, e:
        np.save('long ' + str(nav.longitudinal_pid.kp) + 'lat ' +
                str(nav.lateral_pid.kp) + str(datetime.now()), nav.logger)
        logging.exception(e)
        print e
