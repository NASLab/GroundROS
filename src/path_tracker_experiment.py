#!/usr/bin/env python
import PID
import rospy
import numpy as np
import pathGenerator as pthgen
from geometry_msgs.msg import Pose2D, Twist
from time import sleep
import logging
# from datetime import datetime
import HuskeyConnect as HC
import ActuationControl as AC


logging.basicConfig(filename='control.log', format=50 * '=' +
                    '\n%(asctime)s %(message)s', level=logging.DEBUG)


class navigation_control(object):

    def __init__(self):
        self.logger = [[0, 0, 0, 0, 0, 0, 0, 0]]
        self.connection = HC.HuskeyConnect()
        self.angle_pid = PID.PID(4, 0, 0)

        # setup ROS node and topics
        rospy.init_node('navigation_control')
        self.pub_cmd = rospy.Publisher('/husky/cmd_vel', Twist, queue_size=1)

        # setup execution rate
        self.rate = rospy.Rate(20)

        # initializing pose_msg
        self.pose_msg = Pose2D()
        self.pose_msg.x = float('nan')
        self.pose_msg.y = float('nan')
        self.pose_msg.theta = float('nan')
        self.twist_msg = Twist()
        print 'Connection Established.'
        print 'Starting To Recieve And Publish Pose Data.'
        try:
            self.__run()
        except Exception, e:
            # np.save('/home/administrator/barzin_catkin_ws/src/path_tracking/scripts/experimental_results/' + str(datetime.now()), self.logger)
            logging.exception(e)
        finally:
            self.connection.close()
            # np.save('/home/administrator/barzin_catkin_ws/src/path_tracking/scripts/experimental_results/' + str(datetime.now()), self.logger)

    def __run(self):
        sleep(5)
        bound = .03
        if not rospy.is_shutdown():
            for i in range(1):
                self.pth = pthgen.PathGenerator(path_type='circle', speed=.05)
                # self.act_control = AC.ActuationControl(self.pth, longitudinal_p=10, lateral_p=10, angle_p=0)
                # log = [[-666, -666, -666, -666, -666, -666]]
                # self.logger = np.append(self.logger, log, axis=0)
                make_point = pthgen.PathGenerator(path_type='point')
                make_point.setPoint(self.pth.x_function(0), self.pth.y_function(0))
                move_to_point = AC.ActuationControl(make_point, longitudinal_p=2, lateral_p=2, angle_p=0)
                self.pose_msg.x, self.pose_msg.y, self.pose_msg.theta = self.connection.getStates()
                errors = move_to_point.calculateError(self.pose_msg)
                angle_error_wrapped = np.pi
                counter = 0
                # while (abs(errors[0][0]) > bound or abs(errors[0][1]) > bound or abs(angle_error_wrapped) > (4 * np.pi / 180)) and counter < 300:
                print errors
                print abs(errors[0][0]) > bound, abs(errors[0][1]) > bound, abs(angle_error_wrapped) > (4 * np.pi / 180)
                while (abs(errors[0][0]) > bound or abs(errors[0][1]) > bound) and counter < 300:
                    while abs(errors[0][1]) > bound and counter < 300:
                        print 'fixing lat'
                        self.pose_msg.x, self.pose_msg.y, self.pose_msg.theta = self.connection.getStates()
                        errors = move_to_point.calculateError(self.pose_msg)
                        feedback_linear, feedback_angular = move_to_point.actuationCalculation(error_minimizer=False)
                        self.twist_msg.linear.x = 0
                        print errors
                        self.twist_msg.angular.z = feedback_angular
                        self.pub_cmd.publish(self.twist_msg)
                        self.rate.sleep()
                    # errors = move_to_point.calculateError(self.pose_msg)
                    while abs(errors[0][0]) > bound and counter < 300:
                        print 'fixing long'
                        self.pose_msg.x, self.pose_msg.y, self.pose_msg.theta = self.connection.getStates()
                        errors = move_to_point.calculateError(self.pose_msg)
                        feedback_linear, feedback_angular = move_to_point.actuationCalculation(error_minimizer=False)
                        self.twist_msg.linear.x = feedback_linear
                        self.twist_msg.angular.z = 0
                        self.pub_cmd.publish(self.twist_msg)
                        self.rate.sleep()
                    # errors = move_to_point.calculateError(self.pose_msg)
                self.twist_msg.linear.x = 0
                angle_error_wrapped = np.pi
                while abs(angle_error_wrapped) > (1 * np.pi / 180) and counter < 300:
                    print 'fixing orientation'
                    # reference_x_temp, reference_y_temp = self.path.getPosition()
                    self.pose_msg.x, self.pose_msg.y, self.pose_msg.theta = self.connection.getStates()
                    reference_angle = np.pi / 2
                    angle_error = reference_angle - self.pose_msg.theta
                    angle_error_wrapped = (angle_error + np.pi) % (2 * np.pi) - np.pi
                    feedback_angular = self.angle_pid.calculate(angle_error_wrapped)
                    feedback_angular = min(2, max(feedback_angular, -2))
                    self.twist_msg.angular.z = feedback_angular
                    self.pub_cmd.publish(self.twist_msg)
                    self.rate.sleep()

                # self.pose_msg.x, self.pose_msg.y, self.pose_msg.theta = self.connection.getStates()
                # if self.pose_msg.x == float('nan'):
                #     print 'Recieved "NaN" For Pose'
                #     self.pub_cmd.publish(Twist())
                #     continue
                # log = self.act_control.calculateError(self.pose_msg)
                # feedback_linear, feedback_angular = self.act_control.actuationCalculation()
                # self.twist_msg.linear.x = feedback_linear
                # self.twist_msg.angular.z = feedback_angular
                # self.pub_cmd.publish(self.twist_msg)

                # self.logger = np.append(self.logger, log, axis=0)
                # self.rate.sleep()


if __name__ == "__main__":
    try:
        nav = navigation_control()
    except rospy.ROSInterruptException, e:
        # np.save('/home/administrator/barzin_catkin_ws/src/path_tracking/scripts/experimental_results/' + str(datetime.now()), nav.logger)
        logging.exception(e)
        print e
