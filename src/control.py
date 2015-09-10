#!/usr/bin/env python

import rospy
import numpy as np
import pathGenerator as pthgen
from geometry_msgs.msg import Pose2D, Twist
from time import sleep
import logging
from datetime import datetime
import HuskeyConnect as HC
import ActuationControl as AC


logging.basicConfig(filename='control.log', format=50 * '=' +
                    '\n%(asctime)s %(message)s', level=logging.DEBUG)


class navigation_control(object):

    def __init__(self):
        self.logger = [[0, 0, 0, 0, 0, 0, 0, 0]]
        pth = pthgen.PathGenerator(path_type='two_lines', speed=.3)
        self.act_control = AC.ActuationControl(pth)
        self.connection = HC.HuskeyConnect()

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
            np.save('/home/administrator/barzin_catkin_ws/src/path_tracking/scripts/experimental_results/'+str(datetime.now()), self.logger)
            logging.exception(e)
        finally:
            self.connection.close()
            np.save('/home/administrator/barzin_catkin_ws/src/path_tracking/scripts/experimental_results/'+str(datetime.now()), self.logger)

    def __run(self):
        sleep(5)
        while not rospy.is_shutdown():
            self.pose_msg.x, self.pose_msg.y, self.pose_msg.theta = self.connection.getStates()
            # self.act_control.calculateError(self.pose_msg)
            if self.pose_msg.x == float('nan'):
                print 'Recieved "NaN" For Pose'
                self.pub_cmd.publish(Twist())
                continue
            log = self.act_control.calculateError(self.pose_msg)
            feedback_linear, feedback_angular = self.act_control.actuationCalculation()
            self.twist_msg.linear.x = feedback_linear
            self.twist_msg.angular.z = feedback_angular
            self.pub_cmd.publish(self.twist_msg)

            self.logger = np.append(self.logger, log, axis=0)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        nav = navigation_control()
    except rospy.ROSInterruptException, e:
        np.save('/home/administrator/barzin_catkin_ws/src/path_tracking/scripts/experimental_results/'+str(datetime.now()), nav.logger)
        logging.exception(e)
        print e
