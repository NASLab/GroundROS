import numpy as np
from time import time
import rospy
from geometry_msgs.msg import Twist

nan_var = float('nan')
stop_msg = Twist()

def saturate(value, limit=1):
    return min(limit, max(-limit, value))


class HuskyActuationControl(object):

    def __init__(self, target_type='static', longitudinal_p=4, lateral_p=5, angle_p=.4):
        # setup ROS node and topics
        # rospy.init_node('navigation_control')
        self.pub_cmd = rospy.Publisher('/husky/cmd_vel', Twist, queue_size=1)
        self.twist_msg = Twist()

        self.rate = rospy.Rate(20)
        self.tangential_limit
        self.angular_limit
        # if target_type is 'dynamic':
        # self.longitudinal_pid = PID.PID(longitudinal_p, 0, 0)  # 1.6, 1.70484816196, 1.00106666667
        # self.lateral_pid = PID.PID(lateral_p, 0, 0)  # 2.0, 2.87999906616, 0.925926226157
        # self.angle_pid = PID.PID(angle_p, 0, 0)  # 2.8, 4.26087605796, 1.22666479747
        # elif target_type is 'static':
        # self.longitudinal_pid = PID.PID(longitudinal_p / 2, 0, 0)  # 1.6, 1.70484816196, 1.00106666667
        # self.lateral_pid = PID.PID(lateral_p / 2, 0, 0)  # 2.0, 2.87999906616, 0.925926226157
        # self.angle_pid = PID.PID(angle_p / 2, 0, 0)  # 2.8, 4.26087605796,
        # 1.22666479747

    def setRate(self, rate):
        self.rate = rospy.Rate(20)

    # def calculateError(self, actual_pos, reference_pos):
    # reference_x_temp, reference_y_temp = self.path.getPosition()
    # reference_angle = np.arctan2(reference_y_temp - self.reference_y, reference_x_temp - self.reference_x)
    # self.reference_x = reference_x_temp
    # self.reference_y = reference_y_temp
    #     self.diff_x = reference_pos.x - actual_pos.x
    #     self.diff_y = reference_pos.y - actual_pos.y
    #     angle_error = reference_pos.theta - actual_pos.theta
    #     self.angle_error_wrapped = (angle_error + np.pi) % (2 * np.pi) - np.pi
    #     self.longitudinal_error = np.cos(actual_pos.theta) * self.diff_x + np.sin(actual_pos.theta) * self.diff_y
    #     self.lateral_error = np.cos(actual_pos.theta) * self.diff_y - np.sin(actual_pos.theta) * self.diff_x
    #     return [[self.longitudinal_error, self.lateral_error, self.reference_x, self.reference_y, actual_pos.x, actual_pos.y, actual_pos.theta, time()]]
    # def stop(self):

    # def goToDynamicTarget(self, error_minimizer=True):
    #     limit_1 = 1
    #     limit_2 = 2
    # lateral_error_linearized = np.arcsin(self.lateral_error / np.hypot(self.diff_x, self.diff_y))
    # if error_minimizer is True:
    # vel = max(.3 + abs(self.lateral_error) * (-2.5), 0)
    # self.path.setVelocity(vel)
    #     feedback_linear = self.longitudinal_pid.calculate(self.longitudinal_error)
    #     feedback_angular = self.lateral_pid.calculate(self.lateral_error) + self.angle_pid.calculate(self.angle_error_wrapped)
    #     feedback_linear = min(limit_1, max(feedback_linear, -limit_1))
    #     feedback_angular = min(limit_2, max(feedback_angular, -limit_2))
    #     self.pub_cmd.publish(self.twist_msg)
    #     self.rate.sleep()
    #     return feedback_linear, feedback_angular

    def actuate(self, tangential_velocity, angular_velocity):
        if rospy.is_shutdown():
            return
        self.twist_msg.linear.x = tangential_velocity
        self.twist_msg.angular.z = angular_velocity
        if tangential_velocity+angular_velocity is nan_var:
            self.pub_cmd.publish(stop_msg)
        else:
            self.pub_cmd.publish(self.twist_msg)
        self.rate.sleep()

    def goToStaticTarget(self):
        bound = .05
        counter = 0
        while (abs(self.longitudinal_error) > bound or abs(self.lateral_error) > bound) and counter < 300:
            while abs(self.lateral_error) > bound and counter < 300:
                print 'fixing lat'
                self.pose_msg.x, self.pose_msg.y, self.pose_msg.theta = self.connection.getStates()
                errors = move_to_point.calculateError(self.pose_msg)
                feedback_linear, feedback_angular = move_to_point.actuationCalculation(
                    error_minimizer=False)
                self.twist_msg.linear.x = 0
                print errors
                self.twist_msg.angular.z = feedback_angular
                self.pub_cmd.publish(self.twist_msg)
                self.rate.sleep()
            # errors = move_to_point.calculateError(self.pose_msg)
            while abs(self.longitudinal_error) > bound and counter < 300:
                print 'fixing long'
                self.pose_msg.x, self.pose_msg.y, self.pose_msg.theta = self.connection.getStates()
                errors = move_to_point.calculateError(self.pose_msg)
                feedback_linear, feedback_angular = move_to_point.actuationCalculation(
                    error_minimizer=False)
                self.twist_msg.linear.x = feedback_linear
                self.twist_msg.angular.z = 0
                self.pub_cmd.publish(self.twist_msg)
                self.rate.sleep()

    def faceDirection(self):
        pass
