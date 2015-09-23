import numpy as np
import HuskyActuationControl as AC


class Tracker(object):

    def __init__(self, longitutnal_gain=4, lateral_gain=5, angular_gain=4):
        self.static_target = 
        self.dynamic_target = 
        self.navigate_to_static_target = 
        self.navigate_to_dynamic_target = 
        pass

    def goToStaticPoint(self, x, y, theta=float('nan')):
        errors = move_to_point.calculateError(self.pose_msg)
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

    def faceStaticDirection(self, theta):
        errors = move_to_point.calculateError(self.pose_msg)
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
        pass

    def followPath(self, path, initialization=True):
        pass

    def followTrajectory(self, path, initialization=True):
        pass

    def followLoop(self,path, initialization=True):
        pass
