import PID
import numpy as np
from time import time


class ActuationControl(object):

    def __init__(self, path, longitudinal_p=4, lateral_p=5, angle_p=.4):
        self.longitudinal_pid = PID.PID(longitudinal_p, 0, 0)  # 1.6, 1.70484816196, 1.00106666667
        self.lateral_pid = PID.PID(lateral_p, 0, 0)  # 2.0, 2.87999906616, 0.925926226157
        self.angle_pid = PID.PID(angle_p, 0, 0)  # 2.8, 4.26087605796, 1.22666479747
        self.reference_x, self.reference_y = path.getPosition()
        self.path = path

    def calculateError(self, actual_pos):
        reference_x_temp, reference_y_temp = self.path.getPosition()
        reference_angle = np.arctan2(reference_y_temp - self.reference_y, reference_x_temp - self.reference_x)
        angle_error = reference_angle - actual_pos.theta
        self.angle_error_wrapped = (angle_error + np.pi) % (2 * np.pi) - np.pi
        self.reference_x = reference_x_temp
        self.reference_y = reference_y_temp
        self.diff_x = self.reference_x - actual_pos.x / 1000
        self.diff_y = self.reference_y - actual_pos.y / 1000
        self.longitudinal_error = np.cos(actual_pos.theta) * self.diff_x + np.sin(actual_pos.theta) * self.diff_y
        self.lateral_error = np.cos(actual_pos.theta) * self.diff_y - np.sin(actual_pos.theta) * self.diff_x
        return [[self.longitudinal_error, self.lateral_error, self.reference_x, self.reference_y, actual_pos.x, actual_pos.y, actual_pos.theta, time()]]
    # def stop(self):

    def actuationCalculation(self,error_minimizer=True, limit_1=1, limit_2=2):
        # lateral_error_linearized = np.arcsin(self.lateral_error / np.hypot(self.diff_x, self.diff_y))
        if error_minimizer is True:
            vel = max(.3 + abs(self.lateral_error) * (-2.5), .05)
            self.path.setVelocity(vel)
        feedback_linear = self.longitudinal_pid.calculate(self.longitudinal_error)
        feedback_angular = self.lateral_pid.calculate(self.lateral_error) + self.angle_pid.calculate(self.angle_error_wrapped)
        feedback_linear = min(limit_1, max(feedback_linear, -limit_1))
        feedback_angular = min(limit_2, max(feedback_angular, -limit_2))
        return feedback_linear, feedback_angular
