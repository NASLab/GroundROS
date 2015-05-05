#!usr/bin/env python
from time import time


class PID(object):

    """docstring for ClassName"""

    def __init__(self, kp, ki, kd):
        try:
            self.time = time()
            self.kp = kp
            self.ki = ki
            self.kd = kd
            self.integral_error = 0
            self.error_previous = 0
        except Exception, e:
            raise

    def calculate(self, error):
        try:
            time_now = time()
            time_laps = time_now - self.time
            derivative_error = (error - self.error_previous) / time_laps
            self.integral_error = self.integral_error + error * time_laps
            control_input = self.kp * error + self.ki * self.integral_error +\
                self.kd * derivative_error
            self.time = time_now
            self.error_previous = error

            return control_input
        except Exception, e:
            raise
