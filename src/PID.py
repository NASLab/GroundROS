#!usr/bin/env python

from time import time


class PID(object):

    """docstring for ClassName"""

    def __init__(self, kp, ki, kd):
        self.time = time()
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_error = 0
        self.error_previous = 0

    def calculate(self, error):
        time_now = time()
        elapsed_time = time_now - self.time
        derivative_error = (error - self.error_previous) / elapsed_time
        control_input = self.kp * error + self.ki * self.integral_error +\
            self.kd * derivative_error
        self.integral_error = self.integral_error + error * elapsed_time
        self.time = time_now
        self.error_previous = error

        return control_input
