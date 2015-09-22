from time import time

nan_var = float('nan')


class ControlSystem(object):

    def __init__(self, method='bangbang'):
        self.method = method
        self.time = time()
        self.kp = nan_var
        self.ki = nan_var
        self.kd = nan_var
        self.integral_error = 0
        self.error_previous = 0

    def bangbang(self, error):
        if error > 0:
            return True
        else:
            return False

    def proportional(self, error):
        signal = self.kp * error
        return signal

    def PID(self, error):
        time_now = time()
        elapsed_time = time_now - self.time
        derivative_error = (error - self.error_previous) / elapsed_time
        signal = self.kp * error + self.ki * self.integral_error +\
            self.kd * derivative_error
        self.integral_error = self.integral_error + error * elapsed_time
        self.time = time_now
        self.error_previous = error
        return signal

    def setGain(self, kp=nan_var, ki=nan_var, kd=nan_var):
        if kp is not nan_var:
            self.kp = kp

        if ki is not nan_var:
            self.ki = ki

        if kd is not nan_var:
            self.kd = kd