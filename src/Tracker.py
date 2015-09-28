import numpy as np
# import ROS2DimActuationControl as AC
from rotation import rotate2DimFrame
import ControlSystem as ctrl_sys

# print ctrl_sys


class PlanarTracker(object):

    def __init__(self, actuate_function, localization_function):
        self.actuate = actuate_function
        self.locate = localization_function
        # self.actuate = AC.ROS2DimActuate()
        self.long_ultimate_gain = 8
        self.long_ultimate_period = 999  # should be determined later
        self.lateral_ultimate_gain = 10
        self.lateral_ultimate_period = 999  # should be determined later
        self.angular_ultimate_gain = 1
        self.angular_ultimate_period = 999  # should be determined later
        self.long_control = ctrl_sys.ControlSystem()
        self.lateral_control = ctrl_sys.ControlSystem()
        self.angular_control = ctrl_sys.ControlSystem()
        pass

    def goToPoint(self, x, y):
        bound = .05
        self.long_control.setGain(self.long_ultimate_gain / 4)
        self.lateral_control.setGain(self.lateral_ultimate_gain / 2)
        actual_location = self.locate()
        def getError():
            x_error = x - actual_location.x
            y_error = y - actual_location.y
            long_error, lateral_error = rotate2DimFrame(x_error,y_error,actual_location.theta)
            return long_error,lateral_error
        long_error, lateral_error = getError()
        while abs(long_error)>bound and abs(lateral_error)>bound:
            feedback_linear = self.long_control.proportional(long_error)
            feedback_angular = self.lateral_control.proportional(lateral_error)
            self.actuate(feedback_linear,feedback_angular)
            long_error, lateral_error = getError()
        # pass

    def faceDirection(self):
        self.angular_control.setGain(self.angular_ultimate_gain / 4)
        pass

    def followTrajectory(self):
        self.long_control.setGain(self.long_ultimate_gain / 2)
        self.lateral_control.setGain(self.lateral_ultimate_gain / 2)
        pass

    def followPath(self):
        self.long_control.setGain(self.long_ultimate_gain / 4)
        self.lateral_control.setGain(self.lateral_ultimate_gain / 2)
        pass

    def followLoop(self):
        self.long_control.setGain(self.long_ultimate_gain / 2)
        self.lateral_control.setGain(self.lateral_ultimate_gain / 2)
        pass
