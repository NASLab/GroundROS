import numpy as np
# import ROS2DimActuationControl as AC
from rotation import rotate2DimFrame
import ControlSystem as ctrl_sys

print ctrl_sys


class Tracker(object):

    def __init__(self):
        self.ctrl = ctrl_sys.ControlSystem()
        self.actuate = AC.ROS2DimActuate()
        pass

    def goToPoint(self):
        pass

    def faceDirection(self):
        pass

    def followTrajectory(self):
        pass

    def followPath(self):
        pass

    def followLoop(self):
        pass
