from numpy import pi, sin, cos, append, save
from rotation import rotate2DimFrame, wrapAnglePi
from control_system import Proportional
from datetime import datetime  # remove later
from time import time


class PlanarTracker(object):

    def __init__(self, actuate_function, localization_function=[]):
        self.actuate = actuate_function
        self.locate = localization_function

        self.long_ultimate_gain = 8
        self.long_ultimate_period = 999  # should be determined later
        self.lateral_ultimate_gain = 10
        self.lateral_ultimate_period = 999  # should be determined later
        self.angular_ultimate_gain = 8
        self.angular_ultimate_period = 999  # should be determined later

        self.long_control = Proportional()
        self.lateral_control = Proportional()
        self.angular_control = Proportional()

        self.dynamic_long_control = Proportional()
        self.dynamic_lateral_control = Proportional()
        self.dynamic_long_control.setGain(self.long_ultimate_gain / 4)
        self.dynamic_lateral_control.setGain(self.lateral_ultimate_gain / 2)

        self.logger = [[0, 0, 0, 0, 0, 0, 0, 0]]

    def goToPoint(self, x, y):
        print 'Going to static point.'
        bound = .01
        self.long_control.setGain(self.long_ultimate_gain / 4)
        self.lateral_control.setGain(self.lateral_ultimate_gain / 2)

        def getError():
            x_actual, y_actual, theta_actual = self.locate()
            x_error = x - x_actual
            y_error = y - y_actual
            long_error, lateral_error = rotate2DimFrame(x_error, y_error, theta_actual)
            return long_error, lateral_error

        long_error, lateral_error = getError()
        while abs(long_error) > bound or abs(lateral_error) > bound:
            feedback_linear = self.long_control.controllerOutput(long_error)
            feedback_angular = self.lateral_control.controllerOutput(lateral_error)
            self.actuate(feedback_linear, feedback_angular)
            long_error, lateral_error = getError()
        print 'Longitutional error:', long_error, 'm | Lateral error:', lateral_error, 'm'
        # pass

    def moveTowardsDynamicPoint(self, distance, theta):
        long_error = distance * cos(theta)
        lateral_error = distance * sin(theta)
        feedback_linear = self.dynamic_long_control.controllerOutput(long_error)
        feedback_angular = self.dynamic_lateral_control.controllerOutput(lateral_error)
        self.actuate(feedback_linear, feedback_angular)

    def faceDirection(self, theta):
        print 'Facing direction.'
        bound = .02
        self.angular_control.setGain(self.angular_ultimate_gain / 2)
        theta_error = wrapAnglePi(theta - self.locate()[2])
        while abs(theta_error) > bound:
            feedback_angular = self.angular_control.controllerOutput(theta_error)
            self.actuate(0, feedback_angular)
            theta_error = wrapAnglePi(theta - self.locate()[2])
        print 'Angular error:', theta_error, 'radians =', theta_error * 180 / pi, 'degrees'

    def followTrajectory(self, trajectory):
        print 'Following Trajectory.'
        self.long_control.setGain(self.long_ultimate_gain / 2)
        self.lateral_control.setGain(self.lateral_ultimate_gain / 2)
        reference_pos = trajectory.getPosition()
        while reference_pos == reference_pos:  # check to see if x_reference is not NaN
            x_actual, y_actual, theta_actual = self.locate()
            x_reference, y_reference = reference_pos
            x_error = x_reference - x_actual
            y_error = y_reference - y_actual
            long_error, lateral_error = rotate2DimFrame(x_error, y_error, theta_actual)

            feedback_linear = self.long_control.controllerOutput(long_error)
            feedback_angular = self.lateral_control.controllerOutput(lateral_error)

            self.actuate(feedback_linear, feedback_angular)

            reference_pos = trajectory.getPosition()
            self.logger = append(self.logger, [[long_error, lateral_error,
                                                x_reference, y_reference,
                                                x_actual, y_actual,
                                                theta_actual, time()]], axis=0)

        save('/home/administrator/barzin_catkin_ws/src/path_tracking/scripts/experimental_results/' + str(datetime.now()) + ' followTrajectory', self.logger)
        self.logger = [[0, 0, 0, 0, 0, 0, 0, 0]]
        print 'Reached end of trajectory.'

    def followPath(self, path):
        'Following path.'
        self.long_control.setGain(self.long_ultimate_gain / 4)
        self.lateral_control.setGain(self.lateral_ultimate_gain / 2)
        reference_pos = path.getPosition()
        while reference_pos == reference_pos:  # check to see if x_reference is not NaN
            x_actual, y_actual, theta_actual = self.locate()
            x_reference, y_reference = reference_pos
            x_error = x_reference - x_actual
            y_error = y_reference - y_actual
            long_error, lateral_error = rotate2DimFrame(x_error, y_error, theta_actual)
            vel = max(1 + abs(lateral_error**2) * (-100), .05)
            path.setVelocity(vel)
            feedback_linear = self.long_control.controllerOutput(long_error)
            feedback_angular = self.lateral_control.controllerOutput(lateral_error)

            self.actuate(feedback_linear, feedback_angular)

            reference_pos = path.getPosition()
            self.logger = append(self.logger, [[long_error, lateral_error,
                                                x_reference, y_reference,
                                                x_actual, y_actual,
                                                theta_actual, time()]], axis=0)

        save('/home/administrator/barzin_catkin_ws/src/path_tracking/scripts/experimental_results/' + str(datetime.now()) + ' followPath', self.logger)
        self.logger = [[0, 0, 0, 0, 0, 0, 0, 0]]
        print 'Reached end of path.'

    def followLoop(self):
        self.long_control.setGain(self.long_ultimate_gain / 2)
        self.lateral_control.setGain(self.lateral_ultimate_gain / 2)
        pass
