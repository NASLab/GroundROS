# import numpy as np
from numpy import pi, sin, cos, insert, cumsum, linspace, diff, hypot,arctan2
from scipy.interpolate import interp1d
from time import time

# Generate a path

nan_var = float('nan')


class PathError(Exception):

    def __init__(self, message, errors=-1):

        # Call the base class constructor with the parameters it needs
        super(PathError, self).__init__(message)


class PathGenerator():

    def __init__(self, path_type='circle', speed=.1):
        self.time = time()
        self.travel_distance = 0
        self.path_type = path_type
        self.speed = speed
        self.x_array = []
        self.y_array = []
        self.distance_array = []
        self.total_distance = 1
        self.radial_boundry = 2.3
        if path_type == 'circle':
            self.__generateCircle()
        elif path_type == 'infinity':
            self.__generateInfinity()
        elif path_type == 'broken_line':
            self.__generateBrokenLine()
        elif path_type == 'two_lines':
            self.__twoLines()
        elif path_type == 'square':
            self.__generateSquare()
        elif path_type == 'line':
            self.__generateLine()
        elif path_type == 'point':
            self.speed = 0
            self.__generatePoint()
        else:
            raise PathError('PathGeneration(): Wrong path type. Available options are "circle", "infinity", and "square".')
        # self.x = self.x_array[0]
        # self.y = self.y_array[0]

    def setVelocity(self, velocity):
        if self.path_type is not 'point':
            self.speed = velocity

    def startNow(self):
        self.time = time()
        self.travel_distance = 0
        self.x = self.x_function(0)
        self.y = self.y_function(0)

    def getEnd(self):
        x = self.x_function(self.distance_array[-1])
        y = self.y_function(self.distance_array[-1])
        return x,y

    def getBeginning(self):
        x = self.x_function(0)
        y = self.y_function(0)
        return x,y

    def getPosition(self):
        prev_x = self.x
        prev_y = self.y
        time_now = time()
        # if time_now-self.time>self.time_array[-1]:
        #     return nan_var
        temp_travel_dist = self.travel_distance
        self.travel_distance = self.travel_distance + self.speed * (time_now - self.time)
        # print self.travel_distance-temp_travel_dist,self.travel_distance
        # print 'in getPosition',time_now-self.time, self.time_array[-1]
        if self.travel_distance > self.total_distance:
            # raise PathError('Reached end of path')
            return nan_var
        else:
            self.time = time()
            self.x = self.x_function(self.travel_distance)
            self.y = self.y_function(self.travel_distance)
            self.theta = arctan2(self.y-prev_y,self.x-prev_x)
            return self.x, self.y,self.theta

    def getLoopPosition(self):
        time_now = time()
        self.travel_distance = (self.travel_distance + self.speed * (time_now - self.time)) % self.total_distance
        self.time = time()
        self.x = self.x_function(self.travel_distance)
        self.y = self.y_function(self.travel_distance)
        return self.x, self.y

    def setPoint(self, x, y):
        self.speed = 0
        self.x_function = lambda x: x
        self.y_function = lambda x: y

    def __generateCircle(self):
        angle_steps = linspace(0, 2 * pi, num=101)
        self.x_array = self.radial_boundry * cos(angle_steps)
        self.y_array = self.radial_boundry * sin(angle_steps)
        self.distance_array = linspace(0, self.radial_boundry * 2 * pi, num=101)
        self.total_distance = self.radial_boundry * 2 * pi
        self.x_function = interp1d(self.distance_array, self.x_array, kind='quadratic')
        self.y_function = interp1d(self.distance_array, self.y_array, kind='quadratic')

    def __generateInfinity(self):
        angle_steps = linspace(0, 2 * pi, num=101)
        self.x_array = self.radial_boundry * cos(angle_steps)
        self.y_array = self.radial_boundry * sin(2 * angle_steps)
        x_steps = diff(self.x_array)
        y_steps = diff(self.y_array)
        hypot_steps = hypot(x_steps, y_steps)
        self.distance_array = insert(cumsum(hypot_steps), 0, 0)
        self.total_distance = self.distance_array[-1]
        self.x_function = interp1d(self.distance_array, self.x_array, kind='cubic')
        self.y_function = interp1d(self.distance_array, self.y_array, kind='cubic')

    def __generateSquare(self):
        self.x_function = interp1d(self.distance_array, self.x_array)
        self.y_function = interp1d(self.distance_array, self.y_array)

    def __generateLine(self, slope=pi / 6):
        self.x_array = cos(slope) * linspace(-self.radial_boundry, self.radial_boundry, num=101)
        self.y_array = sin(slope) * linspace(-self.radial_boundry, self.radial_boundry, num=101)
        self.distance_array = linspace(0, 2 * self.radial_boundry, num=101)
        self.total_distance = self.radial_boundry * 2
        self.x_function = lambda x: x * cos(slope) - cos(slope)
        self.y_function = lambda x: x * sin(slope) - sin(slope)

    def __generateBrokenLine(self, slope_1=0, slope_2=pi / 2):
        x_array_1 = cos(slope_1) * linspace(-self.radial_boundry, 0, num=51)
        x_array_2 = cos(slope_2) * linspace(0, self.radial_boundry, num=51)
        self.x_array = [x_array_1] + [x_array_2[1:]]
        y_array_1 = sin(slope_1) * linspace(-self.radial_boundry, 0, num=51)
        y_array_2 = sin(slope_2) * linspace(0, self.radial_boundry, num=51)
        self.y_array = [y_array_1] + [y_array_2[1:]]
        self.distance_array = linspace(0, 2 * self.radial_boundry, num=101)
        self.total_distance = self.radial_boundry * 2
        self.x_function = interp1d(self.distance_array, self.x_array, kind='linear')
        self.y_function = interp1d(self.distance_array, self.y_array, kind='linear')

    def __twoLines(self):
        self.y_array = linspace(-self.radial_boundry, self.radial_boundry, num=101)
        self.total_distance = self.radial_boundry * 2
        self.x_array = [-self.radial_boundry / 20.0 for i in range(31)] + [self.radial_boundry / 20.0 for i in range(70)]
        self.distance_array = linspace(0, 2 * self.radial_boundry, num=101)
        self.x_function = interp1d(self.distance_array, self.x_array, kind='nearest')
        self.y_function = interp1d(self.distance_array, self.y_array, kind='linear')

    def __generatePoint(self):
        self.x_function = lambda x: 0
        self.y_function = lambda x: 0

    def printPos(self):
        print self.x
        print self.y
        print "--------------"
