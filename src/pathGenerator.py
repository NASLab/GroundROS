import numpy as np
from scipy.interpolate import interp1d
from time import time
import sys

# Generate a path


class PathGenerator():

    def __init__(self, path_type='circle', speed=.1):
        self.time = time()
        self.travel_distance = 0
        self.__path_type = path_type
        self.speed = speed
        self.x_array = []
        self.y_array = []
        self.distance_array = []
        self.total_distance = 1
        self.radial_boundry = 1
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
            sys.exit('PathGeneration(): Wrong path type. Available options are "circle", "infinity", and "square".')
        # self.x = self.x_array[0]
        # self.y = self.y_array[0]

    def setVelocity(self, velocity):
        if self.path_type is not 'point':
            self.speed = velocity

    def getPosition(self):
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
        angle_steps = np.linspace(0, 2 * np.pi, num=101)
        self.x_array = self.radial_boundry * np.cos(angle_steps)
        self.y_array = self.radial_boundry * np.sin(angle_steps)
        self.distance_array = np.linspace(0, self.radial_boundry * 2 * np.pi, num=101)
        self.total_distance = self.radial_boundry * 2 * np.pi
        self.x_function = interp1d(self.distance_array, self.x_array, kind='quadratic')
        self.y_function = interp1d(self.distance_array, self.y_array, kind='quadratic')

    def __generateInfinity(self):
        angle_steps = np.linspace(0, 2 * np.pi, num=101)
        self.x_array = self.radial_boundry * np.cos(angle_steps)
        self.y_array = self.radial_boundry * np.sin(2 * angle_steps)
        x_steps = np.diff(self.x_array)
        y_steps = np.diff(self.y_array)
        hypot_steps = np.hypot(x_steps, y_steps)
        self.distance_array = np.insert(np.cumsum(hypot_steps), 0, 0)
        self.total_distance = self.distance_array[-1]
        self.x_function = interp1d(self.distance_array, self.x_array, kind='cubic')
        self.y_function = interp1d(self.distance_array, self.y_array, kind='cubic')

    def __generateSquare(self):
        self.x_function = interp1d(self.distance_array, self.x_array)
        self.y_function = interp1d(self.distance_array, self.y_array)

    def __generateLine(self, slope=np.pi/6):
        self.x_array = np.cos(slope) * np.linspace(-self.radial_boundry, self.radial_boundry, num=101)
        self.y_array = np.sin(slope) * np.linspace(-self.radial_boundry, self.radial_boundry, num=101)
        self.distance_array = np.linspace(0, 2 * self.radial_boundry, num=101)
        self.total_distance = self.radial_boundry * 2
        self.x_function = lambda x: x*np.cos(slope) - np.cos(slope)
        self.y_function = lambda x: x*np.sin(slope) - np.sin(slope)

    def __generateBrokenLine(self, slope_1=0, slope_2=np.pi / 6):
        x_array_1 = np.cos(slope_1) * np.linspace(-self.radial_boundry, 0, num=51)
        x_array_2 = np.cos(slope_2) * np.linspace(0, self.radial_boundry, num=51)
        self.x_array = np.concatenate([x_array_1, x_array_2[1:]])
        y_array_1 = np.sin(slope_1) * np.linspace(-self.radial_boundry, 0, num=51)
        y_array_2 = np.sin(slope_2) * np.linspace(0, self.radial_boundry, num=51)
        self.y_array = np.concatenate([y_array_1, y_array_2[1:]])
        self.distance_array = np.linspace(0, 2 * self.radial_boundry, num=101)
        self.total_distance = self.radial_boundry * 2
        self.x_function = interp1d(self.distance_array, self.x_array, kind='linear')
        self.y_function = interp1d(self.distance_array, self.y_array, kind='linear')

    def __twoLines(self):
        self.x_array = np.linspace(-self.radial_boundry, self.radial_boundry, num=101)
        self.total_distance = self.radial_boundry * 2
        self.y_array = np.concatenate([[-self.radial_boundry / 4.0 for i in range(51)], [self.radial_boundry / 4.0 for i in range(50)]])
        self.distance_array = np.linspace(0, 2 * self.radial_boundry, num=101)
        self.x_function = interp1d(self.distance_array, self.x_array, kind='linear')
        self.y_function = interp1d(self.distance_array, self.y_array, kind='linear')

    def __generatePoint(self):
        self.x_function = lambda x: 0
        self.y_function = lambda x: 0

    def printPos(self):
        print self.x
        print self.y
        print "--------------"
