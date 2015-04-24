import numpy as np
from scipy.interpolate import interp1d
# import matplotlib.pyplot as plt
from time import time
import sys

# Generate a set of paths


class PathGenerator():

    def __init__(self, path_type='infinity', speed=.05):
        self.time = time()
        self.travel_distance = 0
        self.__path_type = path_type
        self.speed = speed
        self.x_array = []
        self.y_array = []
        self.distance_array = []
        self.total_distance = 0
        self.x_limit = 2
        self.y_limit = 1
        if path_type == 'circle':
            self.__generateCircle
        elif path_type == 'infinity':
            self.__generateInfinity()
        elif path_type == 'square':
            self.__generateSquare
        else:
            sys.exit('PathGeneration(): Wrong path type. Available options are "circle", "infinity", and "square".')
        self.x = self.x_array[0]
        self.y = self.y_array[0]

    def getPosition(self):
        time_now = time()
        self.travel_distance = (self.travel_distance + self.speed * (time_now - self.time)) % self.total_distance
        self.time = time()
        self.x = self.x_function(self.travel_distance)
        self.y = self.y_function(self.travel_distance)
        return self.x, self.y

    def __generateCircle(self):
        angle_steps = np.linspace(0, 2 * np.pi, num=101)
        self.x_array = self.x_limit * np.cos(angle_steps)
        self.y_array = self.x_limit * np.sin(angle_steps)
        self.distance_array = np.linspace(0, self.x_limit * 2 * np.pi, num=101)
        self.total_distance = self.x_limit * 2 * np.pi
        self.x_function = interp1d(self.distance_array, self.x_array, kind='quadratic')
        self.y_function = interp1d(self.distance_array, self.y_array, kind='quadratic')
        pass

    def __generateInfinity(self):
        angle_steps = np.linspace(0, 2 * np.pi, num=101)
        self.x_array = self.x_limit * np.cos(angle_steps)
        self.y_array = self.x_limit * np.sin(2 * angle_steps)
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
        pass

    def printPos(self):
        print self.x
        print self.y
        print "--------------"

    # def plotPath(self):
    #     plt.plot(self.x_array, self.y_array, self.x, self.y, 'ro')
    #     plt.show()

        # def update_line(num, data, line):
        #     line.set_data(data[..., :num])
        #     return line,

        # fig1 = plt.figure()

        # data = np.array((x, y))
        # l, = plt.plot([], [], 'r-')
        # plt.xlim(-1.5, 1.5)
        # plt.ylim(-1.5, 1.5)
        # plt.xlabel('X')
        # plt.ylabel('Y')
        # plt.title('position')
        # plt.grid()
        # line_ani = animation.FuncAnimation(fig1, update_line,
        #                                    len(data[0]), fargs=(data, l), interval=200, blit=True)

        # plt.show()


# def main():
#     pth = PathGenerator()
#     pth.plotPath()
#     for t in range(1, 100):
#         pth.getPosition()


# if __name__ == "__main__":
#     main()
