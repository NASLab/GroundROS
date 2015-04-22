import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from time import time

# Generate a set of paths


class PathGenerator():

    def __init__(self, path_type='circle', speed=.1):
        # self.start_time = time.time()
        self.time = self.start_time
        self.travel_distance = 0
        self.__path_type = path_type
        self.speed = speed
        self.x_array
        self.y_array
        self.distance_array
        self.total_distance
        self.x = 0
        self.y = 0
        self.x_limit = 1
        self.y_limit = 1
        if True:
            self.__generateCircle()
        if True:
            self.__generateInfinity()

    def getPosition(self):
        self.travel_distance = (
            self.travel_distance + self.speed * (time() - self.time)) % self.total_distance
        x = self.x_function(self.travel_distance)
        y = self.y_function(self.travel_distance)

        return x, y

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
        self.x_array = self.x_limit * np.cos(2*angle_steps)
        self.y_array = self.x_limit * np.sin(angle_steps)
        x_steps = np.diff(self.x_array)
        y_steps = np.diff(self.y_array)
        hypot_steps = np.(self.x_steps,self.y_steps)
        self.distance_array = [0, np.cumsum(hypot_steps)]
        self.x_function = interp1d(self.distance_array, self.x_array, kind='cubic')
        self.y_function = interp1d(self.distance_array, self.y_array, kind='cubic')
        pass

    def __generateSquare(self):
        self.x_function = interp1d(self.distance_array, self.x_array)
        self.y_function = interp1d(self.distance_array, self.y_array)
        pass

    def printPos(self):
        print self.x
        print self.y
        print "--------------"

    def circlePath(self):
        self.elapsed_time = time - self.start_time
        self.x = self.x_range / 2 * np.cos(self.elapsed_time)
        self.y = self.y_range / 2 * np.sin(self.elapsed_time)
        return self.x, self.y
        self.time = time

    def infinityPath(self, time):
        self.x = np.cos(time) / (np.power(np.sin(time), 2) + 1)
        self.y = np.cos(time) * np.sin(time) / (np.power(np.sin(time), 2) + 1)
        self.time = time

    def squarePath(self, time):
        self.x = np.cos(
            time) / np.maximum(np.abs(np.sin(time)), np.abs(np.cos(time)))
        self.y = np.sin(
            time) / np.maximum(np.abs(np.sin(time)), np.abs(np.cos(time)))
        self.time = time

    def plotPath(self, x, y):

        def update_line(num, data, line):
            line.set_data(data[..., :num])
            return line,

        fig1 = plt.figure()

        data = np.array((x, y))
        l, = plt.plot([], [], 'r-')
        plt.xlim(-1.5, 1.5)
        plt.ylim(-1.5, 1.5)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('position')
        plt.grid()
        line_ani = animation.FuncAnimation(fig1, update_line,
                                           len(data[0]), fargs=(data, l), interval=200, blit=True)

        plt.show()


def main():
    path = PathGenerator(0, 1)
    xvals = []
    yvals = []
    for t in np.arange(0, 3.142 * 2, 0.1):
        path.squarePath(t)
        xvals.append(path.x)
        yvals.append(path.y)
    path.plotPath(xvals, yvals)


if __name__ == "__main__":
    main()
