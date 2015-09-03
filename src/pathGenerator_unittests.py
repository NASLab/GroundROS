import unittest
import sys
# sys.path.insert(0, '../src')
import pathGenerator as pthgen
import matplotlib.pyplot as plt
import time

import pylab 
pylab.ion()

def plotPath(pthgen):
    pthgen.getPosition()
    plt.plot(pthgen.x_array, pthgen.y_array)

    for t in range(1,100):
        ln, = plt.plot(pthgen.x,pthgen.y,'ro')
        plt.draw()
        # plt.draw()
        # pthgen.handle = pthgen.position.pop(0)
        time.sleep(.1)
        # pthgen.handle.remove()
        ln.remove()
        pthgen.getPosition()
        # print pthgen.x

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


pth = pthgen.PathGenerator(path_type='circle', speed=.3)
plotPath(pth)

pth = pthgen.PathGenerator(path_type='infinity', speed=.3)
plotPath(pth)

pth = pthgen.PathGenerator(path_type='circle', speed=.3)
plotPath(pth)