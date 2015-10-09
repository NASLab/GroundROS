import sys
sys.path.insert(0, '..')
import path_generator as pthgen
import matplotlib.pyplot as plt
import time
import numpy as np
import pylab
pylab.ion()


def plotPath(pthgen):
    fig = plt.figure()
    pthgen.getPosition()
    plt.plot(pthgen.x_array, pthgen.y_array)
    plt.xlim(-1.1, 1.1)
    plt.ylim(-1.1, 1.1)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('position')
    plt.axes().set_aspect(1)

    for t in range(1, 100):
        ln, = plt.plot(pthgen.x, pthgen.y, 'ro')
        plt.draw()
        time.sleep(.1)
        ln.remove()
        pthgen.setVelocity(np.sin(t*np.pi/50)/1.8)
        pthgen.getPosition()
    plt.close(fig)


pth = pthgen.PathGenerator(path_type='circle', speed=.3)
plotPath(pth)

pth = pthgen.PathGenerator(path_type='infinity', speed=.3)
plotPath(pth)

pth = pthgen.PathGenerator(path_type='broken_line', speed=.3)
plotPath(pth)

pth = pthgen.PathGenerator(path_type='two_lines', speed=.3)
plotPath(pth)

pth = pthgen.PathGenerator(path_type='line', speed=.3)
plotPath(pth)