# python experimental tests for Husky

import numpy as np
import glob
import matplotlib.pyplot as plt


for file in glob.glob("*.npy"):
    data = np.load(file)[5:, :]
    print file,
    error_long = data[:, 0]
    error_lat = data[:, 1]
    ref_x = [value-data[0,2] for value in data[:, 2]]
    # print ref_x[:30]
    ref_y = [value-data[0,3] for value in data[:, 3]]
    pos_x = [value-data[0,4] for value in data[:, 4]][0::20]
    pos_y = [value-data[0,5] for value in data[:, 5]][0::20]
    pos_theta = data[:, 6]
    time = data[:, 7] - data[0, 7]
    vel = data[:,8]
    # plt.plot(ref_x, ref_y, 'ro')
    # plt.gca().set_aspect('equal', adjustable='box')
    f0 = plt.figure()
    f1 = plt.figure()
    f2 = plt.figure()
    ax0 = f0.add_subplot(111)
    ax0.plot(ref_x, ref_y, '--', lw=3,label='Reference Trajectory')
    ax0.plot(pos_x[0], pos_y[0], 'mo',markersize = 10,label='Start Point')
    ax0.plot(pos_x, pos_y, 'ro',label='Robot Trajectory')
    ax0.legend()
    ax0.axis('equal')

    ax1 = f1.add_subplot(111)
    ax1.plot(time, error_long,lw=3,label='Longitudinal Error')
    ax1.plot(time, error_lat,lw=3,label='Lateral Error')
    ax1.legend()

    ax2 = f2.add_subplot(111)
    ax2.plot(time,vel,lw=3,label='Velocity Profile')
    ax2.legend()

    plt.draw()
    plt.pause(.1)  # <-------
    raw_input("<Hit Enter To Close>")
    plt.close(f0)
    plt.close(f1)
