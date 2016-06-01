# python experimental tests for Husky

import numpy as np
import glob
import matplotlib.pyplot as plt

# fig_size = 
f0 = plt.figure(1,figsize = (9,20))
# f1 = plt.figure()
f2 = plt.figure(2,figsize=(9,9))
iteration = 0
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
    if iteration is 0:
        ax0 = f0.add_subplot(4,2,(1,3))
        ax1 = f0.add_subplot(4,2,(5,6))
        ax0.plot(ref_x, ref_y, '--', lw=3)
        ax0.plot(pos_x[0], pos_y[0], 'ms',markersize = 10)
        ax0.plot(pos_x, pos_y, 'ro')
        ax0.set_title('(a)')
        ax1.set_title('(c)')
    else:
        ax0 = f0.add_subplot(4,2,(2,4))
        ax1 = f0.add_subplot(4,2,(7,8))
        ax0.plot(ref_x, ref_y, '--', lw=3,label='Reference Path')
        ax0.plot(pos_x[0], pos_y[0], 'ms',markersize = 10,label='Start Point')
        ax0.plot(pos_x, pos_y, 'ro',label='Robot Trajectory')
        ax0.set_title('(b)')
        ax1.set_title('(d)')
    ax0.legend()
    ax0.axis('equal')
    ax0.set_xlabel('X (m)')
    ax0.set_ylabel('Y (m)')
    # fig_size = 1.7
    # ax0.set_xlim([-fig_size, fig_size])
    # offset = .2
    ax0.set_ylim([-.1, 5.3])


    ax1.plot(time, error_long,lw=3,label='Longitudinal Error')
    ax1.plot(time, error_lat,lw=3,label='Lateral Error')
    ax1.legend()
    ax1.set_xlim([0, 25])
    # offset = .08
    ax1.set_ylim([-.25, .15])
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Error (m)')

    if iteration is 0:
        ax2 = f2.add_subplot(111)
        ax2.plot(time,vel,lw=3,label='Velocity Profile')
        ax2.legend()
        ax2.set_ylim([0,.55])
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity (m/s)')
    iteration +=1

f0.tight_layout()
f2.tight_layout()
plt.draw()
plt.pause(.1)  # <-------
raw_input("<Hit Enter To Close>")
plt.close(f0)
# plt.close(f1)
