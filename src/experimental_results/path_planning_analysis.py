# python experimental tests for Husky

import numpy as np
from numpy import sin, cos, pi
import matplotlib.pyplot as plt

f0 = plt.figure()
# for p in range(9):
yaw_calibrate = pi/180 * (0)
x_offset_calibrate = .26
y_offset_calibrate = -.064
# data = np.load('pos.npy')[5:, :]
# print len(data)
# error_long = data[:, 0]
# error_lat = data[:, 1]
# ref_x = [value for value in data[:, 2]]
# print ref_x[:30]
# ref_y = [value for value in data[:, 3]]
# pos_x = [value for value in data[:, 4]][0::1]
# pos_y = [value for value in data[:, 5]][0::1]
# pos_theta = data[:, 6]
# print data
# time = data[:, 7] - data[0, 7]
# vel = data[:, 8]
# plt.plot(ref_x, ref_y, 'ro')
# plt.gca().set_aspect('equal', adjustable='box')
ax0 = f0.add_subplot(111)
# ax0.plot(ref_x, ref_y, '--', lw=3, label='Reference Trajectory')
# ax0.plot(pos_x[0], pos_y[0], 'mo', markersize=10, label='Start Point')
# ax0.plot(pos_x, pos_y, 'go', label='Robot Trajectory')
env_data = np.load('env.npy')[1:]
x = [[]] * len(env_data)
y = [[]] * len(env_data)
# print len(env_data)
for i in range(len(env_data)):
    if len(env_data[i]) > 0:
        x[i] = env_data[i][0]
        y[i] = env_data[i][1]
        yaw = env_data[i][2]
        # print yaw
        readings = env_data[i][3]
        readings_x = [[]] * len(readings)
        readings_y = [[]] * len(readings)
        # print len(readings),len(readings_x)
        k = 0
        for j in range(len(readings)):
            # if i<200:
            # print k,j,len(readings_x)
            x_temp = readings[j][0] * cos(-readings[j][1])
            y_temp = readings[j][0] * sin(-readings[j][1])
            x_temp2 = x_temp*cos(yaw_calibrate) - y_temp*sin(yaw_calibrate) + x_offset_calibrate
            y_temp2 = y_temp*cos(yaw_calibrate) + x_temp*sin(yaw_calibrate) + y_offset_calibrate
            readings_x[k] = x_temp2*cos(yaw) - y_temp2*sin(yaw) + x[i]
            readings_y[k] = y_temp2*cos(yaw) + x_temp2*sin(yaw) + y[i]
            k += 1
        ax0.plot(readings_x, readings_y,'r,')

ax0.plot([], [], ',r', label='Lidar Reading'+str( y_offset_calibrate ))
# print x
ax0.plot([value for value in x if value], [value for value in y if value], 'go', lw=3)


# env_y = np.load('env.npy')[1]
# env_x = [value for value in env_x if value]
# env_y = [value for value in env_y if value]
# ax0.plot(env_x, env_y, 'r.', )
# ax0.plot(-.5, 2.7, 'cx', markersize=10, label='Destination')
ax0.legend()
ax0.axis('equal')


plt.draw()
plt.pause(.1)  # <-------
raw_input("<Hit Enter To Close>")
plt.close(f0)
