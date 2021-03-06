# python experimental tests for Husky

from numpy import sin, cos, pi, load
import matplotlib.pyplot as plt
from time import sleep

yaw_bound = 2 * pi / 180
yaw_calibrate = pi / 180 * (0)
x_offset_calibrate = 0
y_offset_calibrate = -.08


f0 = plt.figure()
ax0 = f0.add_subplot(111)
ax1 = f0.add_subplot(111)
env_data = load('loginfo2.npy')[1:]
x = [[]] * len(env_data)
y = [[]] * len(env_data)

m=2

# print len(env_data)
for i in range(m, len(env_data) - m):
    if len(env_data[i]) > 0:
        x[i] = env_data[i][0]
        y[i] = env_data[i][1]
        print i, x[i], y[i]
        yaw = env_data[i][2]

        # filter some of the readings; comment to see the effect
        if len(env_data[i + m]) == 0 or abs(yaw - env_data[i - m][2]) > yaw_bound or abs(yaw - env_data[i + m][2]) > yaw_bound:
            continue

        readings = env_data[i][3]
        readings_x = [[]] * len(readings)
        readings_y = [[]] * len(readings)
        k = 0
        for j in range(len(readings)):
            # lidar readings in lidar frame
            x_temp = readings[j][0] * cos(-readings[j][1])
            y_temp = readings[j][0] * sin(-readings[j][1])

            # lidar readings in robot frame
            x_temp2 = x_temp * \
                cos(yaw_calibrate) - y_temp * \
                sin(yaw_calibrate) + x_offset_calibrate
            y_temp2 = y_temp * \
                cos(yaw_calibrate) + x_temp * \
                sin(yaw_calibrate) + y_offset_calibrate

            # lidar readings in global frame
            readings_x[k] = x_temp2 * cos(yaw) - y_temp2 * sin(yaw) + x[i]
            readings_y[k] = y_temp2 * cos(yaw) + x_temp2 * sin(yaw) + y[i]
            k += 1

        ax0.plot(readings_x, readings_y, 'r.')
        ax0.plot(x[i],y[i], 'go', lw=3)
        ax0.plot(0,-10,'ko')
        ax0.set_xlim([-50, 50])
        ax0.set_ylim([-50, 50])
        # ax0.axis('equal')
        plt.draw()
        plt.pause(.0001)
        ax0.clear()
ax0.plot([], [], 'r.', label='Lidar Reading')

# env_data = load('planner_of_agent_0.npy')[1:]
# x = [[]] * len(env_data)
# y = [[]] * len(env_data)
# for i in range(1, len(env_data) - 1):
#     if len(env_data[i]) > 0:
#         x[i] = env_data[i][0]
#         y[i] = env_data[i][1]
ax0.plot([value for value in x if value],
         [value for value in y if value], 'go', lw=3, label='Robot\'s Trajectory')
# env_data = load('planner_of_agent_1.npy')[1:]

# x = [[]] * len(env_data)
# y = [[]] * len(env_data)
# for i in range(1, len(env_data) - 1):
#     if len(env_data[i]) > 0:
#         x[i] = env_data[i][0]
#         y[i] = env_data[i][1]
# ax0.plot([value for value in x if value],
#          [value for value in y if value], 'bo', lw=3, label='Robot\'s Trajectory')



# ax0.legend()
# ax0.axis('equal')
# plt.draw()
# plt.pause(.1)
# raw_input("<Hit Enter To Close>")
plt.close(f0)


# yaw_bound = 3 * pi / 180
# yaw_calibrate = pi / 180 * (0)
# x_offset_calibrate = .23
# y_offset_calibrate = -.08


# data = np.load('pos.npy')[1:]
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
# f0 = plt.figure(1, figsize=(9, 9))
# ax0 = f0.add_subplot(111)
# ax0.plot(ref_x, ref_y, '--', lw=3, label='Reference Trajectory')
# ax0.plot(pos_x[0], pos_y[0], 'ms', markersize=10, label='Start Point')
# ax0.plot(pos_x, pos_y, 'go', label='Robot Trajectory')
# env_data = np.load('planner_of_agent_0.npy')[1:]
# x = [[]] * len(env_data)
# y = [[]] * len(env_data)
# print len(env_data)

# for i in range(1, len(env_data) - 1):
#     if len(env_data[i]) > 0:
#         x[i] = env_data[i][0]
#         y[i] = env_data[i][1]
#         yaw = env_data[i][2]

# filter some of the readings; comment to see the effect
#         if len(env_data[i + 1]) == 0 or abs(yaw - env_data[i - 1][2]) > yaw_bound or abs(yaw - env_data[i + 1][2]) > yaw_bound:
#             continue

#         readings = env_data[i][3]
#         readings_x = [[]] * len(readings)
#         readings_y = [[]] * len(readings)
#         k = 0
#         for j in range(len(readings)):
# lidar readings in lidar frame
#             x_temp = readings[j][0] * cos(-readings[j][1])
#             y_temp = readings[j][0] * sin(-readings[j][1])

# lidar readings in robot frame
#             x_temp2 = x_temp * \
#                 cos(yaw_calibrate) - y_temp * \
#                 sin(yaw_calibrate) + x_offset_calibrate
#             y_temp2 = y_temp * \
#                 cos(yaw_calibrate) + x_temp * \
#                 sin(yaw_calibrate) + y_offset_calibrate

# lidar readings in global frame
#             readings_x[k] = x_temp2 * cos(yaw) - y_temp2 * sin(yaw) + x[i]
#             readings_y[k] = y_temp2 * cos(yaw) + x_temp2 * sin(yaw) + y[i]
#             k += 1

#         ax0.plot(readings_x, readings_y, 'r.')

# for i in range(len(env_data)):
# if len(env_data[i])>0:
# x[i] = env_data[i][0]
# y[i] = env_data[i][1]
# yaw = env_data[i][2]
# print yaw
# readings = env_data[i][3]
# readings_x = [[]]*len(readings)
# readings_y = [[]]*len(readings)
# print len(readings),len(readings_x)
# k=0
# for j in range(len(readings)):
# if i<200:
# print k,j,len(readings_x)
# readings_x[k] = x[i] + readings[j][0]*sin(pi/2-yaw+readings[j][1])
# readings_y[k] = y[i] + readings[j][0]*cos(pi/2-yaw+readings[j][1])
# k+=1
# ax0.plot(readings_x, readings_y,'r.')

# ax0.plot([], [], 'r.', label='Lidar Reading')
# print x
# ax0.plot([value for value in x if value],
#          [value for value in y if value], 'go', lw=3,label='Robot\'s Trajectory')


# env_y = np.load('env.npy')[1]
# env_x = [value for value in env_x if value]
# env_y = [value for value in env_y if value]
# ax0.plot(env_x, env_y, 'r.', )
# ax0.plot(-.5, 2.7, 'cs', markersize=10, label='Destination')
# ax0.legend()
# ax0.axis('equal')
# ax0.set_xlim(-3.5, 3.5)
# ax0.set_ylim(-3, 4)
# ax0.set_xlabel('X (m)')
# ax0.set_ylabel('Y (m)')
# ax0.axis('equal')

# plt.tight_layout()
# plt.draw()
# plt.pause(.1)  # <-------
# raw_input("<Hit Enter To Close>")
# plt.close(f0)
