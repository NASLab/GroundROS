# python experimental tests for Husky

from numpy import sin, cos, pi, load
import matplotlib.pyplot as plt

yaw_bound = 1 * pi / 180
yaw_calibrate = pi / 180 * (0)
x_offset_calibrate = .23
y_offset_calibrate = -.08


f0 = plt.figure()
ax0 = f0.add_subplot(111)
env_data = load('env.npy')[1:]
x = [[]] * len(env_data)
y = [[]] * len(env_data)

for i in range(1, len(env_data) - 1):
    if len(env_data[i]) > 0:
        x[i] = env_data[i][0]
        y[i] = env_data[i][1]
        yaw = env_data[i][2]

        # filter some of the readings; comment to see the effect
        if len(env_data[i + 1]) == 0 or abs(yaw - env_data[i - 1][2]) > yaw_bound or abs(yaw - env_data[i + 1][2]) > yaw_bound:
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

ax0.plot([], [], 'r.', label='Lidar Reading' + str(y_offset_calibrate))
# ax0.plot([value for value in x if value], [
#          value for value in y if value], 'go', lw=3)
ax0.legend()
ax0.axis('equal')


plt.draw()
plt.pause(.1)
raw_input("<Hit Enter To Close>")
plt.close(f0)
