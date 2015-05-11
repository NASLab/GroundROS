# python experimental tests for Husky

import numpy as np
import glob
import matplotlib.pyplot as plt

file = glob.glob("*.npy")[1]

# for file in glob.glob("*.npy"):
data = np.load(file)[1:, :]
print file,
error_long = data[:, 0]
error_lat = data[:, 1]
ref_x = data[:, 2] * 1000
ref_y = data[:, 3] * 1000
pos_x = data[:, 4]
pos_y = data[:, 5]
pos_theta = data[:, 6]
time = data[:, 7] - data[0, 7]

start = 2
end = -6

plt.plot(time[start:end], error_lat[start:end])
plt.show()
tu = (time[end] - time[start]) / 9
ku = 14
print ku,tu

kp = .2*ku
ki = kp*2/tu
kd = kp*tu/3
print 'PID',kp,ki,kd

kp = .8*ku
# ki = kp*2/tu
kd = kp*tu/8
print 'PD',kp,kd