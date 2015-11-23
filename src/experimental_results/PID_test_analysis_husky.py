# python experimental tests for Husky

import numpy as np
import glob
import matplotlib.pyplot as plt

f0 = plt.figure()
f1 = plt.figure()

for file in glob.glob("*.npy"):
    data = np.load(file)[5:, :]
    print file,
    error_long = data[:, 0]
    error_lat = data[:, 1]
    ref_x = [value-data[0,2]+1 for value in data[:, 2]]
    # print ref_x[:30]
    ref_y = [value-data[0,3] for value in data[:, 3]]
    pos_x = [value-data[0,4]+1 for value in data[:, 4]]
    pos_y = [value-data[0,5] for value in data[:, 5]]
    pos_theta = data[:, 6]
    time = data[:, 7] - data[0, 7]

    # plt.plot(ref_x, ref_y, 'ro')
    # plt.gca().set_aspect('equal', adjustable='box')
    ax0 = f0.add_subplot(111)
    ax0.plot(pos_x[0], pos_y[0], 'bo')
    ax0.plot(pos_x, pos_y,linewidth=3,label='Velocity = 0.'+file[29:30])
    ax0.axis('equal')

    ax1 = f1.add_subplot(111)
    ax1.plot(time, error_long,label='Longitudional error; Velocity = 0.'+file[29:30])
    ax1.plot(time, error_lat,label='Lateral error; Velocity = 0.'+file[29:30])
    ax1.legend()
ax0.plot(ref_x, ref_y, '--',linewidth = 3, label='Reference')
ax0.legend()
plt.draw()
plt.pause(.1)  # <-------
raw_input("<Hit Enter To Close>")
plt.close(f0)
plt.close(f1)
