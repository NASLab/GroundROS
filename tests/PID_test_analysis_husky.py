# python experimental tests for Husky

import numpy as np
import glob
import matplotlib.pyplot as plt


for file in glob.glob("*.npy"):
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

    # plt.plot(ref_x, ref_y, 'ro')
    # plt.gca().set_aspect('equal', adjustable='box')
    f0 = plt.figure()
    f1 = plt.figure()
    ax0 = f0.add_subplot(111)
    ax0.plot(ref_x, ref_y, 'ro', label='Reference')
    ax0.plot(pos_x[0], pos_y[0], 'bo')
    ax0.plot(pos_x, pos_y, 'b',label='Robot trajectory')
    ax0.legend()
    ax0.axis('equal')
    ax1 = f1.add_subplot(111)
    ax1.plot(time, error_long,label='Long error')
    ax1.plot(time, error_lat,label='Lat error')
    ax1.legend()
    plt.draw()
    plt.pause(.1)  # <-------
    raw_input("<Hit Enter To Close>")
    plt.close(f0)
    plt.close(f1)
#     kp = float(file[13:15])
#     if kp == 8.0:
#         print file
#         data = np.load(file)[1:-1,:]
# print data
#         time = data[:,-1]-data[0,-1]
# print time
#         plt.plot(time,data[:,1])
#         plt.show()

# file_name = 'long 0.0 lat 08.0 2015-05-05 18:56:48.875684.npy'
# data = np.load(file_name)[1:-1,:]
# start = 492
# end = -24
# time = data[start:end,-1]-data[start,-1]
# var = data[start:end,1]
# print time[-1]
# plt.plot(time,var)
# plt.show()

# longitudinal_ku = 8
# longitudinal_tu = 1.877
# kp = .2*longitudinal_ku
# ki = kp*2/longitudinal_tu
# kd = kp*longitudinal_tu/3
# print kp,ki,kd


# lateral_ku = 8
# lateral_tu = 16.5/11
# kp = .2*lateral_ku
# ki = kp*2/lateral_tu
# kd = kp*lateral_tu/3
# print kp,ki,kd


# longitudinal_file = 'error_history22015-05-04 16:05:58.462202.npy'
# lateral_file_1 = 'error_history22015-05-04 16:26:26.910771.npy'
# lateral_file_2 = 'error_history22015-05-04 16:25:08.701600.npy'
# print np.load(longitudinal_file)
# plt.plot(np.load(),np.load(longitudinal_file))
# plt.show()

# start = 27
# end = -12
# time_array = np.load('time2015-05-04 17:24:46.536890.npy')[start:end]
# time_array = time_array - time_array[0]
# error_array = np.load("error2015-05-04 17:24:46.536442.npy")[start:end]
# print error_array
# plt.plot(time_array,error_array)
# plt.show()

# print time_array[-1]/7
