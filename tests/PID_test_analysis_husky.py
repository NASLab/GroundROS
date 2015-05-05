# python experimental tests for Husky

import numpy as np
import glob, os
import matplotlib.pyplot as plt


for file in glob.glob("time2015-05-04 17:26*.npy"):
    print '++++++++++++++++++++++++++++++++++++++++++++++++++++'
    # print file
    # if len(np.load(file)) is 0:
    #     print(file)

longitudinal_ku = 8
longitudinal_tu = 1.877
kp = .2*longitudinal_ku
ki = kp*2/longitudinal_tu
kd = kp*longitudinal_tu/3
print kp,ki,kd


lateral_ku = 60
lateral_tu = .85
kp = .2*lateral_ku
ki = kp*2/lateral_tu
kd = kp*lateral_tu/3
print kp,ki,kd



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