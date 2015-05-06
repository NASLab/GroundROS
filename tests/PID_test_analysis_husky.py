# python experimental tests for Husky

import numpy as np
import glob, os
import matplotlib.pyplot as plt


# for file in glob.glob("*.npy"):
#     kp = float(file[13:15])
#     if kp == 8.0:
#         print file
#         data = np.load(file)[1:-1,:]
#         # print data
#         time = data[:,-1]-data[0,-1]
#         # print time
#         plt.plot(time,data[:,1])
#         plt.show()

file_name = 'long 0.0 lat 08.0 2015-05-05 18:56:48.875684.npy'
data = np.load(file_name)[1:-1,:]
start = 492
end = -24
time = data[start:end,-1]-data[start,-1]
var = data[start:end,1]
print time[-1]
plt.plot(time,var)
plt.show()

longitudinal_ku = 8
longitudinal_tu = 1.877
kp = .2*longitudinal_ku
ki = kp*2/longitudinal_tu
kd = kp*longitudinal_tu/3
print kp,ki,kd


lateral_ku = 8
lateral_tu = 16.5/11
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