# python experimental tests for Husky

import numpy as np
from numpy import sin, cos, pi, zeros
# import matplotlib.pyplot as plt



def calibration_cost_function(parameters):
    # print parameters
    yaw_bounde = 1 * pi / 180
    matrix_limits = 20
    matrix_resolution = .005
    number_of_rows = matrix_limits / matrix_resolution
    cost_matrix = zeros((number_of_rows, number_of_rows))
    env_data = np.load('env.npy')[1:]
    x_offset_calibrate, y_offset_calibrate = parameters
    yaw_calibrate = pi / 180 * (0)
# x_offset_calibrate = .2
# y_offset_calibrate = -.064

    # x = [[]] * len(env_data)
    # y = [[]] * len(env_data)
    # print len(env_data)
    for i in range(1, len(env_data) - 1):
        if len(env_data[i]) > 0:
            x = env_data[i][0]
            y = env_data[i][1]
            yaw = env_data[i][2]
            if len(env_data[i+1])==0 or abs(yaw - env_data[i - 1][2]) > yaw_bounde or abs(yaw - env_data[i + 1][2]) > yaw_bounde:
                continue
            readings = env_data[i][3]
            # k = 0
            for j in range(len(readings)):
                x_temp = readings[j][0] * cos(-readings[j][1])
                y_temp = readings[j][0] * sin(-readings[j][1])
                x_temp2 = x_temp * \
                    cos(yaw_calibrate) - y_temp * \
                    sin(yaw_calibrate) + x_offset_calibrate
                y_temp2 = y_temp * \
                    cos(yaw_calibrate) + x_temp * \
                    sin(yaw_calibrate) + y_offset_calibrate
                readings_x = x_temp2 * cos(yaw) - y_temp2 * sin(yaw) + x
                readings_y = y_temp2 * cos(yaw) + x_temp2 * sin(yaw) + y
                if readings_x < matrix_limits / 2 and readings_x > -matrix_limits / 2 and readings_y < matrix_limits / 2 and readings_y > -matrix_limits / 2:
                    cost_matrix[int((readings_x + matrix_limits / 2) / matrix_resolution), int(
                        (readings_y + matrix_limits / 2) / matrix_resolution)] = 1
                # k += 1
    cost = sum(sum(cost_matrix))
    # print parameters,cost
    return cost
