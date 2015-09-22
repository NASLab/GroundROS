from numpy import sin, cos

nan_var = float('nan')


def rotate2DimFrame(initial_x, initial_y, theta):
    rotated_x = cos(theta) * initial_x + sin(theta) * initial_y
    rotated_y = cos(theta) * initial_y - sin(theta) * initial_x
    return rotated_x, rotated_y


def rotate3DimFrame(initial_x=nan_var, initial_y=nan_var, initial_z=nan_var, roll=nan_var, pitch=nan_var, yaw=nan_var):
    raise NotImplementedError(
        'Function is not defined and developed yet. Contact Developer.')
