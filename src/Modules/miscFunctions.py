from numpy import sin, cos, pi
import logging
from time import strftime

HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'

nan_var = float('nan')

radian_over_degree = pi / 180.0
degree_over_radian = 180.0 / pi


def setupLogger(verbose=True, name=[]):
    if not name:
        raise ValueError('The argument "name" should be defined in call to setupLogger(verbose, name)!')
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    formatter = logging.Formatter('%(asctime)s-%(process)d-%(levelname)s-%(message)s')
    debug_file = logging.FileHandler(filename=name + '_logs_' + strftime("%d%b%Y") + '.log')
    debug_file.setLevel(logging.INFO)
    debug_file.setFormatter(formatter)
    logger.addHandler(debug_file)
    debug_stream = logging.StreamHandler()
    if verbose:
        debug_stream.setLevel(logging.DEBUG)
    else:
        debug_stream.setLevel(logging.WARNING)
    debug_stream.setFormatter(formatter)
    logger.addHandler(debug_stream)
    return logger


def rotate2DimFrame(initial_x, initial_y, theta):
    rotated_x = cos(theta) * initial_x + sin(theta) * initial_y
    rotated_y = cos(theta) * initial_y - sin(theta) * initial_x
    return rotated_x, rotated_y


def rotate3DimFrame(initial_x=nan_var, initial_y=nan_var, initial_z=nan_var, roll=nan_var, pitch=nan_var, yaw=nan_var):
    raise NotImplementedError(
        'Function is not defined and developed yet. Contact Developer.')


def wrapAnglePi(theta):
    return (theta + pi) % (2 * pi) - pi


def wrapAngle2Pi(theta):
    return (theta + pi) % (2 * pi)


def degreeToRadian(theta):
    return theta * radian_over_degree


def stringFormat(color, text):
    return color + text + ENDC


def fail(text):
    return stringFormat(FAIL, text)


def warn(text):
    return stringFormat(WARNING, text)


def green(text):
    return stringFormat(OKGREEN, text)


def blue(text):
    return stringFormat(OKBLUE, text)


def bold(text):
    return stringFormat(BOLD, text)
