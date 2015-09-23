# content of test_sample.py
from numpy import pi
from sys import path
path.append('..')
from rotation import *

acceptable_error = .001


def test_2D_angle():
    for i in range(14):
        theta = i * pi / 6
        assert rotate2DimFrame(1, 0, theta)[0] == cos(theta)


def test_2D_length():
    assert rotate2DimFrame(1, 0, pi / 3)[0] - .5 < acceptable_error


def test_wrap_angle_pi():
    assert wrapAnglePi(1.5 * pi) - (-pi / 2) < acceptable_error


def test_wrap_angle_2pi():
    assert wrapAngle2Pi(-pi / 6) - 5 * pi / 6 < acceptable_error
    assert wrapAngle2Pi(7 * pi / 6) - pi / 6 < acceptable_error


def test_degree_to_radian():
    assert degreeToRadian(30) - pi / 6 < acceptable_error
    assert degreeToRadian(150) - 5 * pi / 6 < acceptable_error
