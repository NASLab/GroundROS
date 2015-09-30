# content of test_sample.py
import pytest
from numpy import pi
from sys import path
path.append('..')
from path_planning import GapFinding

acceptable_error = .001


@pytest.fixture
def gap_finder():
    return GapFinding(1)


def test_polarToCartesian(gap_finder):
    x, y = gap_finder.polarToCartesian([1,1], [0,pi/2])
    assert x[0] == 1
    assert y[0] == 0

# def test_2D_angle():
#     for i in range(14):
#         theta = i * pi / 6
#         assert rotate2DimFrame(1, 0, theta)[0] == cos(theta)


# def test_2D_length():
#     assert rotate2DimFrame(1, 0, pi / 3)[0] - .5 < acceptable_error


# def test_wrap_angle_pi():
#     assert wrapAnglePi(1.5 * pi) - (-pi / 2) < acceptable_error


# def test_wrap_angle_2pi():
#     assert wrapAngle2Pi(-pi / 6) - 5 * pi / 6 < acceptable_error
#     assert wrapAngle2Pi(7 * pi / 6) - pi / 6 < acceptable_error


# def test_degree_to_radian():
#     assert degreeToRadian(30) - pi / 6 < acceptable_error
#     assert degreeToRadian(150) - 5 * pi / 6 < acceptable_error
