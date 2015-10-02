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
    x, y = gap_finder.polarToCartesian([1, 1], [0, pi / 2])
    assert x[0] == 1
    assert y[0] == 0


def test_filterReadings(gap_finder):
    readings = 6
    distances = [20 for i in range(readings)]
    distances[3] = 3
    angles = [10 * i * pi / 180 for i in range(readings)]
    distances, angles = gap_finder.filterReadings(distances, angles)


def test_findObstacleLimits1(gap_finder):
    print 'Std Outputs:'
    readings = 6
    distances = [20 for i in range(readings)]
    distances[1] = 9
    distances[2] = 9
    distances[3] = 3
    distances[4] = 14
    # distances[5] = 9
    angles = [10 * i * pi / 180 for i in range(readings)]
    distances, angles = gap_finder.filterReadings(distances, angles)
    x, y = gap_finder.polarToCartesian(distances, angles)
    print 'Distances and angles:', distances, angles
    limits = gap_finder.findObstacleLimits(x, y)
    assert limits == [2, 2, 3, 3, 0, 1]


def test_findObstacleLimits2(gap_finder):
    print 'Std Outputs:'
    readings = 6
    distances = [20 for i in range(readings)]

    distances[3] = 2
    distances[4] = 2
    angles = [10 * i * pi / 180 for i in range(readings)]
    distances, angles = gap_finder.filterReadings(distances, angles)
    x, y = gap_finder.polarToCartesian(distances, angles)
    print 'Distances and angles:', distances, angles
    limits = gap_finder.findObstacleLimits(x, y)
    assert limits == [0, 1]


def test_findObstacleLimits3(gap_finder):
    print 'Std Outputs:'
    readings = 6
    distances = [20 for i in range(readings)]

    distances[3] = 3
    # distances[4] = 3
    angles = [10 * i * pi / 180 for i in range(readings)]
    distances, angles = gap_finder.filterReadings(distances, angles)
    x, y = gap_finder.polarToCartesian(distances, angles)
    print 'Distances and angles:', distances, angles
    limits = gap_finder.findObstacleLimits(x, y)
    assert limits == [0, 0]


def test_findObstacleLimits4(gap_finder):
    print 'Std Outputs:'
    readings = 6
    distances = [20 for i in range(readings)]
    # distances[1] = 9
    distances[2] = 9
    distances[3] = 9
    distances[4] = 9
    # distances[5] = 9
    angles = [10 * i * pi / 180 for i in range(readings)]
    distances, angles = gap_finder.filterReadings(distances, angles)
    x, y = gap_finder.polarToCartesian(distances, angles)
    print 'Distances and angles:', distances, angles
    limits = gap_finder.findObstacleLimits(x, y)
    assert limits == [0, 2]


def test_findObstacleLimits5(gap_finder):
    print 'Std Outputs:'
    readings = 6
    distances = [20 for i in range(readings)]
    distances[1] = 9
    distances[2] = 9
    distances[3] = 3
    distances[4] = 14
    distances[5] = 9
    angles = [10 * i * pi / 180 for i in range(readings)]
    distances, angles = gap_finder.filterReadings(distances, angles)
    x, y = gap_finder.polarToCartesian(distances, angles)
    print 'Distances and angles:', distances, angles
    limits = gap_finder.findObstacleLimits(x, y)
    assert limits == [2, 2, 3, 3, 4, 4, 0, 1]

def test_findObstacleLimits6(gap_finder):
    print 'Std Outputs:'
    readings = 100
    distances = [20 for i in range(readings)]
    for i in range (20,30):
        distances[i] = 9

    distances[1] = 9
    distances[2] = 9
    distances[3] = 3
    distances[4] = 14
    distances[5] = 9
    angles = [ i * pi / 180 for i in range(readings)]
    distances, angles = gap_finder.filterReadings(distances, angles)
    x, y = gap_finder.polarToCartesian(distances, angles)
    print 'Distances and angles:', distances, angles
    limits = gap_finder.findObstacleLimits(x, y)
    print limits


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
