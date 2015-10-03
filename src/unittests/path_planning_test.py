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
    gap_finder.findObstacleLimits(x, y)
    assert gap_finder.obstacle_limits == [[2, 2],[ 3, 3], [0, 1]]


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
    gap_finder.findObstacleLimits(x, y)
    assert gap_finder.obstacle_limits == [[0, 1]]


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
    gap_finder.findObstacleLimits(x, y)
    assert gap_finder.obstacle_limits == [[0, 0]]


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
    gap_finder.findObstacleLimits(x, y)
    assert gap_finder.obstacle_limits == [[0, 2]]


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
    gap_finder.findObstacleLimits(x, y)
    assert gap_finder.obstacle_limits == [[2, 2], [3, 3], [4, 4], [0, 1]]


def test_findObstacleLimits6(gap_finder):
    print 'Std Outputs:'
    readings = 100
    distances = [20 for i in range(readings)]
    for i in range(20, 30):
        distances[i] = 9

    distances[1] = 9
    distances[2] = 9
    distances[3] = 3
    distances[4] = 14
    distances[5] = 9
    angles = [i * pi / 180 for i in range(readings)]
    distances, angles = gap_finder.filterReadings(distances, angles)
    x, y = gap_finder.polarToCartesian(distances, angles)
    print 'Distances and angles:', distances, angles
    gap_finder.findObstacleLimits(x, y)
    print gap_finder.obstacle_limits

