# content of test_sample.py
import pytest
from numpy import pi, load, linspace, sin, cos
from sys import path
import glob
import matplotlib.pyplot as plt
path.append('..')
from path_planning import GapFinding, SimpleGapFinder
from time import time

acceptable_error = .001


@pytest.fixture
def simple_lidar():
    return GapFinding(1)


@pytest.fixture
def actual_lidar():
    finder = GapFinding(.5)
    finder.setDistanceRange(20)
    return finder


@pytest.fixture
def simple_gap_finder():
    finder = SimpleGapFinder(.5)
    finder.setDistanceRange(5)
    return finder
# def test_polarToCartesian(simple_lidar):
#     x, y = simple_lidar.polarToCartesian([1, 1], [0, pi / 2])
#     assert x[0] == 1
#     assert y[0] == 0


def test_filterReadings(simple_lidar):
    readings = 6
    distances = [20 for i in range(readings)]
    angles = [10 * i * pi / 180 for i in range(readings)]
    distances[3] = 3
    distances, angles = simple_lidar.filterReadings(distances, angles)


def test_findObstacleLimits1(simple_lidar):
    readings = 6
    distances = [20 for i in range(readings)]
    angles = [10 * i * pi / 180 for i in range(readings)]

    distances[1] = 8.0
    distances[2] = 8.0
    distances[3] = 2.0
    distances[4] = 14.0

    simple_lidar.filterReadings(distances, angles)
    x, y = simple_lidar.polarToCartesian()
    simple_lidar.findObstacleLimits(x, y)
    assert simple_lidar.obstacle_limits == [[2, 2], [3, 3], [0, 1]]


def test_findObstacleLimits2(simple_lidar):
    readings = 6
    distances = [20 for i in range(readings)]

    distances[3] = 2
    distances[4] = 2
    angles = [10 * i * pi / 180 for i in range(readings)]
    distances, angles = simple_lidar.filterReadings(distances, angles)
    x, y = simple_lidar.polarToCartesian()
    simple_lidar.findObstacleLimits(x, y)
    assert simple_lidar.obstacle_limits == [[0, 1]]


def test_findObstacleLimits3(simple_lidar):
    readings = 6
    distances = [20 for i in range(readings)]

    distances[3] = 3
    # distances[4] = 3
    angles = [10 * i * pi / 180 for i in range(readings)]
    distances, angles = simple_lidar.filterReadings(distances, angles)
    x, y = simple_lidar.polarToCartesian()
    simple_lidar.findObstacleLimits(x, y)
    assert simple_lidar.obstacle_limits == [[0, 0]]


def test_findObstacleLimits4(simple_lidar):
    readings = 6
    distances = [20 for i in range(readings)]
    angles = [10 * i * pi / 180 for i in range(readings)]

    distances[2] = 9
    distances[3] = 9
    distances[4] = 9

    distances, angles = simple_lidar.filterReadings(distances, angles)
    x, y = simple_lidar.polarToCartesian()
    simple_lidar.findObstacleLimits(x, y)
    assert simple_lidar.obstacle_limits == [[0, 2]]


def test_findObstacleLimits5(simple_lidar):
    readings = 6
    distances = [20 for i in range(readings)]
    angles = [10 * i * pi / 180 for i in range(readings)]

    distances[1] = 9
    distances[2] = 9
    distances[3] = 3
    distances[4] = 14
    distances[5] = 9

    distances, angles = simple_lidar.filterReadings(distances, angles)
    x, y = simple_lidar.polarToCartesian()
    simple_lidar.findObstacleLimits(x, y)
    assert simple_lidar.obstacle_limits == [[2, 2], [3, 3], [4, 4], [0, 1]]


def test_findObstacleLimits6(simple_lidar):
    readings = 100
    distances = [20 for i in range(readings)]
    angles = [i * pi / 180 for i in range(readings)]

    for i in range(20, 30):
        distances[i] = 9

    distances[1] = 9
    distances[2] = 9
    distances[3] = 3
    distances[4] = 14
    distances[5] = 9

    distances, angles = simple_lidar.filterReadings(distances, angles)
    x, y = simple_lidar.polarToCartesian()
    simple_lidar.findObstacleLimits(x, y)
    assert simple_lidar.obstacle_limits == [[2, 2], [3, 3], [4, 14], [0, 1]]


def test_defineSubgoals(simple_lidar):
    readings = 6
    distances = [20 for i in range(readings)]
    angles = [10 * i * pi / 180 for i in range(readings)]

    distances[3] = 9
    distances[4] = 3

    distances, angles = simple_lidar.filterReadings(distances, angles)
    x, y = simple_lidar.polarToCartesian()
    simple_lidar.findObstacleLimits(x, y)
    # print simple_lidar.defineSubgoals(distances, angles)


def ntest_lidarSamples(actual_lidar):
    for file in glob.glob("*.npy"):
        # print test
        data = load(file)
        print file
        distances = data
        angles = linspace(-pi / 4, 5 * pi / 4, len(distances))
        actual_lidar.filterReadings(distances, angles)
        x, y = actual_lidar.polarToCartesian()
        actual_lidar.findObstacleLimits(x, y)
        # print data
        # print actual_lidar.readings_polar

        f0 = plt.figure()
        ax0 = f0.add_subplot(111)
        ax0.plot(x, y, 'r.')
        ax0.plot(0, 0, 'ko', markersize=10)
        for i in range(len(actual_lidar.obstacle_limits)):
            distance = actual_lidar.readings_polar[actual_lidar.obstacle_limits[i][0]][0]
            theta = actual_lidar.readings_polar[actual_lidar.obstacle_limits[i][0]][1]
            x1 = distance * cos(theta)
            y1 = distance * sin(theta)
            distance = actual_lidar.readings_polar[actual_lidar.obstacle_limits[i][1]][0]
            theta = actual_lidar.readings_polar[actual_lidar.obstacle_limits[i][1]][1]
            x2 = distance * cos(theta)
            y2 = distance * sin(theta)
            # print actual_lidar.obstacle_limits,x1, x2
            # xdata =
            # print xdata
            ax0.plot([x1, x2], [y1, y2], 'b', linewidth=3)
        ax0.axis('equal')
        plt.draw()
        plt.pause(.1)
        raw_input("<Hit Enter To Close>")
        plt.close(f0)


def test_findGaps(simple_gap_finder):
    simple_gap_finder.filterReadings([1, 1], [.17, .34])
    travel = simple_gap_finder.findGaps()
    print travel


def ntest_findGaps_actual_data(simple_gap_finder):
    for file in glob.glob("*.npy"):
        # print test
        data = load(file)
        print file
        distances = data
        # distances[:190]=[30]*190
        # distances[200:]=[30]*341
        angles = linspace(-pi / 4, 5 * pi / 4, len(distances))
        simple_gap_finder.filterReadings(distances, angles)
        x, y = simple_gap_finder.polarToCartesian()
        # simple_gap_finder.findObstacleLimits(x, y)
        travel = simple_gap_finder.findGaps()
        # print distances,travel
        # print simple_gap_finder.readings_polar

        f0 = plt.figure()
        ax0 = f0.add_subplot(111)
        ax0.plot(x, y, 'r.', markersize=15)
        ax0.plot(0, 0, 'ko', markersize=10)
        for i in range(len(travel)):
            x[i] = travel[i] * cos(simple_gap_finder.readings_polar[i][1])
            y[i] = travel[i] * sin(simple_gap_finder.readings_polar[i][1])

        ax0.plot(x, y, 'b')
        ax0.axis('equal')
        plt.draw()
        plt.pause(.1)
        raw_input("<Hit Enter To Close>")
        plt.close(f0)


def ntest_findSubgoals_actual_data(simple_gap_finder):
    for file in glob.glob("*.npy"):
        # print test
        data = load(file)
        print file
        distances = data
        # distances[:190]=[30]*190
        # distances[200:]=[30]*341
        angles = linspace(-pi / 4, 5 * pi / 4, len(distances))
        simple_gap_finder.filterReadings(distances, angles)
        x, y = simple_gap_finder.polarToCartesian()
        # simple_gap_finder.findObstacleLimits(x, y)
        travel = simple_gap_finder.findGaps()
        subgoals = simple_gap_finder.findSubgoals(travel)
        # print distances,travel
        # print simple_gap_finder.readings_polar

        f0 = plt.figure()
        ax0 = f0.add_subplot(111)
        ax0.plot(x, y, 'r.', markersize=15)
        ax0.plot(0, 0, 'ko', markersize=10)
        for i in range(len(travel)):
            x[i] = travel[i] * cos(simple_gap_finder.readings_polar[i][1])
            y[i] = travel[i] * sin(simple_gap_finder.readings_polar[i][1])
            if i in subgoals:
                ax0.plot(x[i], y[i], 'mo', markersize=15)
        ax0.plot(x, y, 'b.')
        ax0.axis('equal')
        plt.draw()
        plt.pause(.1)
        raw_input("<Hit Enter To Close>")
        plt.close(f0)


def test_findSubgoals_actual_data(simple_gap_finder):
    obj = simple_gap_finder
    target_distance = 1
    target_angle = pi / 4

    for file in glob.glob("*.npy"):
        # print test
        data = load(file)
        # print file
        distances = data
        # distances[:100]=[30]*100
        # distances[250:]=[30]*291
        for i in range(4):
            angles = linspace(-pi / 4 + i * pi / 2, 5 * pi / 4+ i * pi / 2, len(distances))

            obj.filterReadings(distances, angles)
            x, y = obj.polarToCartesian()
            # obj.findObstacleLimits(x, y)
            obj.findGaps()
            obj.findSubgoals()
            best_subgoal = obj.selectSubgoal(target_distance, target_angle)
            environment_state = obj.isObstacleInTheWay(target_distance, target_angle)
        # print 'Is obstacle in the way?',environment_state
        # print 'the algorithm took:', time() - start_time
        # print 'travel:',travel
        # print distances,travel
        # print obj.readings_polar

            f0 = plt.figure()
            ax0 = f0.add_subplot(111)
            ax0.plot(x, y, 'r.')
            ax0.plot(0, 0, 'ko', markersize=5)
            for i in range(len(obj.possible_travel)):
                x[i] = obj.possible_travel[i] * cos(obj.readings_polar[i][1])
                y[i] = obj.possible_travel[i] * sin(obj.readings_polar[i][1])
                if i in obj.subgoals:
                    ax0.plot(x[i], y[i], 'mo', markersize=10)
                    # print obj.possible_travel[i]
            if environment_state is 'not_safe':
                ax0.plot(obj.possible_travel[best_subgoal] * cos(obj.readings_polar[best_subgoal][1]),
                         obj.possible_travel[best_subgoal] * sin(obj.readings_polar[best_subgoal][1]), 'go', markersize=20)
            elif environment_state is 'safe':
                ax0.plot(target_distance * cos(target_angle),
                         target_distance * sin(target_angle), 'go', markersize=20)
            elif environment_state is 'close_to_obstacle':
                ax0.plot(target_distance * cos(target_angle),
                         target_distance * sin(target_angle), 'ro', markersize=20)
            ax0.plot(target_distance * cos(target_angle), target_distance * sin(target_angle), 'cx', markersize=20, linewidth=10)
            ax0.plot(x, y, 'b.')
            ax0.axis('equal')
            plt.draw()
            plt.pause(.1)
            raw_input("<Hit Enter To Close>")
            plt.close(f0)


def ntest_execution_time(simple_gap_finder):
    total_time = 0
    target_distance = 1
    target_angle = pi / 4
    for file in glob.glob("*.npy"):
        # print test
        data = load(file)
        # print file
        distances = data
        # distances[:100]=[30]*100
        # distances[250:]=[30]*291
        for i in range(20):
            angles = linspace(-pi / 4 + i * pi / 10, 5 * pi / 4, len(distances))
            # distances = distances[start:end]
            # angles = angles[start:end]
            start_time = time()

            simple_gap_finder.filterReadings(distances, angles)
            # x, y = simple_gap_finder.polarToCartesian()
            # simple_gap_finder.findObstacleLimits(x, y)
            travel = simple_gap_finder.findGaps()
            simple_gap_finder.findSubgoals(travel)
            simple_gap_finder.selectSubgoal(target_distance, target_angle)
            simple_gap_finder.isObstacleInTheWay(target_distance, target_angle)
            time_elapsed = time() - start_time
            total_time = total_time + time_elapsed
    print 'took:', total_time
