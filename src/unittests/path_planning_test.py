# content of test_sample.py
import pytest
from numpy import pi, load, linspace, sin, cos
import glob
import matplotlib.pyplot as plt
from sys import path
path.append('..')
from path_planning import GapFinding, GapFinder
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
    finder = GapFinder(.5)
    finder.setDistanceRange(5)
    return finder
# def test_polarToCartesian(simple_lidar):
#     x, y = simple_lidar.polarToCartesian([1, 1], [0, pi / 2])
#     assert x[0] == 1
#     assert y[0] == 0


# def test_filterReadings(simple_lidar):
#     readings = 6
#     distances = [20 for i in range(readings)]
#     angles = [10 * i * pi / 180 for i in range(readings)]
#     distances[3] = 3
#     distances, angles = simple_lidar.filterReadings(distances, angles)


# def test_findObstacleLimits1(simple_lidar):
#     readings = 6
#     distances = [20 for i in range(readings)]
#     angles = [10 * i * pi / 180 for i in range(readings)]

#     distances[1] = 8.0
#     distances[2] = 8.0
#     distances[3] = 2.0
#     distances[4] = 14.0

#     simple_lidar.filterReadings(distances, angles)
#     x, y = simple_lidar.polarToCartesian()
#     simple_lidar.findObstacleLimits(x, y)
#     assert simple_lidar.obstacle_limits == [[2, 2], [3, 3], [0, 1]]


# def test_findObstacleLimits2(simple_lidar):
#     readings = 6
#     distances = [20 for i in range(readings)]

#     distances[3] = 2
#     distances[4] = 2
#     angles = [10 * i * pi / 180 for i in range(readings)]
#     distances, angles = simple_lidar.filterReadings(distances, angles)
#     x, y = simple_lidar.polarToCartesian()
#     simple_lidar.findObstacleLimits(x, y)
#     assert simple_lidar.obstacle_limits == [[0, 1]]


# def test_findObstacleLimits3(simple_lidar):
#     readings = 6
#     distances = [20 for i in range(readings)]

#     distances[3] = 3
#     angles = [10 * i * pi / 180 for i in range(readings)]
#     distances, angles = simple_lidar.filterReadings(distances, angles)
#     x, y = simple_lidar.polarToCartesian()
#     simple_lidar.findObstacleLimits(x, y)
#     assert simple_lidar.obstacle_limits == [[0, 0]]


# def test_findObstacleLimits4(simple_lidar):
#     readings = 6
#     distances = [20 for i in range(readings)]
#     angles = [10 * i * pi / 180 for i in range(readings)]

#     distances[2] = 9
#     distances[3] = 9
#     distances[4] = 9

#     distances, angles = simple_lidar.filterReadings(distances, angles)
#     x, y = simple_lidar.polarToCartesian()
#     simple_lidar.findObstacleLimits(x, y)
#     assert simple_lidar.obstacle_limits == [[0, 2]]


# def test_findObstacleLimits5(simple_lidar):
#     readings = 6
#     distances = [20 for i in range(readings)]
#     angles = [10 * i * pi / 180 for i in range(readings)]

#     distances[1] = 9
#     distances[2] = 9
#     distances[3] = 3
#     distances[4] = 14
#     distances[5] = 9

#     distances, angles = simple_lidar.filterReadings(distances, angles)
#     x, y = simple_lidar.polarToCartesian()
#     simple_lidar.findObstacleLimits(x, y)
#     assert simple_lidar.obstacle_limits == [[2, 2], [3, 3], [4, 4], [0, 1]]


# def test_findObstacleLimits6(simple_lidar):
#     readings = 100
#     distances = [20 for i in range(readings)]
#     angles = [i * pi / 180 for i in range(readings)]

#     for i in range(20, 30):
#         distances[i] = 9

#     distances[1] = 9
#     distances[2] = 9
#     distances[3] = 3
#     distances[4] = 14
#     distances[5] = 9

#     distances, angles = simple_lidar.filterReadings(distances, angles)
#     x, y = simple_lidar.polarToCartesian()
#     simple_lidar.findObstacleLimits(x, y)
#     assert simple_lidar.obstacle_limits == [[2, 2], [3, 3], [4, 14], [0, 1]]


# def test_defineSubgoals(simple_lidar):
#     readings = 6
#     distances = [20 for i in range(readings)]
#     angles = [10 * i * pi / 180 for i in range(readings)]

#     distances[3] = 9
#     distances[4] = 3

#     distances, angles = simple_lidar.filterReadings(distances, angles)
#     x, y = simple_lidar.polarToCartesian()
#     simple_lidar.findObstacleLimits(x, y)
# print simple_lidar.defineSubgoals(distances, angles)


# def test_lidarSamples(actual_lidar):
#     for file in glob.glob("*.npy"):
#         print file
#         data = load(file)
#         distances = data
#         angles = linspace(-pi / 4, 5 * pi / 4, len(distances))
#         actual_lidar.filterReadings(distances, angles)
#         x, y = actual_lidar.polarToCartesian()
#         actual_lidar.findObstacleLimits(x, y)

#         f0 = plt.figure()
#         ax0 = f0.add_subplot(111)
#         ax0.plot(x, y, 'r.')
#         ax0.plot(0, 0, 'ko', markersize=10)
#         for i in range(len(actual_lidar.obstacle_limits)):
#             distance = actual_lidar.readings_polar[actual_lidar.obstacle_limits[i][0]][0]
#             theta = actual_lidar.readings_polar[actual_lidar.obstacle_limits[i][0]][1]
#             x1 = distance * cos(theta)
#             y1 = distance * sin(theta)
#             distance = actual_lidar.readings_polar[actual_lidar.obstacle_limits[i][1]][0]
#             theta = actual_lidar.readings_polar[actual_lidar.obstacle_limits[i][1]][1]
#             x2 = distance * cos(theta)
#             y2 = distance * sin(theta)
#             ax0.plot([x1, x2], [y1, y2], 'b', linewidth=3)
#         ax0.axis('equal')
#         plt.draw()
#         plt.pause(.1)
#         raw_input("<Hit Enter To Close>")
#         plt.close(f0)

def test_filterReadings(simple_gap_finder):
    simple_gap_finder.filterReadings([1, 1], [.17, .34])
    assert simple_gap_finder.readings_polar == [[1, 0.17], [1, 0.34]]
    assert simple_gap_finder.number_of_readings == 2


def test_setDistance(simple_gap_finder):
    simple_gap_finder.setDistanceRange(1)
    assert simple_gap_finder.distance_range == 1.0
    simple_gap_finder.setDistanceRange(25)
    assert simple_gap_finder.distance_range == 25


def test_polarToCartesian(simple_gap_finder):
    simple_gap_finder.filterReadings([1, 1], [0, pi / 2])
    x, y = simple_gap_finder.polarToCartesian()
    assert x[0] == 1
    assert y[0] == 0


def test_findSubgoals(simple_gap_finder):
    distances = [1, 2, 3, 4, 5, 4, 3, 2, 1]
    angles = linspace(-pi * 3 / 4, pi * 3 / 4, len(distances))
    simple_gap_finder.filterReadings(distances, angles)
    simple_gap_finder.findGaps()
    simple_gap_finder.findSubgoals()
    best_subgoal = simple_gap_finder.selectSubgoal(1, pi / 4)
    assert simple_gap_finder.possible_travel == [0.5, 1.5, 2.5, 3.5, 4.5, 3.5, 2.5, 1.5, 0.5]
    assert simple_gap_finder.subgoals == [0, 8, 1, 2, 3, 4, 4, 5, 6, 7]  # TODO: FIXT THIS SHIT. THERE ARE TWO 4s
    assert best_subgoal == 8


def ntest_user_feedback(simple_gap_finder):
    obj = simple_gap_finder
    target_distance = 4

    for file in glob.glob("*.npy"):
        print file
        data = load(file)
        distances = data
        angles = linspace(-pi * 3 / 4, pi * 3 / 4, len(distances))

        for i in range(5):
            target_angle = -pi / 2 + i * pi / 4

            obj.filterReadings(distances, angles)
            x, y = obj.polarToCartesian()
            obj.findGaps()
            obj.findSubgoals()
            best_subgoal = obj.selectSubgoal(target_distance, target_angle)
            environment_state = obj.isObstacleInTheWay(target_distance, target_angle)

            f0 = plt.figure()
            ax0 = f0.add_subplot(111)
            ax0.plot(x, y, 'r.')
            ax0.plot(0, 0, 'ko', markersize=5)
            for i in range(len(obj.possible_travel)):
                x[i] = obj.possible_travel[i] * cos(obj.readings_polar[i][1])
                y[i] = obj.possible_travel[i] * sin(obj.readings_polar[i][1])
                if i in obj.subgoals:
                    ax0.plot(x[i], y[i], 'mo', markersize=10)
            if environment_state is 'not_safe':
                ax0.plot(obj.possible_travel[best_subgoal] * cos(obj.readings_polar[best_subgoal][1]),
                         obj.possible_travel[best_subgoal] * sin(obj.readings_polar[best_subgoal][1]), 'mo', markersize=20)
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


def test_execution_time(simple_gap_finder):
    total_time = 0
    target_distance = 1
    target_samples=20

    for file in glob.glob("*.npy"):
        data = load(file)
        distances = data
        angles = linspace(-pi * 3 / 4, pi * 3 / 4, len(distances))

        for i in range(target_samples):
            target_angle = -pi / 2 + i * pi / 20
            start_time = time()

            simple_gap_finder.filterReadings(distances, angles)
            simple_gap_finder.findGaps()
            simple_gap_finder.findSubgoals()
            simple_gap_finder.selectSubgoal(target_distance, target_angle)
            simple_gap_finder.isObstacleInTheWay(target_distance, target_angle)

            time_elapsed = time() - start_time
            print 'Elapsed time (seconds):',time_elapsed
            total_time = total_time + time_elapsed
    print 'Total time (seconds):', total_time
    print 'Average (seconds):',total_time/(target_samples*len(glob.glob("*.npy")))
