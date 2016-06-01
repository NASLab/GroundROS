from numpy import cos, sin, pi, sqrt, arctan2
from rotation import wrapAnglePi
import miscFunctions as misc
# from time import time
nan = float('nan')
inf = float('inf')


class PathPlanningError(Exception):

    def __init__(self, message, errors=-1):
        super(PathPlanningError, self).__init__(message)


class GapFinder(object):

    def __init__(self, safe_radius):
        # These are configurations and can be changed as needed
        self.safe_radius = safe_radius * 1.0
        self.danger_radius = safe_radius * .9
        self.lidar_offset = .23
        self.max_range = 4

        # DO NOT CHANGE THE BELOW PARAMETERS
        self.safe_gap = self.safe_radius**2 * 4
        self.number_of_readings = 0
        self.is_in_dange_zone = False
        self.readings_polar = []
        self.subgoals = []


    def polarToCartesian(self):
        x_cordinate = [reading[0] * cos(reading[1]) for reading in self.readings_polar]
        y_cordinate = [reading[0] * sin(reading[1]) for reading in self.readings_polar]

        return x_cordinate, y_cordinate

    def filterReadings(self, distances, angles, inner_bound=.1):
        '''Removes the readings that fall within the 'inner_bound' input. It also finds the nearest reading after filtering. 'distances' and 'angles' inputs are the lidar readings.'''

        number_of_unfiltered_readings = len(distances)
        angles_trasfered = [[]] * number_of_unfiltered_readings
        distances_trasfered = [[]] * number_of_unfiltered_readings
        self.minimum_reading = inf
        self.minimum_reading_angle = []

        for i in range(number_of_unfiltered_readings):
            if angles[i] < -2.0 or angles[i] > 2.0 or distances[i] < inner_bound or distances[i] > self.max_range:
                distances[i] = -1
            else:
                x = distances[i] * cos(angles[i]) + self.lidar_offset
                y = distances[i] * sin(angles[i])
                distances_trasfered[i] = sqrt(x**2 + y**2)
                angles_trasfered[i] = arctan2(y, x)

                if distances_trasfered[i] < self.minimum_reading:
                    self.minimum_reading = distances_trasfered[i]
                    self.minimum_reading_angle = angles_trasfered[i]

        print self.minimum_reading , self.danger_radius
        if self.minimum_reading < self.danger_radius:
            self.is_in_dange_zone = True
            print misc.fail('Something is too close to the robot')
        else:
            self.is_in_dange_zone = False

        self.readings_polar = [[distances_trasfered[i], angles_trasfered[i]] for i in range(number_of_unfiltered_readings) if distances[i] != -1]
        self.number_of_readings = len(self.readings_polar)

    def getMinimumReading(self):
        return self.minimum_reading, self.minimum_reading_angle

    # def findGaps(self):
    #     d = self.safe_radius**2
    #     self.possible_travel = [reading[0] - self.safe_radius for reading in self.readings_polar]
    #     self.is_in_dange_zone = False

    #     for i in range(self.number_of_readings):

    #         for j in range(i - 1, -1, -1):
    #             angular_difference = self.readings_polar[i][1] - self.readings_polar[j][1]
    #             if angular_difference > pi / 2:
    #                 break
    #             if self.readings_polar[j][0] > self.readings_polar[i][0]:
    #                 continue
    #             c = (self.readings_polar[j][0] * sin(angular_difference))**2
    #             a = (self.readings_polar[j][0] * cos(angular_difference))
    #             if c < d:
    #                 if self.readings_polar[j][0] > self.safe_radius:
    #                     self.possible_travel[i] = min(self.possible_travel[i], a - sqrt(d - c))
    #                 else:
    #                     self.possible_travel[i] = 0.0
    #                     self.is_in_dange_zone = True

    #         for j in range(i + 1, self.number_of_readings):
    #             angular_difference = self.readings_polar[j][1] - self.readings_polar[i][1]
    #             if angular_difference > pi / 2:
    #                 break
    #             if self.readings_polar[j][0] > self.readings_polar[i][0]:
    #                 continue
    #             c = (self.readings_polar[j][0] * sin(angular_difference))**2
    #             a = (self.readings_polar[j][0] * cos(angular_difference))
    #             if c < d:
    #                 if self.readings_polar[j][0] > self.safe_radius:
    #                     self.possible_travel[i] = min(self.possible_travel[i], a - sqrt(d - c))
    #                 else:
    #                     self.possible_travel[i] = 0.0
    #                     self.is_in_dange_zone = True

    def findGaps(self):
        d = self.safe_radius**2
        self.possible_travel = [reading[0] - self.safe_radius for reading in self.readings_polar]

        for i in range(self.number_of_readings):

            for j in range(i - 1, -1, -1):
                angular_difference = self.readings_polar[i][1] - self.readings_polar[j][1]
                if angular_difference > pi / 2:
                    break
                if self.readings_polar[j][0] > self.readings_polar[i][0]:
                    continue
                c = (self.readings_polar[j][0] * sin(angular_difference))**2
                a = (self.readings_polar[j][0] * cos(angular_difference))
                if c < d:
                    # if self.readings_polar[j][0] > self.safe_radius:
                    self.possible_travel[i] = min(self.possible_travel[i], a - sqrt(d - c))
                    # else:
                    #     self.possible_travel[i] = 0

            for j in range(i + 1, self.number_of_readings):
                angular_difference = self.readings_polar[j][1] - self.readings_polar[i][1]
                if angular_difference > pi / 2:
                    break
                if self.readings_polar[j][0] > self.readings_polar[i][0]:
                    continue
                c = (self.readings_polar[j][0] * sin(angular_difference))**2
                a = (self.readings_polar[j][0] * cos(angular_difference))
                if c < d:
                    # if self.readings_polar[j][0] > self.safe_radius:
                    self.possible_travel[i] = min(self.possible_travel[i], a - sqrt(d - c))
                    # else:
                    #     self.possible_travel[i] = 0
    # def findGaps(self):
    #     d = self.safe_radius**2
    #     self.possible_travel = [reading[0] - self.safe_radius for reading in self.readings_polar]

    #     for i in range(self.number_of_readings):

    #         for j in range(i - 1, -1, -1):
    #             angular_difference = self.readings_polar[i][1] - self.readings_polar[j][1]
    #             if angular_difference > pi / 2:
    #                 break
    #             if self.readings_polar[j][0] > self.readings_polar[i][0]:
    #                 continue
    #             c = (self.readings_polar[j][0] * sin(angular_difference))**2
    #             a = (self.readings_polar[j][0] * cos(angular_difference))
    #             if c < d:
    #                 if self.readings_polar[j][0] > self.safe_radius:
    #                     self.possible_travel[i] = min(self.possible_travel[i], a - sqrt(d - c))
    #                 else:
    #                     self.possible_travel[i] = 0.0

    #         for j in range(i + 1, self.number_of_readings):
    #             angular_difference = self.readings_polar[j][1] - self.readings_polar[i][1]
    #             if angular_difference > pi / 2:
    #                 break
    #             if self.readings_polar[j][0] > self.readings_polar[i][0]:
    #                 continue
    #             c = (self.readings_polar[j][0] * sin(angular_difference))**2
    #             a = (self.readings_polar[j][0] * cos(angular_difference))
    #             if c < d:
    #                 if self.readings_polar[j][0] > self.safe_radius:
    #                     self.possible_travel[i] = min(self.possible_travel[i], a - sqrt(d - c))
    #                 else:
    #                     self.possible_travel[i] = 0.0

    def getFrontTravel(self, distances, angles, target_reading = nan):
        number_of_unfiltered_readings = len(distances)
        angles_trasfered = [[]] * number_of_unfiltered_readings
        distances_trasfered = [[]] * number_of_unfiltered_readings
        self.minimum_reading = []
        self.minimum_reading_angle = []

        inner_bound = .05

        # for i in range(number_of_unfiltered_readings):
        #     if distances[i] < inner_bound:
        # print '????????????????',distances[i],i
        #         distances[i] = -1
        #     elif distances[i]>=0 and distances[i]<self.minimum_reading:
        #         self.minimum_reading = distances[i]
        #         self.minimum_reading_angle = angles[i]

        # print angles
        for i in range(number_of_unfiltered_readings):
            if distances[i] < inner_bound:
                distances[i] = -1
            else:
                x = distances[i] * cos(angles[i]) + self.lidar_offset
                y = distances[i] * sin(angles[i])
                distances_trasfered[i] = sqrt(x**2 + y**2)
                angles_trasfered[i] = arctan2(y, x)
                # print angles_trasfered[40]
                if distances[i] >= 0 and distances[i] < self.minimum_reading:
                    self.minimum_reading = distances[i]
                    self.minimum_reading_angle = angles[i]

        self.readings_polar = [[distances_trasfered[i], angles_trasfered[i]] for i in range(number_of_unfiltered_readings) if distances[i] != -1]
        self.number_of_readings = len(self.readings_polar)
        # print self.readings_polar[40]
        # for i in range(self.number_of_readings):
        #     print self.readings_polar[i]
        d = self.safe_radius**2
        self.possible_travel = [reading[0] - self.safe_radius for reading in self.readings_polar]

        # for i in range(self.number_of_readings):
        if target_reading != target_reading:
            target_reading = 0

        for j in range(target_reading - 1, -1, -1):
            angular_difference = self.readings_polar[target_reading][1] - self.readings_polar[j][1]
            if angular_difference > pi / 2:
                break
            if self.readings_polar[j][0] > self.readings_polar[target_reading][0]:
                continue
            c = (self.readings_polar[j][0] * sin(angular_difference))**2
            a = (self.readings_polar[j][0] * cos(angular_difference))
            if c < d:
                # if self.readings_polar[j][0] > self.safe_radius:
                self.possible_travel[target_reading] = min(self.possible_travel[target_reading], a - sqrt(d - c))
                # else:
                #     self.possible_travel[target_reading] = 0

        for j in range(target_reading + 1, self.number_of_readings):
            angular_difference = self.readings_polar[j][1] - self.readings_polar[target_reading][1]
            if angular_difference > pi / 2:
                break
            if self.readings_polar[j][0] > self.readings_polar[target_reading][0]:
                continue
            c = (self.readings_polar[j][0] * sin(angular_difference))**2
            a = (self.readings_polar[j][0] * cos(angular_difference))
            if c < d:
                # if self.readings_polar[j][0] > self.safe_radius:
                self.possible_travel[target_reading] = min(self.possible_travel[target_reading], a - sqrt(d - c))
                # else:
                #     self.possible_travel[i] = 0
        return self.possible_travel[target_reading]
        
        for i in range(len(self.readings_polar)):
            if self.readings_polar[i][1] > 0:
                return min(self.possible_travel[i], self.possible_travel[i - 1])

    def findSubgoalsDEBUG(self):
        self.subgoals = [0, self.number_of_readings - 1]  # Initial subgoals
        for i in range(self.number_of_readings - 1):
            if self.possible_travel[i] > 0 and self.possible_travel[i + 1] > 0:
                possible_travel_change = self.possible_travel[i + 1] - self.possible_travel[i]  # Difference between each two neighboring possible travel
                if possible_travel_change > self.safe_radius:
                    # self.subgoals.append(i)
                    self.subgoals.append(i + 1)
                elif -possible_travel_change > self.safe_radius:
                    self.subgoals.append(i)
                    # self.subgoals.append(i + 1)

        # return self.subgoals

    def findSubgoals(self):   # BACKUP
        self.subgoals = [0, self.number_of_readings - 1]  # Initial subgoals
        for i in range(self.number_of_readings - 1):
            possible_travel_change = self.possible_travel[i + 1] - self.possible_travel[i]  # Difference between each two neighboring possible travel
            if possible_travel_change > self.safe_radius:
                self.subgoals.append(i + 1)
            elif -possible_travel_change > self.safe_radius:
                self.subgoals.append(i)

    def selectSubgoal(self, distance, angle):
        # checks which subgoal is closer to the target and returns its index number
        best_subgoal = []
        best_distance = []

        for subgoal in self.subgoals:
            distance_to_target_sq = distance**2 + self.possible_travel[subgoal]**2 - 2 * distance * \
                self.possible_travel[subgoal] * cos(self.readings_polar[subgoal][1] - angle)
            predicted_trajectory_length = sqrt(distance_to_target_sq)  # + self.possible_travel[subgoal]
            # print 'distance of',subgoal,'is',distance**2 , self.possible_travel[subgoal]**2 ,  distance , self.possible_travel[subgoal] , cos(self.readings_polar[subgoal][1] - angle),predicted_trajectory_length
            # print 'asserting',distance_to_target_sq < best_distance
            # print 'subgoal',subgoal,'is this far',self.possible_travel[subgoal]
            if (predicted_trajectory_length < best_distance or not best_distance):
                best_subgoal = subgoal
                best_distance = predicted_trajectory_length

        # print 'subgoals', self.subgoals
        # print 'best subgoal', best_subgoal
        # print 'best subgoal is this much far',self.possible_travel[best_subgoal]
        return best_subgoal

    # def selectSubgoal(self, distance, angle):
    #     # checks which subgoal is closer to the target and returns its index number
    #     best_subgoal = 0
    #     best_distance = distance**2 + self.possible_travel[0]**2 - 2 * distance * self.possible_travel[0] * cos(self.readings_polar[0][1] - angle)

    #     for subgoal in self.subgoals[1:]:
    #         distance_to_target_sq = distance**2 + self.possible_travel[subgoal]**2 - 2 * distance * \
    #             self.possible_travel[subgoal] * cos(self.readings_polar[subgoal][1] - angle)
    #         if distance_to_target_sq < best_distance and self.possible_travel[subgoal] > self.safe_radius:
    #             best_subgoal = subgoal
    #             best_distance = distance_to_target_sq

    #     return best_subgoal

    def isObstacleInTheWay(self, distance, angle):
        # if something is too close to robot return 'danger'
        if self.is_in_dange_zone:
            return 'danger', 0

        if len(self.possible_travel) is 0:  # there are no obstacles at all
            return 'safe', distance

        # check if all the obstacles are on one side only
        # if so it is 'safe'. BUG: it returns 'safe' if on one side only even 
        # if very close to the obstacle
        nearest_reading = nan
        if self.readings_polar[0][1] < angle:
            for i in range(self.number_of_readings):
                if self.readings_polar[i][1] - angle > 0:
                    nearest_reading = i
                    break
            if nearest_reading != nearest_reading:
                print 'one sided'
                return 'safe', distance
        else:
            print 'one sided'
            return 'safe', distance

        print nearest_reading

        safe_travel = max(self.possible_travel[nearest_reading], self.possible_travel[nearest_reading - 1])
        # if distance to target is less than possible travel in that direction
        # then return 'safe'
        if distance < safe_travel:
            return 'safe', distance
        # if distance to target is larger than safe travel and samller than lidar reading
        elif distance > safe_travel and distance < min(self.possible_travel[nearest_reading], self.possible_travel[nearest_reading - 1]):
            return 'close_to_obstacle',safe_travel
        # if distance to target is larger than lidar reading then return 'collision'
        else:
            return 'collision', safe_travel

    # def isObstacleInTheWayBACKUP(self, distance, angle):
    #     nearest_reading = 0
    #     for i in range(self.number_of_readings):
    #         if self.readings_polar[i][1] - angle > 0:
    #             nearest_reading = i
    #             break

    #     safe_travel = min(self.possible_travel[nearest_reading], self.possible_travel[nearest_reading - 1])
    #     self.maximum_travel_to_target = safe_travel
    #     if distance < safe_travel:
    #         return 'safe', distance
    #     elif distance < safe_travel + self.safe_radius and nearest_reading != 0:
    #         return 'close_to_obstacle', safe_travel
    #     else:
    #         return 'collision', safe_travel

    def planPath(self, distance, angle):
        self.findGaps()
        angle = wrapAnglePi(angle)
        environment_state, max_travel = self.isObstacleInTheWay(distance, angle)
        print environment_state
        if environment_state is 'safe':
            return distance, angle
        elif environment_state is 'collision':
            self.findSubgoals()
            best_subgoal = self.selectSubgoal(distance, angle)
            return self.possible_travel[best_subgoal], self.readings_polar[best_subgoal][1]
        elif environment_state is 'close_to_obstacle':
            print misc.bold(misc.warn('WARNING: Target is too close to an obstacle!!!'))
            print environment_state
            return max_travel, angle
        elif self.is_in_dange_zone:
            print misc.bold(misc.fail('DANGER: Obstacle to close to robot!!!'))
            return 0, 0
        else:
            raise PathPlanningError.ImplementationError('Something wrong in planPath method of' + __name__)

    def pathToGlobalTarget(self, target_pos, agent_pos):
        diff_x = target_pos[0] - agent_pos[0]
        diff_y = target_pos[1] - agent_pos[1]
        distance = sqrt(diff_x**2 + diff_y**2)
        angle = arctan2(diff_y, diff_x) - agent_pos[2]
        subgoal_distance, subgoal_angle = self.obstacleAvoidance(distance, -angle)
