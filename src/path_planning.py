from numpy import cos, sin, pi, arcsin
nan = float('nan')


class GapFinding(object):

    """docstring for ClassName"""

    def __init__(self, safe_radius):
        self.safe_radius = safe_radius * 1.0
        self.safe_gap = self.safe_radius**2 * 4
        self.distance_range = 15.0
        self.readings_polar = []

    def setDistanceRange(self, distance_range):
        self.distance_range = distance_range

    def polarToCartesian(self):
        x_cordinate = [0.0 for reading in self.readings_polar]
        y_cordinate = [0.0 for reading in self.readings_polar]

        for i in range(len(self.readings_polar)):
            x_cordinate[i] = self.readings_polar[i][0] * cos(self.readings_polar[i][1])
            y_cordinate[i] = self.readings_polar[i][0] * sin(self.readings_polar[i][1])

        return x_cordinate, y_cordinate

    def __findFirstGap(self, x, y):
        i = 1
        first_obstacle_end = -1

        while first_obstacle_end < 0 and i < self.number_of_readings:

            gap_x = x[i] - x[i - 1]
            gap_y = y[i] - y[i - 1]
            gap = gap_x**2 + gap_y**2
            # print gap, i
            if gap > self.safe_gap:
                # print 'TTTTTTTTTThe gap is', gap, self.safe_gap, i - 1
                return i - 1
            i += 1

        return self.number_of_readings - 1

    def filterReadings(self, distances, angles):
        number_of_all_readings = len(distances)
        inner_bound = self.safe_radius * .7
        for i in range(number_of_all_readings):
            if distances[i] > .95 * self.distance_range or distances[i] < inner_bound:
                distances[i] = -1
                # angles[i] = []
            # else:
                # print 'found one'

        # print distances
        self.readings_polar = [[distances[i], angles[i]] for i in range(number_of_all_readings) if distances[i] != -1]
        # print len(self.readings_polar),self.readings_polar
        self.number_of_readings = len(self.readings_polar)
        # distances = [distance for distance in distances if distance != None]
        # angles = [angle for angle in angles if angle != None]
        # self.reading_distances = distances
        return distances, angles

    def findObstacleLimits(self, x, y):
        print 'mmmmmmmmmmmmmmmmmmmmmmmmmmm'
        # print len(x)
        self.first_obstacle_end = self.__findFirstGap(x, y)
        print self.first_obstacle_end
        # print 'number of readings_polar:', len(x)

        def isSafe(index_1, index_2):
            gap_x = x[index_1] - x[index_2]
            gap_y = y[index_1] - y[index_2]
            gap = gap_x**2 + gap_y**2
            # print 'the gap is', gap, index_1, index_2
            return gap > self.safe_gap

        number_of_obstacles = 0
        obstacle_end_index = self.first_obstacle_end
        self.obstacle_limits = []
        obstacle_start_index = (obstacle_end_index + 1) % self.number_of_readings
        obstacle_end_index = None

        while not obstacle_end_index == (self.first_obstacle_end % self.number_of_readings):
            # print 'explorriing new obstacle with start index:', obstacle_start_index
            number_of_obstacles += 1
            # if number_of_obstacles > 4:
            # print 'number of obstacles got more than 2'
            #     break
            self.obstacle_limits.append([obstacle_start_index])

            obstacle_vertex_index = obstacle_start_index
            obstacle_end_index = obstacle_start_index
            found_obstacle_end = False

            scanner = obstacle_vertex_index
            # control_val = 0
            while not found_obstacle_end:
                scanner = (self.number_of_readings + self.first_obstacle_end) % self.number_of_readings
                while True:
                    # print 'Evaluating indexes:', obstacle_vertex_index, scanner
                    if scanner == obstacle_vertex_index:
                        obstacle_end_index = scanner
                        found_obstacle_end = True
                        obstacle_start_index = (obstacle_vertex_index + 1) % self.number_of_readings
                        # break
                        break
                    if not isSafe(scanner, obstacle_vertex_index):
                        obstacle_vertex_index = (obstacle_vertex_index + 1) % self.number_of_readings
                        # print 'found new vertex', obstacle_vertex_index
                        break
                    else:
                        scanner = (scanner - 1) % self.number_of_readings

                # control_val += 1
                # if control_val > 100:
                # print 'DIDN"T GET OUT OF THIS LOOP'
                #     break
            # print 'obstacle end is:', obstacle_end_index
            self.obstacle_limits[-1].append(obstacle_end_index)
        # self.obstacle_limits[1:]

    def showReadings(self, distances, angles):
        pass

    def defineSubgoals(self, distances, angles):
        number_of_obstacles = len(self.obstacle_limits)
        subgoals_angle = [[[], []] for i in range(number_of_obstacles)]
        subgoals_distance = subgoals_angle
        subgoal_temp = [[], []]
        for i in range(number_of_obstacles):
            # print 'evaluating new obstacle'
            obstacle_vertices = range(self.obstacle_limits[i][0], self.obstacle_limits[i][1] + 1)
            for obstacle_vertex in obstacle_vertices:
                # print angles[obstacle_vertex], self.safe_radius, distances[obstacle_vertex], arcsin(self.safe_radius / distances[obstacle_vertex])
                subgoal_temp[0] = angles[obstacle_vertex] - arcsin(self.safe_radius / distances[obstacle_vertex])
                subgoal_temp[1] = angles[obstacle_vertex] + arcsin(self.safe_radius / distances[obstacle_vertex])

                subgoals_angle[i][0] = min(subgoal_temp[0], subgoals_angle[i][0])
                subgoals_angle[i][1] = max(subgoal_temp[1], subgoals_angle[i][1])

        return subgoals_angle

                # subgoals_angle(i) = min([subgoals_angle(i),subgoal])

    def selectSubgoal(self, distances, angles, target_distnce, target_theta):
        x, y = self.polarToCartesian(distances, angles)
        self.findObstacleLimits(x, y)
        for i in range(self.obstacle_limits / 2):
            diff = wrapTo180()
        pass

    def isSafe(x_gap, y_gap, safe_radius, safe_gap):
        if x_gap > safe_radius or y_gap > safe_radius:
            return True
        if x_gap**2 + y_gap**2 > safe_gap:
            return True
        return False


class SimpleGapFinder(object):

    def __init__(self, safe_radius):
        self.safe_radius = safe_radius * 1.0
        self.safe_gap = self.safe_radius**2 * 4
        self.distance_range = 15.0
        self.readings_polar = []

    def setDistanceRange(self, distance_range):
        self.distance_range = distance_range

    def polarToCartesian(self):
        x_cordinate = [0.0 for reading in self.readings_polar]
        y_cordinate = [0.0 for reading in self.readings_polar]

        for i in range(len(self.readings_polar)):
            x_cordinate[i] = self.readings_polar[i][0] * cos(self.readings_polar[i][1])
            y_cordinate[i] = self.readings_polar[i][0] * sin(self.readings_polar[i][1])

        return x_cordinate, y_cordinate

    def filterReadings(self, distances, angles):
        number_of_all_readings = len(distances)
        inner_bound = self.safe_radius * .7
        for i in range(number_of_all_readings):
            if distances[i] > .95 * self.distance_range or distances[i] < inner_bound:
                distances[i] = -1
                # angles[i] = []
            # else:
                # print 'found one'

        # print distances
        self.readings_polar = [[distances[i], angles[i]] for i in range(number_of_all_readings) if distances[i] != -1]
        # print len(self.readings_polar),self.readings_polar
        self.number_of_readings = len(self.readings_polar)
        # distances = [distance for distance in distances if distance != None]
        # angles = [angle for angle in angles if angle != None]
        # self.reading_distances = distances
        return distances, angles

    def findGaps(self):
        print self.readings_polar
        possible_travel = [0.0 for i in range(len(self.readings_polar))]
        for i in range(len(self.readings_polar)):
            possible_travel[i] = self.readings_polar[i][0]
            for j in range(i - 1, -1, -1):
                angular_difference = self.readings_polar[i][1] - self.readings_polar[j][1]
                distance = self.readings_polar[i][0] - self.readings_polar[j][0]
                if  angular_difference > pi / 2:
                    print 'breakings'
                    break
                if abs(distance)>self.safe_radius*2:
                    continue
                elif 

            for j in range(i + 1, self.number_of_readings):
                if self.readings_polar[j][1]-self.readings_polar[j][1] > pi/2:
                    print 'breaking'
                    break
                if abs(self.readings_polar[i][0] - self.readings_polar[j][0])>self.safe_radius*2:
                    continue

