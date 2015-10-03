from numpy import cos, sin, pi, arcsin


class GapFinding(object):

    """docstring for ClassName"""

    def __init__(self, safe_radius):
        self.safe_radius = safe_radius
        self.safe_gap = safe_radius**2 * 4
        self.distance_range = 15

    def polarToCartesian(self, distance, angle):

        self.number_of_readings = len(distance)
        # variable initialization
        i = 1
        x = [0 for reading in distance]
        y = [0 for reading in distance]

        x[0] = distance[0] * cos(angle[0])
        y[0] = distance[0] * sin(angle[0])

        while i < self.number_of_readings:
            x[i] = distance[i] * cos(angle[i])
            y[i] = distance[i] * sin(angle[i])
            i += 1

        self.first_obstacle_end = self.__findFirstGap(x, y)
        print 'first obstacle end:', self.first_obstacle_end

        return x, y

    def __findFirstGap(self, x, y):
        i = 1
        first_obstacle_end = -1

        while first_obstacle_end < 0 and i < self.number_of_readings:
            gap_x = x[i] - x[i - 1]
            gap_y = y[i] - y[i - 1]
            gap = gap_x**2 + gap_y**2
            if gap > self.safe_gap:
                return i - 1
            i += 1

        return -1

    def filterReadings(self, distances, angles):
        self.number_of_readings = len(distances)
        for i in range(self.number_of_readings):
            if distances[i] > .95 * self.distance_range:
                distances[i] = None
                angles[i] = None

        distances = [distance for distance in distances if distance != None]
        angles = [angle for angle in angles if angle != None]
        return distances, angles

    def findObstacleLimits(self, x, y):
        print 'number of readings:', len(x)

        def isSafe(index_1, index_2):
            gap_x = x[index_1] - x[index_2]
            gap_y = y[index_1] - y[index_2]
            gap = gap_x**2 + gap_y**2
            print 'the gap is', gap
            return gap > self.safe_gap

        number_of_obstacles = 0
        obstacle_end_index = self.first_obstacle_end
        self.obstacle_limits = []
        obstacle_start_index = obstacle_end_index + 1
        obstacle_end_index = None

        while not obstacle_end_index == (self.first_obstacle_end % self.number_of_readings):
            print 'explorriing new obstacle with start index:', obstacle_start_index
            number_of_obstacles += 1
            if number_of_obstacles > 4:
                print 'number of obstacles got more than 2'
                break
            self.obstacle_limits.append([obstacle_start_index])

            obstacle_vertex_index = obstacle_start_index
            obstacle_end_index = obstacle_start_index
            found_obstacle_end = False

            scanner = obstacle_vertex_index
            control_val = 0
            while not found_obstacle_end:
                scanner = (self.number_of_readings + self.first_obstacle_end) % self.number_of_readings
                while True:
                    print 'Evaluating indexes:', obstacle_vertex_index, scanner
                    if scanner == obstacle_vertex_index:
                        obstacle_end_index = scanner
                        found_obstacle_end = True
                        obstacle_start_index = (obstacle_vertex_index + 1) % self.number_of_readings
                        # break
                        break
                    if not isSafe(scanner, obstacle_vertex_index):
                        obstacle_vertex_index = (obstacle_vertex_index + 1) % self.number_of_readings
                        print 'found new vertex', obstacle_vertex_index
                        break
                    else:
                        scanner = (scanner - 1) % self.number_of_readings

                control_val += 1
                if control_val > 100:
                    print 'DIDN"T GET OUT OF THIS LOOP'
                    break
            print 'obstacle end is:', obstacle_end_index
            self.obstacle_limits[-1].append(obstacle_end_index)
        # self.obstacle_limits[1:]

    def showReadings(self, distances, angles):
        pass

    def defineSubgoals(self, distances, angles):
        number_of_obstacles = len(self.obstacle_limits) / 2
        subgoals_distance = subgoals_angle = [[None,None] for i in range(number_of_obstacles)]
        for i in range(number_of_obstacles):
            obstacle_vertices = range(self.obstacle_limits[2*i],self.obstacle_limits[2*i+1])
            for obstacle_vertex in obstacle_vertices:
                subgoal_clockwise = angles[obstacle_vertex] - arcsin(self.safe_radius/distances[obstacle_vertex])
                subgoal_counter_clockwise = angles[obstacle_vertex] + arcsin(self.safe_radius/distances[obstacle_vertex])
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
