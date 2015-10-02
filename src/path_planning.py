from numpy import cos, sin, pi


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

        self.first_obstacle_end = self.findFirstGap(x, y)
        print self.first_obstacle_end

        return x, y

    def findFirstGap(self, x, y):
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
        print 'input length:',len(x)

        def isSafe(index_1, index_2):
            gap_x = x[index_1] - x[index_2]
            gap_y = y[index_1] - y[index_2]
            gap = gap_x**2 + gap_y**2
            return gap > self.safe_gap

        number_of_obstacles = 0
        obstacle_end_index = self.first_obstacle_end
        print 'first obstacle end:',self.first_obstacle_end
        obstacle_limits = [None]
        obstacle_start_index = obstacle_end_index + 1
        while not obstacle_limits[-1] == (self.first_obstacle_end % self.number_of_readings):
            print 'in top while'
            number_of_obstacles += 1
            if number_of_obstacles > 4:
                print 'number of obstacles got more than 2'
                break
            obstacle_limits.append(obstacle_start_index)
            obstacle_vertex_index = obstacle_start_index
            print 'obstacle_start_index',obstacle_start_index
            obstacle_end_index = obstacle_start_index
            found_obstacle_end = False

            control_val = 0
            while not found_obstacle_end:
                print 'in the middle while'
                temp2 = (obstacle_vertex_index) % self.number_of_readings
                for index in range(self.number_of_readings + self.first_obstacle_end, obstacle_vertex_index, -1):
                    end_finder = (index) % self.number_of_readings
                    print 'in the for loop, end_finder:',end_finder
                    if not isSafe(end_finder, temp2):
                        # obstacle_limits.append(end_finder)
                        print 'found new vertex',obstacle_vertex_index
                        obstacle_start_index = (obstacle_vertex_index + 1)%self.number_of_readings
                        # found_obstacle_end = True
                        # obstacle_end_index = end_finder
                        break
                obstacle_vertex_index = (obstacle_vertex_index+1)%self.number_of_readings
                if obstacle_limits[-1] == temp2:
                    print 'found end of obstacle:', temp2
                    found_obstacle_end = True
                    obstacle_end_index = temp2
                    obstacle_start_index = (temp2 + 1)%self.number_of_readings
                    break
                control_val+=1
                if control_val>10:
                    print 'DIDN"T GET OUT OF THIS LOOP'
                    break
            obstacle_limits.append(obstacle_end_index)
            print 'obstacle_end_index', obstacle_end_index
        # obstacle_limits.append(self.first_obstacle_end)
        return obstacle_limits[1:]

    def isSafe(x_gap, y_gap, safe_radius, safe_gap):
        if x_gap > safe_radius or y_gap > safe_radius:
            return True
        if x_gap**2 + y_gap**2 > safe_gap:
            return True
        return False
