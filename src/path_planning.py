from numpy import cos, sin, pi


class GapFinding(object):

    """docstring for ClassName"""

    def __init__(self, safe_radius):
        self.safe_radius = safe_radius
        self.safe_gap = safe_radius**2 * 4

    def polarToCartesian(self, distance, angle):
        # variable initialization
        i = 1
        self.number_of_readings = len(distance)
        x = [0 for reading in distance]
        y = [0 for reading in distance]

        x[0] = distance[0] * cos(angle[0])
        y[0] = distance[0] * sin(angle[0])

        while i < self.number_of_readings:
            x[i] = distance[i] * cos(angle[i])
            y[i] = distance[i] * sin(angle[i])
            i += 1

        self.first_obstacle_end = self.findFirstGap(x, y)

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

        return None
    # def isSafe(self, index_1, index_2):
    #     gap_x = x[index_1] - x[index_2]
    #     gap_y = y[index_1] - y[index_2]
    #     gap = gap_x**2 + gap_y**2
    #     return gap > self.safe_gap

    def findObstacleLimits(self, x, y):
        def isSafe(index_1, index_2):
            gap_x = x[index_1] - x[index_2]
            gap_y = y[index_1] - y[index_2]
            gap = gap_x**2 + gap_y**2
            return gap > self.safe_gap

        number_of_obstacles = 0
        obstacle_end_index = self.first_obstacle_end
        obstacle_limits = []
        while obstacle_end_index != self.first_obstacle_end or number_of_obstacles == 0:
            number_of_obstacles += 1
            obstacle_start_index = obstacle_end_index + 1
            obstacle_limits.append(obstacle_start_index)
            obstacle_vertex_index = obstacle_start_index
            obstacle_end_index = obstacle_start_index
            found_obstacle_end = False

            while (not found_obstacle_end) and obstacle_vertex_index < self.number_of_readings:
                pass
        obstacle_limits.append(self.first_obstacle_end)
        return None

    def isSafe(x_gap, y_gap, safe_radius, safe_gap):
        if x_gap > safe_radius or y_gap > safe_radius:
            return True
        if x_gap**2 + y_gap**2 > safe_gap:
            return True
        return False
