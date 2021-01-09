import math
import random
from enum import Enum, unique, auto

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.pyplot import figure
import numpy as np

from DSM_Paths.DsmParser import DSMParcer


@unique
class PathType(Enum):
    MAP_ROAM = auto()
    AREA_EXPLORE = auto()


@unique
class ConstraintType(Enum):
    TIME = auto()
    DISTANCE = auto()


class PathGenerator:
    """
    This is the main class. To create paths, create an instance of a PathGenerator with the desired parameters then,
    call gen_paths with the constraints of your choosing.
    """
    SAMPLE_RATE = 0.6  # The drone way point minimal distance coefficient is 0.6 * velocity.
    MAX_STRIDE_LEN = 6  # Allowing the largest stride length to be 6 meters.
    MIN_STRIDE_LEN = 2  # Allowing the smallest stride length to be 2 meters.

    def __init__(self, velocity, flight_height, dsm=None, origin=(0.0, 0.0, 0.0), map_dimensions=(0, 0),
                 pixel_dimensions=(0.0, 0.0), stride_multiplier=1.0):
        """
        Creates an instance of a PathGenerator.

         Parameters
        ----------
        velocity : float
            The drone's velocity in meters/second.
        flight_height : float
            The flight's altitude in meters.
        dsm : List of Lists of floats
            The DSM map. We assume it is squared.
        origin: tuple of floats TODO: make sure this description is correct.
            A distance vector between the world map's origin and the dsm map's origin. len(origin) must be 3.
        map_dimensions: tuple of floats
            A two value tuple: (<row_number>, <column_number>).
        pixel_dimensions: tuple of floats
            A two value tuple: (<Wdx>, <Wdy>) where Wdx is the distance (in meters) one stride in the x axis direction
            ((x, y) -> (x + 1, y)) represents and Wdy is the distance (in meters) one stride in the y axis
            direction represents.
        stride_multiplier: float
            Adjusts the strides' length. default stride length is 0.6 * velocity must be > 0.
        """
        self._velocity = velocity
        self._flight_height = flight_height
        if stride_multiplier > 0:
            self._stride_length = self.SAMPLE_RATE * velocity * stride_multiplier
        else:
            self._stride_length = self.SAMPLE_RATE * velocity
        self.__adjust_stride()
        if dsm is not None and origin is not None and map_dimensions is not None and pixel_dimensions is not None\
                and len(origin) == 3 and len(map_dimensions) == 2 and len(pixel_dimensions) == 2:
            self._dsm = dsm
            self._org_vector = origin
            self._map_dim = map_dimensions
            self._pixel_dim = pixel_dimensions
            self.__process_map()
            # These fields are used to determine the boundaries of the map (where the object appear).
            self._x_lower_bound, self._y_lower_bound, self._x_upper_bound, self._y_upper_bound = \
                self.__bound_map_main_area()
        else:
            self._dsm = None
            print('Need to initiate map using init_map before we start.')

    def init_map(self, input_path=None, file_name=None, save_tif=False):
        """
        This method loads the DSM map to an instance using a binary file representing it. The binary file must be in
        a directory named 'BinFiles' inside the 'input_path' directory.
        If the save_tif argument is True the dsm map will be saved as a .tif image in a folder named DSMout under the
        name DSM0.tif.
        """
        if input_path is not None and file_name is not None:
            _, _, _, x_org, y_org, z_org, Wx, Wy, dWx, dWy, self._dsm = DSMParcer(input_path, file_name, save_tif)
            self._org_vector = (x_org, y_org, z_org)
            self._map_dim = (Wx, Wy)
            self._pixel_dim = (dWx, dWy)
            self.__process_map()
            # Updating the map boundary values.
            self._x_lower_bound, self._y_lower_bound, self._x_upper_bound, self._y_upper_bound = \
                self.__bound_map_main_area()

    def gen_paths(self, flag: ConstraintType, constraint: float, path_type: PathType, start_location=None, path_num=1,
                  to_print=False, weight=1.0):
        """
         Parameters
        ---------
        flag : ConstraintType
            Will hold either ConstraintType.DISTANCE for distance constraint or ConstraintType.TIME for time constraint.
        constraint : float
            Either the paths' length in meters for distance constrained paths or the travel time for time
            in seconds constrained paths.
        path_type : PathType
            The kind of path generating algorithm used.
            Will hold either PathType.AREA_EXPLORE for probability random path and PathType.MAP_ROAM for A*
            generated path to random spots.
        start_location : list
            The path's first point. If None is passed or the given point is not valid a random one will
            be generated.
        path_num : int
            The number of paths wanted.
        to_print : bool
            Pass True to print the resulting paths over the dsm map.
        weight : float
            Pass the weight (>= 1.0) for the heuristic value of weighted A*. If the weight given to this function is
            epsilon, and the original heuristic function is h_0(x) so the value used by A* would be epsilon*h_0(x).
            Using weighted A* promises a solution of weight times the optimal path between the chosen points.
        """
        if self._dsm is not None:
            need_new_start_pos = start_location is None or not self._can_fly_over_in_bound(start_location)
            if flag != ConstraintType.DISTANCE and flag != ConstraintType.TIME:
                print('Wrong flag option!')
                return []
            if constraint <= 0:
                print('Constrain should be positive')
                return []
            cost_per_meter = 1
            if flag == ConstraintType.TIME:
                cost_per_meter /= self._velocity
            paths = []
            if path_type == PathType.AREA_EXPLORE:
                for i in range(path_num):
                    if need_new_start_pos:
                        start_location = self._gen_random_point_under_constraints()
                    paths += [self.__gen_path_probability(start_location=start_location, max_cost=constraint,
                                                          cost_per_meter=cost_per_meter)]
            elif path_type == PathType.MAP_ROAM:
                for i in range(path_num):
                    if need_new_start_pos:
                        start_location = self._gen_random_point_under_constraints()
                    paths += [self.__gen_path_weighted_a_star(start_location=start_location, max_cost=constraint,
                                                              cost_per_meter=cost_per_meter, weight=weight)]

            else:
                print('Path type is not one of the correct path generating ways.')
                print('This method except PathType.AREA_EXPLORE and PathType.MAP_ROAM.')
                return []
            if to_print:
                for path in paths:
                    self.print_path(path)
            return paths
        else:
            print('Need to initiate map using init_map before we start.')
            return []

    def map_zoom_in(self, multiplier: int):
        """
        Enhancing the dsm resolution by the given multiplier.
         Parameters
        ---------
        multiplier : int
             The enhance multiplier. Must be > 0.
        """
        if self._dsm is not None:
            if multiplier <= 0:
                return  # Not allowing a negative enhance multiplier.
            self._map_dim = (self._map_dim[0] * multiplier, self._map_dim[1] * multiplier)
            new_dsm = [[0 for j in range(self._map_dim[1])] for i in range(self._map_dim[0])]
            for x in range(0, self._map_dim[0]):
                for y in range(0, self._map_dim[1]):
                    new_dsm[x][y] = self._dsm[int(x / multiplier)][int(y / multiplier)]
            self._pixel_dim = (self._pixel_dim[0] / float(multiplier), self._pixel_dim[1] / float(multiplier))
            self._dsm = np.array(new_dsm)
            # These fields are used to determine the boundaries of the map (where the object appear).
            self._x_lower_bound *= multiplier
            self._y_lower_bound *= multiplier
            self._x_upper_bound = math.ceil(self._x_upper_bound * multiplier)
            self._y_upper_bound = math.ceil(self._y_upper_bound * multiplier)
            # self._x_lower_bound, self._y_lower_bound, self._x_upper_bound, self._y_upper_bound = \
            #     self.__bound_map_main_area()
        else:
            print('Need to initiate the dsm map using init_map before using the zoom in method.')

    def map_zoom_out(self, multiplier: int):
        """
        Downgrading the dsm resolution by the given multiplier.
         Parameters
        ---------
        multiplier : int
            The pixel merge multiplier. Must be > 0.

         Notes
        ---------
        Decreasing resolution might cause information loss as we use max value substitution.
        Decreasing resolution using a multiplier value that the map's sides are not divisible by will cause
        representation of parts of the world map that are not included in the given dsm.
        """
        if self._dsm is not None:
            prev_dim = self._map_dim
            new_x_dim = int(prev_dim[0] / multiplier) if prev_dim[0] % multiplier == 0 else\
                1 + int(prev_dim[0] / multiplier)
            new_y_dim = int(prev_dim[1] / multiplier) if prev_dim[1] % multiplier == 0 else \
                1 + int(prev_dim[1] / multiplier)
            new_dsm = [[0 for j in range(new_y_dim)] for i in range(new_x_dim)]
            for x in range(0, new_x_dim):
                for y in range(0, new_y_dim):
                    maxi = -np.inf
                    for i in range(0, multiplier):
                        for j in range(0, multiplier):
                            x_idx = x * multiplier + i
                            y_idx = y * multiplier + j
                            val = -np.inf if x_idx >= self._map_dim[0] or y_idx >= self._map_dim[1] \
                                else self._dsm[x_idx][y_idx]
                            maxi = max(maxi, val)
                    new_dsm[x][y] = maxi
            self._map_dim = (new_x_dim, new_y_dim)
            dWx = self._pixel_dim[0] * multiplier
            dWy = self._pixel_dim[1] * multiplier
            self._pixel_dim = (dWx, dWy)
            self._dsm = np.array(new_dsm)
            # These fields are used to determine the boundaries of the map (where the object appear).
            self._x_lower_bound = int(self._x_lower_bound / multiplier)
            self._y_lower_bound = int(self._y_lower_bound / multiplier)
            self._x_upper_bound = int(self._x_upper_bound / multiplier)
            self._y_upper_bound = int(self._y_upper_bound / multiplier)
            # self._x_lower_bound, self._y_lower_bound, self._x_upper_bound, self._y_upper_bound = \
            #    self.__bound_map_main_area()
        else:
            print('Need to initiate the dsm map using init_map before using the zoom out method.')

    def print_path(self, path=None, path_color='r', path_style='--'):
        """
        Outputs the map with the given path on it.

         Parameters
        ----------
        path : A 2D list.
            The points that makes the path. points[i] holds the i's point of the path.
        path_color : str
            A single character indicating the paths color. The default value will create a red path.
        path_style : str
            A string that represent the graphing style. The default is dashed line.

         See Also
        --------
        Examples for path_color and path_style can be found in the plot description
        here: https://matplotlib.org/2.1.1/api/_as_gen/matplotlib.pyplot.plot.html
        """
        if self._dsm is not None:
            figure(num=None, figsize=(8, 6))
            plot_style = path_color + path_style
            plt.figure(1)
            plt.imshow(self._dsm)
            if path is not None and len(path) >= 2:
                points = np.array(path)
                plt.plot(points[:, 1], points[:, 0], plot_style)
                # printing the path's start and end points with their descriptions on the map
                start_patch = patches.Patch(color='cyan', label='Path\'s start')
                end_patch = patches.Patch(color='blue', label='Path\'s end')
                plt.legend(handles=[start_patch, end_patch], bbox_to_anchor=(1.32, 1), loc='upper right')
                plt.plot(points[0][1], points[0][0], 'c.')
                plt.plot(points[-1][1], points[-1][0], 'b.')

            plt.show()
        else:
            print('Need to initiate map using init_map before we start.')

    # TODO: delete after testing
    def print_map_sizes(self):
        print(f"stride_length: {self._stride_length}")
        print(f"map dimensions: {self._map_dim}")
        print(f"pixel dimensions: {self._pixel_dim}")
        print("map boundaries:")
        print(f"    {self._x_lower_bound} <= x <= {self._x_upper_bound}")
        print(f"    {self._y_lower_bound} <= y <= {self._y_upper_bound}")

    def calc_path_distance(self, path: list):
        """
        This method gets a path and returns the path's distance on the instance's map.
        """
        if path is not None:
            distance = 0
            for i in range(len(path) - 1):
                distance += self.__real_life_distance(path[i], path[i + 1])
            return distance
        return 0

    def calc_path_travel_time(self, path: list):
        """
        This method gets a path and returns the path's travel duration on the instance's map.
        """
        if path is not None:
            travel_time = 0
            for i in range(len(path) - 1):
                travel_time += self.__real_life_distance(path[i], path[i + 1]) / self._velocity
            return travel_time
        return 0

    def __gen_path_probability(self, start_location: list, max_cost: float, cost_per_meter: float):
        """
        This method calculate a path with cost = cost max_cost where a move's cost is calculated as the distance
        between the two neighbors (sqrt(2) for diagonal neighbors and 1 for non-diagonal ones) multiplied by pixel_cost.
        """
        cur_pos = start_location
        path = [cur_pos]
        directions = [[-1, 0], [0, -1], [1, 0], [0, 1], [1, 1], [1, -1], [-1, 1], [-1, -1]]
        last_move = 0
        last_legal_move = 0
        path_cost = 0
        while True:
            new_pos_option = [sum(x) for x in zip(directions[last_move], cur_pos)]
            if self._can_fly_over_in_bound(new_pos_option) and random.randrange(0, 100) <= 96:
                last_legal_move = last_move
                path += [new_pos_option]

                path_cost += self.__real_life_distance(new_pos_option, cur_pos) * cost_per_meter

                if abs(max_cost - path_cost) < cost_per_meter * max(self._pixel_dim[0], self._pixel_dim[1]):
                    return path

                cur_pos = new_pos_option
            else:
                while True:
                    move = random.randrange(0, 8)
                    sum_temp = [sum(x) for x in zip(directions[last_legal_move], directions[move])]
                    if sum_temp != [0, 0]:
                        break
                last_move = move

    def __h(self, location, goal, epsilon):
        """The heuristic function we used - aerial distance."""
        return self.__real_life_distance(goal, location) * epsilon

    def __weighted_a_star(self, start_location, end_location, cost_per_meter, weight):
        """
        This method calculates weighted A* algorithm between start_location and end_location.
        The weight (epsilon) for weighted A* is the parameter weight.
        """
        open_set = {tuple(start_location)}
        came_from = {}
        g_score = {tuple(start_location): 0}
        f_score = {tuple(start_location): self.__h(start_location, end_location, weight)}

        while bool(open_set):
            current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
            # TODO: write an equal method for two points and add a path cost sum and another stopping condition.
            if current == tuple(end_location):  # The stop condition.
                path = [list(current)]
                while current in came_from.keys():
                    current = came_from.get(current)
                    path.insert(0, list(current))
                return path

            open_set.remove(current)
            # TODO: delete diagonals
            directions = [[-1, 0], [1, 0], [0, -1], [0, 1], [1, 1], [1, -1], [-1, 1], [-1, -1]]
            for direction in directions:
                neighbor = [sum(x) for x in zip(direction, list(current))]  # TODO: get neighbors differently.
                if self._can_fly_over_in_bound(neighbor):
                    # TODO: make sure the change to tentative_g_score is correct.
                    move_cost = cost_per_meter * self.__real_life_distance(current, neighbor)
                    tentative_g_score = g_score.get(current, float('inf')) + move_cost
                    if tentative_g_score < g_score.get(tuple(neighbor), float('inf')):
                        came_from[tuple(neighbor)] = current
                        g_score[tuple(neighbor)] = tentative_g_score
                        f_score[tuple(neighbor)] = g_score.get(tuple(neighbor), float('inf')) + self.__h(neighbor,
                                                                                                         end_location,
                                                                                                         weight)
                        if tuple(neighbor) not in open_set:
                            open_set.add(tuple(neighbor))
        return []

    # TODO: fix the cost calculation.
    def __gen_path_weighted_a_star(self, start_location: list, max_cost: float, cost_per_meter: float,
                                   weight: float = 1.0):
        """
        This method creates a path from the given starting location to a randomized end
        location with the cost of max_cost.
        """
        path = [start_location]
        cur_start = start_location
        while True:
            next_goal = self._gen_random_point_under_constraints()
            path += self.__weighted_a_star(cur_start, next_goal, cost_per_meter, weight)[1:]
            cur_start = path[-1]
            if (len(path) - 1) * cost_per_meter > max_cost:  # TODO: trim the path differently
                return path[:int(max_cost / cost_per_meter) + 1]

    """
    Constraint checks
    """

    def _can_fly_over(self, pos_x, pos_y):
        """This method checks if a position in the dsm map got height value larger then the drone's flight height"""
        return self._dsm[pos_x][pos_y] <= self._flight_height

    def _in_map_bound(self, pos):
        """
        This method returns true if the position is in the maps main area (without the leading zero height areas
        on the sides of the map).
        """
        in_x_bound = self._x_upper_bound >= pos[0] >= self._x_lower_bound
        in_y_bound = self._y_upper_bound >= pos[1] >= self._y_lower_bound
        return in_x_bound and in_y_bound

    def _can_fly_over_in_bound(self, pos):
        return self._in_map_bound(pos) and self._can_fly_over(pos[0], pos[1])

    """
    Map processing methods. 
    """

    def __bound_map_main_area(self):
        """
        Assuming the given map is a square matrix. This method returns the map's boundaries so we  can look at the
        map without leading zero valued side rows or columns. For example:
                                           0000000000
                                           0001111100        011111
                                           0033262500   ->   332625
                                           0057625200        576252
                                           0000000000
        Will result x_lower_bound, y_lower_bound, x_upper_bound, y_upper_bound = 1, 2, 3, 8.
        """
        if self._dsm is not None:
            x_indices = [i for i in range(self._map_dim[0])]
            y_indices = [i for i in range(self._map_dim[1])]
            x_lower_bound = self.__get_first_nonzero_row(x_indices)
            y_lower_bound = self.__get_first_nonzero_col(y_indices)
            x_indices.reverse()
            y_indices.reverse()
            x_upper_bound = self.__get_first_nonzero_row(x_indices)
            y_upper_bound = self.__get_first_nonzero_col(y_indices)
            return x_lower_bound, y_lower_bound, x_upper_bound, y_upper_bound
        else:
            return 0, 0, 0, 0

    def __get_first_nonzero_row(self, indices):
        """
        This method returns the first cell value, x, of indices in which row number x of the dsm map
        holds at least one value that is not zero.
        """
        if indices is not None:
            for x in indices:
                for j in range(self._map_dim[1]):
                    if not self.__is_zero(self._dsm[x][j]):
                        return x
        return 0  # Shouldn't get here with correct use

    def __get_first_nonzero_col(self, indices):
        """
        This method returns the first cell value, y, of indices in which column number y of the dsm map
        holds at least one value that is not zero.
        """
        if indices is not None:
            for y in indices:
                for i in range(self._map_dim[0]):
                    if not self.__is_zero(self._dsm[i][y]):
                        return y
        return 0  # Shouldn't get here with correct use

    def __adjust_heights(self):
        for i in range(self._map_dim[0]):
            for j in range(self._map_dim[1]):
                self._dsm[i][j] = abs(self._dsm[i][j] + self._org_vector[2])

    def __process_map(self):
        self.__adjust_heights()

    """
    Helping methods. 
    """

    def __real_life_distance(self, p1, p2):
        """This method gets two points on the dsm grid and return the distance between them in real life."""
        x_squared_dist = math.pow(self._pixel_dim[0] * abs(p1[0] - p2[0]), 2)
        y_squared_dist = math.pow(self._pixel_dim[1] * abs(p1[1] - p2[1]), 2)
        return math.sqrt(x_squared_dist + y_squared_dist)

    def _gen_random_point_under_constraints(self):
        """
        This method return a random point in the map's main area (bounded in _bound_map_main_area)
        whose height value is shorter then the flight's height.
        """
        while True:
            rand_x = random.randrange(self._x_lower_bound, self._x_upper_bound)
            rand_y = random.randrange(self._y_lower_bound, self._y_upper_bound)
            if self._can_fly_over(rand_x, rand_y):
                return [rand_x, rand_y]

    def __adjust_stride(self):
        if self._stride_length > self.MAX_STRIDE_LEN:
            self._stride_length = self.MAX_STRIDE_LEN
        if self._stride_length < self.MIN_STRIDE_LEN:
            self._stride_length = self.MIN_STRIDE_LEN

    @staticmethod
    def __is_zero(val):
        return -2 <= int(val) <= 2
