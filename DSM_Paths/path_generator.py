import os
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import random
import math
from DSM_Paths.DsmParser import create_map


class PathGenerator:
    """
    This is the main class. To create paths, create an instance of a PathGenerator with the desired parameters.
    """
    def __init__(self, velocity, flight_height, dsm=None, pixel_dist=2.0):
        """
        Outputs the map with the given path on it.

         Parameters
        ----------
        velocity : float
            The drone's velocity in meters/second.
        flight_height : float
            The flight's altitude in meters.
        dsm : List of Lists of floats
            The DSM map.
        pixel_dist : float
            The distance of a pixel's width in real life (in meters).
        """
        self._velocity = velocity
        self._flight_height = flight_height
        self._pixel_dist = pixel_dist
        if dsm is not None:
            self._dsm = dsm
            self._map_side = len(dsm)
        else:
            self._dsm = None
            print('Need to initiate map using init_map before we start.')
        self._process_map()  # TEMP
        # These fields are used to determine the boundaries of the map (where the object appear).
        self._x_lower_bound, self._y_lower_bound, self._x_upper_bound, self._y_upper_bound = self._bound_map_main_area()

    def init_map(self, input_path=None, file_name=None, save_tif=False, pixel_dist=2):
        """
        This method loads the DSM map to an instance using a binary file representing it. The binary file must be in
        a directory named 'BinFiles' inside the project's main directory.
        If the save_tif argument is True the dsm map will be saved as a .tif image in a folder named DSMout under the
        name DSM0.tif.
        """
        if input_path is not None and file_name is not None:
            self._dsm = create_map(input_path, file_name, save_tif)
            self._map_side = len(self._dsm)
            self._pixel_dist = pixel_dist
        self._process_map()  # TEMP
        # Updating the map boundary values.
        self._x_lower_bound, self._y_lower_bound, self._x_upper_bound, self._y_upper_bound = self._bound_map_main_area()

    def gen_paths(self, flag, constrain, path_type, start_location=None, path_num=1, to_print=False, epsilon=1.0):
        """
                Enhancing the dsm resolution by the given multiplier
                 Parameters
                ---------
                flag : str
                    Will hold either 'd' for distance constrain or 't' for time constrain.
                constrain : float
                    The paths' length in meters for distance constrained paths or the travel time for time
                    in seconds constrained paths.
                start_location : list
                    The path's first point. If none is passed or the given point is not valid a random one will
                    be generated.
                path_num : int
                    The number of paths wanted.
                path_type
                    The kind of path generating algorithm used.
                    Will hold either 'prob' for probability random path and 'a_star' for a_star generated path to
                    random spots.
                to_print : bool
                    Pass True to print the resulting paths over the dsm map.
        """
        if self._dsm is not None:
            need_new_start_pos = start_location is None or not self._can_fly_over_in_bound(start_location)
            if flag != 'd' and flag != 't':
                print('Wrong flag option!')
                return []
            if constrain <= 0:
                print('Constrain should be positive')
                return []
            pixel_cost = self._pixel_dist
            if flag == 't':
                # self._pixel_dis = orig_pixel_dist / self._velocity  # Note: time*velocity = distance -> t = d/v
                pixel_cost /= self._velocity
            paths = []
            if path_type == 'prob':
                for i in range(path_num):
                    if need_new_start_pos:
                        start_location = self._gen_random_point_under_constraints()
                    paths += [self.__gen_path_probability(start_location=start_location, max_cost=constrain,
                                                          pixel_cost=pixel_cost)]
            elif path_type == 'a_star':
                for i in range(path_num):
                    if need_new_start_pos:
                        start_location = self._gen_random_point_under_constraints()
                    paths += [self.__gen_path_a_star_epsilon(start_location=start_location, max_cost=constrain,
                                                             pixel_cost=pixel_cost, epsilon=epsilon)]
            else:
                print('Path type is not one of the correct path generating ways')
                return []
            if to_print:
                for path in paths:
                    self.print_path(path)
            return paths
        else:
            print('Need to initiate map using init_map before we start.')
            return []

    def print_path(self, points=None, path_color='r', path_style='--'):
        """
        Outputs the map with the given path on it.

         Parameters
        ----------
        points : A 2D list.
            The points that makes the path. points[0] holds the x values of the path's points and points[1] holds
            the y values.
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
            plot_style = path_color + path_style
            plt.figure(1)
            plt.imshow(self._dsm)
            if points is not None and len(points) >= 2:
                x_list = []
                y_list = []
                for point in points:
                    x_list += [point[0]]
                    y_list += [point[1]]
                plt.plot(y_list, x_list, plot_style)
                # TODO: decide if we want start and end points showed.
                # plt.plot(y_list[0], x_list[0], 'w.')
                # plt.plot(y_list[-1], x_list[-1], 'b.')
            plt.show()
        else:
            print('Need to initiate map using init_map before we start.')

    def resize_dsm(self, multiplier, enhance=True):
        """
        Enhancing the dsm resolution by the given multiplier
         Parameters
        ---------
        multiplier : int
            The enhance multiplier. Must be > 0.
        enhance : bool
            True for enhancing and False for decrease resolution.

         Notes
        ---------
        Decreasing resolution might cause information loss as we use max value substitution.
        """
        if self._dsm is not None:
            if multiplier <= 0:
                return  # Not allowing a negative enhance multiplier.
            if enhance:
                self._map_side *= multiplier
                new_dsm = [[0 for j in range(self._map_side)] for i in range(self._map_side)]
                for x in range(0, len(new_dsm)):
                    for y in range(0, len(new_dsm)):
                        new_dsm[x][y] = self._dsm[int(x/multiplier)][int(y/multiplier)]
                self._pixel_dist = self._pixel_dist/multiplier
            else:
                prev_side = self._map_side
                self._map_side = int(prev_side/multiplier) if prev_side % multiplier == 0 else 1 + int(prev_side/multiplier)
                new_dsm = [[0 for j in range(self._map_side)] for i in range(self._map_side)]
                for x in range(0, len(new_dsm)):
                    for y in range(0, len(new_dsm)):
                        maxi = -np.inf
                        for i in range(0, multiplier):
                            for j in range(0, multiplier):
                                x_idx = x*multiplier + i
                                y_idx = y*multiplier + j
                                val = -np.inf if x_idx >= prev_side or y_idx >= prev_side else self._dsm[x_idx][y_idx]
                                maxi = max(maxi, val)
                        new_dsm[x][y] = maxi
                self._pixel_dist = prev_side*self._pixel_dist/self._map_side
            self._dsm = new_dsm
            self._map_side = len(self._dsm)
        else:
            print('Need to initiate the dsm map using init_map before we resize it.')

    def print_map_size(self):
        if self._dsm is not None:
            print(f'self._map_side: {self._map_side}')
            print(f'len(self._dsm): {len(self._dsm)}')
            print(f'len(self._dsm[0]): {len(self._dsm[0])}')
            print(f'self._pixel_dist: {self._pixel_dist}')
        else:
            print('Need to initiate the dsm map using init_map before we show it\'s size.')

    def _bound_map_main_area(self):
        """ Assuming the given map is a square matrix. This method returns the map's boundaries so we  can look at the
        map without leading zero valued side rows or columns. For example:
                                           0000000000
                                           0001111100        011111
                                           0033262500   ->   332625
                                           0057625200        576252
                                           0000000000
        Will result x_lower_bound, y_lower_bound, x_upper_bound, y_upper_bound = 1, 2, 3, 8.
        """
        if self._dsm is not None:
            indices = [i for i in range(self._map_side)]
            x_lower_bound = self._get_first_nonzero_row(indices)
            y_lower_bound = self._get_first_nonzero_col(indices)
            indices.reverse()
            x_upper_bound = self._get_first_nonzero_row(indices)
            y_upper_bound = self._get_first_nonzero_col(indices)
            return x_lower_bound, y_lower_bound, x_upper_bound, y_upper_bound
        else:
            return 0, 0, 0, 0

    def _get_first_nonzero_row(self, indices):
        if indices is not None:
            for x in indices:
                for j in range(self._map_side):
                    if not self.__is_zero(self._dsm[x][j]):
                        return x
        return 0  # Shouldn't get here with correct use

    def _get_first_nonzero_col(self, indices):
        if indices is not None:
            for y in indices:
                for i in range(self._map_side):
                    if not self.__is_zero(self._dsm[i][y]):
                        return y
        return 0  # Shouldn't get here with correct use

    def __inside_array(self, pos):
        return self._dsm.shape[0] > pos[0] >= 0 and self._dsm.shape[1] > pos[1] >= 0

    def __gen_path_probability(self, start_location: list, max_cost: float, pixel_cost: float):
        cur_pos = start_location
        path = [cur_pos]
        directions = [[-1, 0], [0, -1], [1, 0], [0, 1], [1, 1], [1, -1], [-1, 1], [-1, -1]]
        last_move = 0
        path_cost = 0
        while True:
            new_pos_option = [sum(x) for x in zip(directions[last_move], cur_pos)]
            if self._can_fly_over_in_bound(new_pos_option) and random.randrange(0, 100) <= 96:
                path += [new_pos_option]

                x_dist_squared = math.pow(new_pos_option[0] - cur_pos[0], 2)
                y_dist_squared = math.pow(new_pos_option[1] - cur_pos[1], 2)
                path_cost += math.sqrt(x_dist_squared + y_dist_squared) * pixel_cost

                if abs(max_cost - path_cost) < pixel_cost:
                    path.pop()
                    return path

                cur_pos = new_pos_option
            else:
                while True:
                    move = random.randrange(0, 8)
                    sum_temp = [sum(x) for x in zip(directions[last_move], directions[move])]
                    if sum_temp != [0, 0]:
                        break
                last_move = move

    # Alon's Note: Should be a static method in my opinion. Look at the to-do comment.
    def __h(self, location, goal, epsilon):  # TODO: Why do we need to multiply by self._pixel_dist * epsilon?
        """The heuristic function we used - Manhattan distance."""
        return (abs(goal[0] - location[0]) + abs(goal[1] - location[1])) * self._pixel_dist * epsilon

    def a_star_epsilon(self, start_location, end_location, epsilon):
        """ Alon's note: this is regular A-star, epsilon has no meaning here. We need to choose a random next node
        from a set containing node's with f <= (1+epsilon)min_f and not the node with the smallest f value.
        We can leave it as is (A-Star algorithm) and just change the name and delete epsilon. The results are good
        with the regular algorithm you implemented.
        """
        open_set = {tuple(start_location)}
        came_from = {}
        g_score = {tuple(start_location): 0}
        f_score = {tuple(start_location): self.__h(start_location, end_location, epsilon)}

        while bool(open_set):
            # Alon's note: cool syntax can you explain what does lambda do?
            current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
            if current == tuple(end_location):
                path = [list(current)]
                while current in came_from.keys():
                    current = came_from.get(current)
                    path.insert(0, list(current))
                return path

            open_set.remove(current)
            directions = [[-1, 0], [1, 0], [0, -1], [0, 1]]
            for direction in directions:  # Alon's note: dir is a built-in function so I changed dir -> direction
                neighbor = [sum(x) for x in zip(direction, list(current))]
                # Alon's note: changed __inside_array -> _can_fly_over_in_bound
                if self._can_fly_over_in_bound(neighbor):
                    tentative_g_score = g_score.get(current, float('inf')) + self._pixel_dist
                    if tentative_g_score < g_score.get(tuple(neighbor), float('inf')):
                        came_from[tuple(neighbor)] = current
                        g_score[tuple(neighbor)] = tentative_g_score
                        # Alon's note: Should'nt it be: f_score = *tentative_g_score* + h??
                        f_score[tuple(neighbor)] = g_score.get(tuple(neighbor), float('inf')) + self.__h(neighbor,
                                                                                                         end_location,
                                                                                                         epsilon)
                        if tuple(neighbor) not in open_set:
                            open_set.add(tuple(neighbor))
        return []

    def __gen_path_a_star_epsilon(self, start_location: list, max_cost: float, pixel_cost: float, epsilon: float = 1.0):
        """
        This method creates a path from the given starting location to a randomized end
        location with the length of max_len.
        """
        path = []
        cur_start = start_location
        while True:
            next_goal = self._gen_random_point_under_constraints()  # Note: replaced the loop with this method.
            path += self.a_star_epsilon(cur_start, next_goal, epsilon)
            cur_start = path[-1]
            if len(path) * pixel_cost > max_cost:
                return path[:int(max_cost / pixel_cost)]

    def _gen_random_point_under_constraints(self):
        """This method return a random point in the map's main area (bounded in _bound_map_main_area)
        whose height value is shorter then the flight's height."""
        while True:
            rand_x = random.randrange(self._x_lower_bound, self._x_upper_bound)
            rand_y = random.randrange(self._y_lower_bound, self._y_upper_bound)
            if self._can_fly_over(rand_x, rand_y):
                return [rand_x, rand_y]

    def _can_fly_over(self, pos_x, pos_y):
        """This method checks if a position in the dsm map got height value larger then the drone's flight height"""
        # TODO: change later to <= when we understand the negative values in the dsm.
        # TEMP - changed need permanent fix for height values!
        return self._dsm[pos_x][pos_y] <= self._flight_height

    def _in_map_bound(self, pos):
        """This method returns true if the position is in the maps main area (without the leading zero height areas
        on the sides of the map)."""
        in_x_bound = self._x_upper_bound >= pos[0] >= self._x_lower_bound
        in_y_bound = self._y_upper_bound >= pos[1] >= self._y_lower_bound
        return in_x_bound and in_y_bound

    def _can_fly_over_in_bound(self, pos):
        return self._in_map_bound(pos) and self._can_fly_over(pos[0], pos[1])

    def calc_path_distance(self, path: list):
        if path is not None:
            distance = 0
            for i in range(len(path) - 1):
                x_squared = math.pow(path[i+1][0] - path[i][0], 2)
                y_squared = math.pow(path[i+1][1] - path[i][1], 2)
                distance += math.sqrt(x_squared + y_squared)*self._pixel_dist
            return distance
        return 0

    def calc_path_travel_time(self, path: list):
        if path is not None:
            travel_time = 0
            for i in range(len(path) - 1):
                x_squared = math.pow(path[i+1][0] - path[i][0], 2)
                y_squared = math.pow(path[i+1][1] - path[i][1], 2)
                distance = math.sqrt(x_squared + y_squared)*self._pixel_dist
                travel_time += distance / self._velocity
            return travel_time
        return 0

    # TODO: get rid of this method TEMP.
    def _process_map(self):
        zero_val = self._dsm[0][0]
        for i in range(len(self._dsm)):
            for j in range(len(self._dsm)):
                self._dsm[i][j] = abs(self._dsm[i][j] - zero_val)

    @staticmethod
    def __is_zero(val):
        # TEMP
        # return -2 <= int(val) + 243 <= 2
        return -2 <= int(val) <= 2




