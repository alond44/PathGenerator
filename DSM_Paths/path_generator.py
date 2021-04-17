import math
import os  # new
import random
import sys
import csv  # new
from enum import Enum, unique, auto
from pathlib import Path  # new

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.pyplot import figure
from rtree import index  # new

from DSM_Paths.DsmParser import DSMParcer
from DSM_Paths.geometry import ConvexPolygon, Point

# a flag for printing the obstacles polygons when printing the map.
# if set to True, a print_path() method call will also print the obstacle polygons.
DEBUG_OBSTACLE = False


@unique
class PathType(Enum):
    MAP_ROAM = auto()
    AREA_EXPLORE = auto()
    A_STAR = auto()


@unique
class ConstraintType(Enum):
    TIME = auto()
    DISTANCE = auto()


class PathGenerator:
    """
    This is the main class. To create paths, create an instance of a PathGenerator with the desired parameters then,
    call gen_paths with the constraints of your choosing.
    """
    SAMPLE_RATE = 0.6           # The drone way point minimal distance coefficient is SAMPLE_RATE * velocity.
    MAX_STRIDE_LEN = 6.0        # Allowing the largest stride length to be MAX_STRIDE_LEN meters.
    MIN_STRIDE_LEN = 2.0        # Allowing the smallest stride length to be MIN_STRIDE_LEN meters.
    MAX_ANGLE = 60.0            # Allowing the largest maximum turn angle value to be MAX_ANGLE meters
    MIN_ANGLE = 20.0            # Allowing the smallest maximum turn angle value to be MIN_ANGLE meters
    DEGREE_DIVISOR = 2          # Allowing DEGREE_DIVISOR possible angles of right turn and DEGREE_DIVISOR of left turn.
    RADIUS = 1.2                # We don't allow the drone to be within RADIUS meter of an obstacle.
    RANDOM_TURN_PROB = 0.10     # The probability of a random turn while creating paths of type AREA_EXPLORE.
    RANDOM_TURN_ANGLE = 120.0   # The desired turn angle during a random turn (in degrees).
    EPSILON = 0.001
    # Important: read the doc if you decide to change these constants.
    # These are used to better our obstacle creation process.
    MIN_RECURSION_DEPT = 50  # We don't allow the recursive method for obstacle tracing to have
    # a maximal dept lower then 50.
    RECURSION_TO_MAP_SIZE_RATIO = 75.0 / 770.0  # Used to set a depth maximum for the obstacle finding function.

    def __init__(self, velocity, flight_height, dsm=None, origin=(0.0, 0.0, 0.0), map_dimensions=(0, 0),
                 pixel_dimensions=(0.0, 0.0), stride_multiplier=1.0, max_angle=45.0):
        """
        Creates an instance of a PathGenerator.

         Parameters
        ----------
        velocity : float
            The drone's velocity in meters/second.
        flight_height : float
            The flight's altitude in the world's coordinate system (measured in meters).
        dsm : List of Lists of floats
            The DSM map. We assume it is squared.
        origin: tuple of floats
            A distance vector between the world map's origin and the dsm map's origin. len(origin) must be 3.
        map_dimensions: tuple of floats
            A two value tuple: (<row_number>, <column_number>).
        pixel_dimensions: tuple of floats
            A two value tuple: (<dWx>, <dWy>) where dWx is the distance (in meters) one stride in the x axis direction
            ((x, y) -> (x + 1, y)) represents and dWy is the distance (in meters) one stride in the y axis
            direction represents.
        stride_multiplier: float
            Adjusts the strides' length. default stride length is 0.6 * velocity must be > 0.
        max_angle: float
            The maximal drone turn degree we are allowing.
        """
        self._velocity = velocity
        self._flight_height = flight_height
        self.__max_recursion_limit = sys.getrecursionlimit() - 10
        self._max_angle = max_angle
        if stride_multiplier > 0:
            self._stride_length = self.SAMPLE_RATE * velocity * stride_multiplier
        else:
            self._stride_length = self.SAMPLE_RATE * velocity
        self._adjust_stride()
        self._adjust_angle()
        if dsm is not None and origin is not None and map_dimensions is not None and pixel_dimensions is not None \
                and len(origin) == 3 and len(map_dimensions) == 2 and len(pixel_dimensions) == 2:
            self._dsm = dsm
            self._org_vector = list(origin)
            self._flight_height -= self._org_vector[2][0]
            self._max_recursion_limit = int(self.RECURSION_TO_MAP_SIZE_RATIO *
                                            max(map_dimensions[0], map_dimensions[1]))
            self._map_dim = list(map_dimensions)
            self._pixel_dim = list(pixel_dimensions)
            self.__adjust_heights()
            self.__adjust_recursion_depth()
            # Updating the map boundary values.
            self._x_lower_bound, self._y_lower_bound, self._x_upper_bound, self._y_upper_bound = \
                self.__bound_map_main_area()
            # creating the obstacles.
            self._obstacle_list = self.__get_obstacle_polygon_list()
            self._obstacle_index = index.Index()  # creating the obstacle's rtree.
            self.__initiate_obstacle_index()
        else:
            self._dsm = None
            print('Need to initiate map using init_map before we start.')

    def init_map(self, input_path=None, file_name=None, save_tif=False):
        """
        This method loads the DSM map to an instance using a binary file representing it. The binary file must be in
        a directory named 'BinFiles' inside the 'input_path' directory.
        If the save_tif argument is True the dsm map will be saved as a .tif image in a folder named DSMout under the
        name DSM0.tif.

        Note:   - This method can be used for initiating the DSM map only. If you want to create paths on a
                  different map create a new PathGenerator instance.
        """
        if input_path is not None and file_name is not None:
            _, _, _, x_org, y_org, z_org, wx, wy, dwx, dwy, self._dsm = DSMParcer(input_path, file_name, save_tif)
            self._flight_height -= z_org
            self._org_vector = [x_org, y_org, z_org]
            self._map_dim = [wx, wy]
            self._pixel_dim = [dwx, dwy]
            self._max_recursion_limit = int(self.RECURSION_TO_MAP_SIZE_RATIO * max(wx, wy))
            self.__adjust_heights()
            self.__adjust_recursion_depth()
            # Updating the map boundary values.
            self._x_lower_bound, self._y_lower_bound, self._x_upper_bound, self._y_upper_bound = \
                self.__bound_map_main_area()
            # creating the obstacles.
            self._obstacle_list = self.__get_obstacle_polygon_list()
            self._obstacle_index = index.Index()  # creating the obstacle's rtree.
            self.__initiate_obstacle_index()

    def gen_paths(self, flag: ConstraintType, constraint: float, path_type: PathType, start_location=None, path_num=1,
                  to_print=False, weight=1.0, result_folder_path=None):
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
        start_location : Point
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
        result_folder_path : str
            The path to the folder we want our output csv files to be written to.
        """
        if self._dsm is not None:
            # error checks.
            if flag != ConstraintType.DISTANCE and flag != ConstraintType.TIME:
                print('Wrong flag option!')
                return []
            if constraint <= 0:
                print('Constrain should be positive')
                return []
            if path_type != PathType.MAP_ROAM and path_type != PathType.AREA_EXPLORE and path_type != PathType.A_STAR:
                print('Path type is not one of the correct path generating ways.')
                print('This method except PathType.AREA_EXPLORE and PathType.MAP_ROAM.')
                return []
            # creating the base folder for the paths' csv files.
            if result_folder_path is None:
                result_folder_path = f'{Path(__file__).parent.absolute()}/Paths'
            if not (os.path.isdir(result_folder_path)):
                os.mkdir(result_folder_path)
            cost_per_meter = 1
            if flag == ConstraintType.TIME:
                cost_per_meter /= self._velocity  # (passing a meter takes (1 / velocity) seconds)
            paths = []
            need_new_start_pos = start_location is None or not self._in_legal_flight_zone(start_location)
            for i in range(path_num):  # The path generating loop.
                if need_new_start_pos:
                    start_location = self._gen_random_point_under_constraints()
                if path_type == PathType.MAP_ROAM:
                    path = self.__gen_path_probability(start_location=start_location, max_cost=constraint,
                                                       cost_per_meter=cost_per_meter)
                elif path_type == PathType.AREA_EXPLORE:
                    path = self.__gen_path_probability(start_location=start_location, max_cost=constraint,
                                                       cost_per_meter=cost_per_meter, random_turn=True)
                else:  # meaning path_type is PathType.A_STAR
                    list_start = [start_location.x, start_location.y]  # converting Point to list
                    list_path = self.__gen_path_weighted_a_star(start_location=list_start, max_cost=constraint,
                                                                cost_per_meter=cost_per_meter, weight=weight)
                    path = []
                    for item in list_path:  # converting back to Point
                        path.append(Point(item[0], item[1]))
                self.__create_path_csv_file(path=path, directory=result_folder_path, path_number=i + 1)
                paths += [path]
                print(f"created path number {i + 1} out of {path_num}")
            if to_print:
                for path in paths:
                    self.print_path(path)
            return paths
        else:
            print('Need to initiate map using init_map before we start.')
            return []

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
                path_list = []
                for point in path:
                    path_list.append([point.x, point.y])
                points = np.array(path_list)
                plt.plot(points[:, 1], points[:, 0], plot_style)
                # printing the path's start and end points with their descriptions on the map
                start_patch = patches.Patch(color='cyan', label='Path\'s start')
                end_patch = patches.Patch(color='blue', label='Path\'s end')
                plt.legend(handles=[start_patch, end_patch], bbox_to_anchor=(1.32, 1), loc='upper right')
                plt.plot(points[0][1], points[0][0], 'c.')
                plt.plot(points[-1][1], points[-1][0], 'b.')

            # Print obstacle for testing purposes.
            if DEBUG_OBSTACLE:
                for obstacle in self._obstacle_list:
                    obstacle_point_list = []
                    for point in obstacle.sorted_points:
                        obstacle_point_list.append([point.x, point.y])
                    points = np.array(obstacle_point_list)
                    x_values, y_values = [], []
                    for y in points[:, 1]:
                        y_values.append(y)
                    y_values.append(points[:, 1][0])
                    for x in points[:, 0]:
                        x_values.append(x)
                    x_values.append(points[:, 0][0])
                    plt.plot(y_values, x_values, 'k-')

            plt.show()
        else:
            print('Need to initiate map using init_map before we start.')

    def calc_path_distance(self, path: list):
        """
            This method gets a path and returns the path's distance on the instance's map.
        """
        if path is not None:
            distance = 0
            for i in range(len(path) - 1):
                distance += self.__euclidean_distance(path[i], path[i + 1])
            return distance
        return 0

    def calc_path_travel_time(self, path: list):
        """
            This method gets a path and returns the path's travel duration on the instance's map.
        """
        if path is not None:
            travel_time = 0
            for i in range(len(path) - 1):
                travel_time += self.__euclidean_distance(path[i], path[i + 1]) / self._velocity
            return travel_time
        return 0

    def __gen_path_probability(self, start_location: list, max_cost: float, cost_per_meter: float, random_turn=False):
        """
            This method calculate a path with cost = cost max_cost where a move's cost is calculated as the distance
            between the two neighbors multiplied by cost_per_meter.
        """
        random_turn_max_cost = \
            (self.RANDOM_TURN_ANGLE * self.DEGREE_DIVISOR / self._max_angle) * self._stride_length * cost_per_meter
        cur_pos = start_location
        strides = self._get_possible_strides(cur_pos)  # randomizing the first possible strides
        step = strides[random.randrange(len(strides))]  # randomizing the first move
        strides_copy = strides.copy()
        strides_copy.remove(step)
        path_points = [(cur_pos, strides_copy)]
        path_cost = 0
        made_turn = False
        while True:
            new_pos_option = Point(cur_pos.x + step.x, cur_pos.y + step.y)
            if self.__is_stride_legal(new_pos_option, cur_pos):
                # the other options from this point are strides without the chosen stride.
                strides_copy = strides.copy()
                self._remove_closest_step(strides_copy, step)
                path_points += [(new_pos_option, strides_copy)]
                path_cost += self.__euclidean_distance(new_pos_option, cur_pos) * cost_per_meter

                if random_turn and random.randrange(0, 100) < 100 * self.RANDOM_TURN_PROB and\
                        max_cost - path_cost > random_turn_max_cost:
                    if len(path_points) > 2:
                        addition_cost, path_addition, cur_addition, prev_addition \
                            = self._add_turn(Point(new_pos_option.x, new_pos_option.y), Point(cur_pos.x, cur_pos.y),
                                             self.RANDOM_TURN_ANGLE, cost_per_meter)
                        if len(path_addition) != 0:
                            path_points += path_addition
                            path_cost += addition_cost
                            new_pos_option = cur_addition
                            cur_pos = prev_addition
                            made_turn = True

                if abs(max_cost - path_cost) < cost_per_meter * self._stride_length:
                    path = []
                    for point in path_points:
                        path += [point[0]]
                    return path
                strides = self._get_possible_strides(cur_pos=new_pos_option, prev_pos=cur_pos)
                if made_turn:
                    made_turn = False
                    step = strides[random.randrange(len(strides))]
                cur_pos = new_pos_option
            else:
                self._remove_closest_step(strides, step)
                if len(strides) != 0:
                    step = strides[random.randrange(len(strides))]
                else:  # the current pos got no legal strides left
                    while len(path_points) > 1:
                        prev_pos, _ = path_points[-2]
                        _, strides = path_points[-1]
                        del path_points[-1]
                        # Note: the stride's length supposed to be constant but numerical error cause it not to be.
                        # (errors that derive from the way we calculate the possible steps).
                        path_cost -= self.__euclidean_distance(prev_pos, cur_pos) * cost_per_meter
                        cur_pos = prev_pos
                        if len(strides) != 0:
                            break
                    if len(path_points) <= 1:  # path was emptied out and there are no legal moves
                        cur_pos = self._gen_random_point_under_constraints()
                        strides = self._get_possible_strides(cur_pos)  # randomizing the first possible strides again.
                        path_points = [(cur_pos, strides)]
                        path_cost = 0
                    step = strides[random.randrange(len(strides))]

    def __h(self, location: list, goal: list, weight: float):
        """The heuristic function we used - euclidean distance."""
        return self.__euclidean_distance(Point(goal[0], goal[1]), Point(location[0], location[1])) * weight

    def __weighted_a_star(self, start_location: list, end_location: list, cost_per_meter: float, weight: float,
                          previous_point=None):
        """
        This method calculates weighted A* algorithm between start_location and end_location.
        The weight (epsilon) for weighted A* is the parameter weight.
        """
        if previous_point is None:
            previous_point = [None]
        start_state = start_location + previous_point
        open_set = {tuple(start_state)}
        came_from = {}
        g_score = {tuple(start_state): 0}
        f_score = {tuple(start_state): self.__h(start_location, end_location, weight)}
        while bool(open_set):
            current = list(min(open_set, key=lambda x: f_score.get(x, float('inf'))))
            # The stop condition.
            if self.__euclidean_distance(Point(current[:2][0], current[:2][1]),
                                         Point(end_location[0], end_location[1])) <= 2 * self._stride_length:
                path = [current[:2]]
                while tuple(current) in came_from.keys():
                    current = came_from.get(tuple(current))
                    path.insert(0, current[:2])
                return path

            open_set.remove(tuple(current))
            if current[2] is None:  # randomizing the first possible strides
                strides = self._get_possible_strides(Point(current[:2][0], current[:2][1]), None)
            else:  # randomizing the first possible strides
                strides = self._get_possible_strides(Point(current[:2][0], current[:2][1]),
                                                     Point(current[2:][0], current[2:][1]))

            num_legal_strides = len(strides)
            for stride in strides:
                neighbor = [sum(x) for x in zip([stride.x, stride.y], current[:2])]
                if self.__is_stride_legal(Point(neighbor[0], neighbor[1]), Point(current[:2][0], current[:2][1])):
                    self.__remove_already_inside(neighbor, open_set)
                    move_cost = cost_per_meter * self.__euclidean_distance(Point(current[0], current[1]),
                                                                           Point(neighbor[0], neighbor[1]))
                    tentative_g_score = g_score.get(tuple(current), float('inf')) + move_cost
                    neighbor_state = tuple(neighbor + current[:2])
                    if tentative_g_score < g_score.get(neighbor_state, float('inf')):
                        came_from[neighbor_state] = current
                        g_score[neighbor_state] = tentative_g_score
                        f_score[neighbor_state] = tentative_g_score + self.__h(neighbor, end_location, weight)
                        if neighbor_state not in open_set:
                            open_set.add(neighbor_state)
                else:
                    num_legal_strides -= 1
            if num_legal_strides == 0 and current[:2] == start_location:
                return []
        return []

    def __gen_path_weighted_a_star(self, start_location: list, max_cost: float, cost_per_meter: float,
                                   weight: float = 1.0):
        """
        This method creates a path from the given starting location to a randomized end
        location with the cost of max_cost.
        """
        path = [start_location]
        cur_start = start_location
        sample_strides = self._get_possible_strides(start_location)
        sample_strides_world = [self.__convert_map_point_to_world_point(i) for i in sample_strides]
        min_distance = float("inf")
        for i in range(len(sample_strides_world)):
            for j in range(i+1, len(sample_strides_world)):
                dis = self.__euclidean_distance(sample_strides_world[i],sample_strides_world[j])
                if dis < min_distance:
                    self._min_stride_distance = 0.5 * dis
        while True:
            next_goal = self._gen_random_point_under_constraints()
            previous_step = None
            if len(path) > 1:
                previous_step = path[-2]
            new_path = self.__weighted_a_star(cur_start, [next_goal.x, next_goal.y], cost_per_meter, weight,
                                              previous_step)
            if not new_path and len(path) > 1:
                path = path[:-1]
            else:
                path += new_path[1:]
            cur_start = path[-1]
            if (len(path) - 1) * self._stride_length * cost_per_meter > max_cost:
                return path[:int((max_cost / cost_per_meter) / self._stride_length) + 1]

    def __create_path_csv_file(self, path: list, directory: str, path_number: int):
        """
            write the given path to a file named: 'path_<path_number>.csv' in the given directory.
        """
        if not (os.path.isdir(directory)):
            os.mkdir(directory)
        file_name = f'{directory}/path_{path_number}.csv'
        with open(file_name, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["x_m_w", "y_m_w", "z_m_w", "vx_m_s", "vy_m_s", "vz_m_s"])
            next_world_pos = self.__convert_map_point_to_world_point(path[0])
            z = self._flight_height + self._org_vector[2][0]
            for i in range(len(path) - 1):
                cur_world_pos = next_world_pos
                next_world_pos = self.__convert_map_point_to_world_point(path[i + 1])
                theta = math.atan2(next_world_pos.y - cur_world_pos.y, next_world_pos.x - cur_world_pos.x)
                x, y = cur_world_pos.x + self._org_vector[0][0], cur_world_pos.y + self._org_vector[1][0]
                vx, vy = self._velocity * math.cos(theta), self._velocity * math.sin(theta)
                writer.writerow([x, y, z, vx, vy, 0.])
            final_pos = self.__convert_map_point_to_world_point(path[-1])
            writer.writerow([final_pos.x, final_pos.y, z, 0., 0., 0.])

    def _add_turn(self, cur_pos: Point, prev_pos: Point, turn_angle: float, cost_per_meter: float):
        """
        This method responsible for random turns when generating paths of type AREA_EXPLORE.
        :return: The turn's way points and strides list, the turn's cost, and the new current and previous way points.
        """
        total_turn = 0.0
        addition_cost = 0.0
        path_addition = []
        random_sign = -1 if random.randrange(1) == 0 else 1
        while total_turn < turn_angle:
            strides = self._get_possible_strides(cur_pos, prev_pos)
            while True:
                if len(strides) == 0:
                    return addition_cost, path_addition, cur_pos, prev_pos
                step = strides[random.randrange(len(strides))]
                next_pos = Point(cur_pos.x + step.x, cur_pos.y + step.y)
                next_angle = self._get_orientation_degree(next_pos, cur_pos)
                cur_angle = self._get_orientation_degree(cur_pos, prev_pos)
                step_angle = next_angle - cur_angle
                if abs(step_angle) < self.EPSILON:
                    step_sign = 0
                else:
                    step_sign = -1 if step_angle < 0 else 1
                if step_sign == random_sign:
                    break
                else:
                    self._remove_closest_step(strides, step)
            if self.__is_stride_legal(next_pos, cur_pos):
                strides_copy = strides.copy()
                self._remove_closest_step(strides_copy, step)
                path_addition += [(next_pos, strides_copy)]
                addition_cost += self.__euclidean_distance(next_pos, cur_pos) * cost_per_meter
                total_turn += abs(step_angle)
                prev_pos = cur_pos
                cur_pos = next_pos
            else:
                break
        return addition_cost, path_addition, cur_pos, prev_pos

    """
    Constraint checks
    """

    def _can_fly_over(self, point: Point):
        """This method checks if a position in the dsm map got height value larger then the drone's flight height"""
        return self._dsm[int(point.x)][int(point.y)] > self._flight_height

    def _in_map_bound(self, point: Point):
        """
            This method returns true if the position is in the maps main area (without the leading zero height areas
            on the sides of the map).
        """
        in_x_bound = self._x_upper_bound >= int(point.x) >= self._x_lower_bound
        in_y_bound = self._y_upper_bound >= int(point.y) >= self._y_lower_bound
        return in_x_bound and in_y_bound

    def _can_fly_over_in_bound(self, point: Point):
        return self._in_map_bound(point) and self._can_fly_over(point)

    def _is_obstacle(self, point: Point):
        return self._in_map_bound(point) and not self._can_fly_over(point)

    def _too_close_to_obstacle(self, polygon: ConvexPolygon, next_point: Point, cur_point: Point):
        next_world_point = self.__convert_map_point_to_world_point(next_point)
        cur_world_point = self.__convert_map_point_to_world_point(cur_point)
        # Note: added these conversions and if statements as the loop does not cover the last polygon point.
        poly_point = self.__convert_map_point_to_world_point(polygon.sorted_points[-1])
        if self._point_within_radius_from_line(poly_point, next_world_point, cur_world_point):
            return True
        d = self.__euclidean_distance(polygon.sorted_points[-1], next_point)
        if d <= self.RADIUS:
            return True
        for i in range(-1, len(polygon.sorted_points) - 1):
            d = self.__euclidean_distance(polygon.sorted_points[i + 1], next_point)
            if d <= self.RADIUS:
                return True
            poly1 = self.__convert_map_point_to_world_point(polygon.sorted_points[i])
            poly2 = self.__convert_map_point_to_world_point(polygon.sorted_points[i + 1])
            if abs(poly1.x - poly2.x) < self.EPSILON:
                if abs(next_world_point.x - poly1.x) <= self.RADIUS and \
                        min(poly1.y, poly2.y) <= next_world_point.y <= max(poly1.y, poly2.y):
                    return True
            elif abs(poly1.y - poly2.y) < self.EPSILON:
                if abs(next_world_point.y - poly1.y) <= self.RADIUS and \
                        max(poly1.x, poly2.x) <= next_world_point.x <= max(poly1.x, poly2.x):
                    return True
            elif self._point_within_radius_from_line(next_world_point, poly1, poly2):
                return True
            if self._point_within_radius_from_line(poly2, next_world_point, cur_world_point):
                return True
        return False

    def _point_within_radius_from_line(self, point: Point, line_p1: Point, line_p2: Point):
        # distance from point to line calculation. The line between line_p1 and line_p2 is y = mx + b.
        m = (line_p1.y - line_p2.y) / (line_p1.x - line_p2.x)
        b = line_p1.y - m * line_p1.x
        distance = abs(m * point.x - point.y + b) / math.sqrt((m * m) + 1)
        return distance < self.RADIUS and min(line_p1.x, line_p2.x) <= point.x <= max(line_p1.x, line_p2.x)

    def __is_stride_legal(self, next_point: Point, cur_point: Point):
        if not (self._can_fly_over_in_bound(next_point) and self._can_fly_over_in_bound(cur_point)):
            return False
        min_x, max_x = min(next_point.x, cur_point.x) - self.RADIUS, max(next_point.x, cur_point.x) + self.RADIUS
        min_y, max_y = min(next_point.y, cur_point.y) - self.RADIUS, max(next_point.y, cur_point.y) + self.RADIUS
        obstacle_indices = self._obstacle_index.intersection((min_x, min_y, max_x, max_y))
        for idx in obstacle_indices:
            if self._obstacle_list[idx].line_intersect(next_point, cur_point) or \
                    self._too_close_to_obstacle(self._obstacle_list[idx], next_point, cur_point):
                return False
        return True

    def _in_legal_flight_zone(self, point: Point):
        """"
        Returns true if the point is within RADIUS of any of the *obstacle polygons*
        Note:   - this method is using the obstacle's R-Tree to check if the zone is clear and the other obstacle
                  detecting methods don't (because they are used to initiate the data structure).
        """
        return self._obstacle_index.count((point.x - self.RADIUS, point.y - self.RADIUS,
                                           point.x + self.RADIUS, point.y + self.RADIUS)) == 0

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
                self._dsm[i][j] = self._dsm[i][j] + self._org_vector[2]

    def __get_obstacle_from_point(self, binary_map, pos, depth):
        x = pos[0]
        y = pos[1]
        # Important note: is_obstacle checks if a point is in map bound.
        if self._is_obstacle(Point(x, y)) and binary_map[x][y] == 0:
            binary_map[x][y] = 1
            # Note: was- {(x, y), (x + 1, y), (x, y + 1), (x + 1, y + 1)} to fit the
            # obstacles presented on the map when printing. However the dsm numpy array values are these:
            point_set = {(x - 0.5, y - 0.5), (x + 0.5, y - 0.5), (x - 0.5, y + 0.5), (x + 0.5, y + 0.5)}
            if depth < self._max_recursion_limit:
                point_set = point_set.union(self.__get_obstacle_from_point(binary_map, (x + 1, y), depth + 1))
                point_set = point_set.union(self.__get_obstacle_from_point(binary_map, (x - 1, y), depth + 1))
                point_set = point_set.union(self.__get_obstacle_from_point(binary_map, (x, y + 1), depth + 1))
                point_set = point_set.union(self.__get_obstacle_from_point(binary_map, (x, y - 1), depth + 1))
            return point_set
        return {}

    def __get_obstacle_polygon_list(self):
        binary_obstacle_map = [[0 for _ in range(self._map_dim[1])] for _ in range(self._map_dim[0])]
        obstacle_list = []
        for x in range(self._map_dim[0]):
            for y in range(self._map_dim[1]):
                if self._is_obstacle(Point(x, y)) and binary_obstacle_map[x][y] == 0:
                    obstacle_polygon = ConvexPolygon(self.__get_obstacle_from_point(binary_obstacle_map, (x, y), 1))
                    obstacle_list.append(obstacle_polygon)
        return obstacle_list

    def __initiate_obstacle_index(self):
        """
            This method passes over the obstacle list and insert each obstacle (represented by its index in the
            obstacle list) to the rtree.
        """
        for i in range(len(self._obstacle_list)):
            obstacle = self._obstacle_list[i]
            self._obstacle_index.insert(i, (obstacle.min_x, obstacle.min_y, obstacle.max_x, obstacle.max_y))

    def __process_map(self):
        self.__adjust_heights()
        return self.__get_obstacle_polygon_list()

    """
    Helping methods. 
    """

    def __convert_map_point_to_world_point(self, point: Point):
        """
            This method gets a point in the dsm map coordinates and return the point in the world's dimensions.
            For example if the point is (1, 2) ad each pixel's dimensions in the world are dWx = 2 and dWy = 3
            the returned point will be (2, 6).
        """
        return Point(self._pixel_dim[0] * point.x, self._pixel_dim[1] * point.y)

    def __convert_world_point_to_map_point(self, point: Point):
        """
            This method reverse the effects of __convert_map_point_to_world_point.
        """
        return Point(point.x / self._pixel_dim[0], point.y / self._pixel_dim[1])

    def __euclidean_distance(self, p1: Point, p2: Point):
        """
            This method gets two points on the dsm grid and return the real world euclidean distance between them using
            the pixels dimension.
        """
        x_squared_dist = (self._pixel_dim[0] * abs(p1.x - p2.x)) ** 2
        y_squared_dist = (self._pixel_dim[1] * abs(p1.y - p2.y)) ** 2
        return math.sqrt(x_squared_dist + y_squared_dist)

    def _gen_random_point_under_constraints(self):
        """
            This method return a random point in the map's main area (bounded in _bound_map_main_area)
            whose height value is shorter then the flight's height.
        """
        while True:
            rand_x = np.random.uniform(self._x_lower_bound, self._x_upper_bound)
            rand_y = np.random.uniform(self._y_lower_bound, self._y_upper_bound)
            point = Point(rand_x, rand_y)
            # checking that the randomized point does not intersect with non of the obstacle polygons.
            if self._in_legal_flight_zone(point):
                return point

    def _adjust_stride(self):
        """Makes sure the stride length does not deviate from the maximum and minimum stride length values."""
        self._stride_length = self.__adjustment(self._stride_length, self.MAX_STRIDE_LEN, self.MIN_STRIDE_LEN)

    def _adjust_angle(self):
        """Makes sure the maximal turn angle does not deviate from the maximum and minimum angle values."""
        self._max_angle = self.__adjustment(self._max_angle, self.MAX_ANGLE, self.MIN_ANGLE)

    def __adjust_recursion_depth(self):
        """
        Makes sure the recursion depth does not deviate from the system's maximal value and the
        minimal value we defined.
        """
        self.__max_recursion_limit = int(self.__adjustment(float(self._max_recursion_limit),
                                                           sys.getrecursionlimit() - 10, self.MIN_RECURSION_DEPT))

    def _get_orientation_degree(self, cur_pos: Point, prev_pos: Point):
        """
        Receives two points and return the angle between the vector connecting these two points and the x axis.
        """
        world_cur_pos = self.__convert_map_point_to_world_point(cur_pos)
        world_prev_pos = self.__convert_map_point_to_world_point(prev_pos)
        cur_orientation = Point(world_cur_pos.x - world_prev_pos.x, world_cur_pos.y - world_prev_pos.y)
        return (math.degrees(math.atan2(cur_orientation.y, cur_orientation.x)) + 360) % 360

    def _get_possible_strides(self, cur_pos, prev_pos=None):
        """
            receives the current position and a previous position (so we can calculate orientation)
            and returns number of stride options. each stride option will make the drone turn in an angle
            that is at most max_angle degrees.
            Note:   - the number of stride options depends on the DEGREE_DIVISOR variable.
                    - if prev_pos is None the orientation is randomized
                    - the returned strides are in the maps (and not world) coordinates
        """
        if prev_pos is None:
            cur_deg = random.uniform(0, 360)
        else:
            cur_deg = self._get_orientation_degree(cur_pos, prev_pos)
        degrees1 = [cur_deg + (self._max_angle * float(i) / float(self.DEGREE_DIVISOR))
                    for i in range(1, self.DEGREE_DIVISOR + 1)]
        degrees2 = [cur_deg - (self._max_angle * float(i) / float(self.DEGREE_DIVISOR))
                    for i in range(1, self.DEGREE_DIVISOR + 1)]
        possible_degrees = [cur_deg] + degrees1 + degrees2
        orientations = []
        for degree in possible_degrees:
            point = Point(self._stride_length * math.cos(math.radians(degree)),
                          self._stride_length * math.sin(math.radians(degree)))
            orientations.append(self.__convert_world_point_to_map_point(point))
        return orientations

    def _remove_closest_step(self, strides: list, step: Point):
        for i in range(len(strides)):
            if abs(strides[i].x - step.x) <= self.EPSILON and abs(strides[i].y - step.y) <= self.EPSILON:
                del strides[i]
                return

    def __remove_already_inside(self, neighbor: list, open_set):
        world_nei = self.__convert_map_point_to_world_point(Point(neighbor[0], neighbor[1]))
        to_remove = []
        for i in open_set:
            world_i = self.__convert_map_point_to_world_point(Point(i[:2][0], i[:2][1]))
            if self.__euclidean_distance(world_i, world_nei) < self._min_stride_distance:
                to_remove.append(i)
        for i in to_remove:
            open_set.remove(i)

    @staticmethod
    def __is_zero(val):
        return -2 <= int(val) <= 2

    @staticmethod
    def __adjustment(value: float, max_value: float, min_value: float):
        if value > max_value:
            return max_value
        if value < min_value:
            return min_value
        return value
