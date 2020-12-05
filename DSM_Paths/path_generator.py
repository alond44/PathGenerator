import os
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import random
import math
import time
from pathlib import Path
from DSM_Paths.DsmParser import create_map


class PathGenerator:
    """
    This is the main class. To create paths, create an instance of it with the desired parameters.
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
            print('Need to initiate map using init_map before we start.')  # commit check

    def init_map(self, input_path=None, file_name=None, saveTIF=True, pixel_dist=2):
        if input_path is not None and file_name is not None:
            self._dsm = create_map(input_path, file_name, saveTIF)
            self._map_side = len(self._dsm)
            self._pixel_dist = pixel_dist

    def resize_dsm(self, multi, enhance=True):
        """
        Enhancing the dsm resolution by the given multiplier
         Parameters
        ---------
        multi : int
            The enhance multiplier. Must be > 0.
        enhance : bool
            True for enhancing and false for decrease resolution.

         Notes
        ---------
        Decreasing resolution might cause information loss.
        """
        if multi <= 0:
            return
        if enhance:
            self._map_side *= multi
            new_dsm = [[0 for j in range(self._map_side)] for i in range(self._map_side)]
            for x in range(0, len(new_dsm)):
                for y in range(0, len(new_dsm)):
                    new_dsm[x][y] = self._dsm[int(x/multi)][int(y/multi)]
            self._pixel_dist = self._pixel_dist/multi
        else:
            # TODO: make sure its correct
            prev_side = self._map_side
            self._map_side = int(prev_side/multi) if prev_side % multi == 0 else 1 + int(prev_side/multi)
            new_dsm = [[0 for j in range(self._map_side)] for i in range(self._map_side)]
            for x in range(0, len(new_dsm)):
                for y in range(0, len(new_dsm)):
                    maxi = -np.inf
                    for i in range(0, multi):
                        for j in range(0, multi):
                            x_idx = x*multi + i
                            y_idx = y*multi + j
                            val = -np.inf if x_idx >= prev_side or y_idx >= prev_side else self._dsm[x_idx][y_idx]
                            maxi = max(maxi, val)
                    new_dsm[x][y] = maxi
            self._pixel_dist = self._pixel_dist*multi

        self._dsm = new_dsm
        self._map_side = len(self._dsm)

    def print_map_size(self):
        print(f'self._map_side: {self._map_side}')
        print(f'len(self._dsm): {len(self._dsm)}')
        print(f'len(self._dsm[0]): {len(self._dsm[0])}')

    def __inside_array(self, pos):
        return self._dsm.shape[0] > pos[0] >= 0 and self._dsm.shape[1] > pos[1] >= 0

    def __gen_path_probability(self, start_location: list, max_len: float):
        directions = [[-1, 0], [1, 0], [0, -1], [0, 1]]
        cur_pos = start_location
        path = [cur_pos]
        left_len = max_len
        last_move = None
        while True:
            relevant_dirs = []
            for dir in directions:
                new_pos = [sum(x) for x in zip(dir, cur_pos)]
                if self.__inside_array(new_pos):
                    if self._dsm[new_pos[0]][new_pos[1]] <= self._flight_height:
                        relevant_dirs += [new_pos]
            #TODO: Change!
            if last_move is not None:
                relevant_dirs.remove([sum(x) for x in zip(dir, cur_pos)])
            new_pos = random.choice(relevant_dirs)
            last_move = [new_pos[0] - cur_pos[0], new_pos[1] - cur_pos[1]]
            path += [new_pos]
            left_len -= self._pixel_dist
            if left_len < 0:
                path.pop()
                return path
            cur_pos = new_pos
        pass

    def __h(self, location, goal, epsilon):
        return (abs(goal[0] - location[0]) + abs(goal[1] - location[1])) * self._pixel_dist * epsilon

    def a_star_epsilon(self, start_location, end_location, epsilon):
        open_set = {tuple(start_location)}
        cameFrom = {}
        g_score = {tuple(start_location): 0}
        f_score = {tuple(start_location): self.__h(start_location, end_location, epsilon)}

        while bool(open_set):
            current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
            if current == tuple(end_location):
                path = [list(current)]
                while current in cameFrom.keys():
                    current = cameFrom.get(current)
                    path.insert(0, list(current))
                return path

            open_set.remove(current)
            directions = [[-1, 0], [1, 0], [0, -1], [0, 1]]
            for dir in directions:
                neighbor = [sum(x) for x in zip(dir, list(current))]
                if self.__inside_array(neighbor):
                    #TODO: change later to <=
                    if self._dsm[neighbor[0]][neighbor[1]] >= self._flight_height:
                        tentative_g_score = g_score.get(current, float('inf')) + self._pixel_dist
                        if tentative_g_score < g_score.get(tuple(neighbor),float('inf')):
                            cameFrom[tuple(neighbor)] = current
                            g_score[tuple(neighbor)] = tentative_g_score
                            f_score[tuple(neighbor)] = g_score.get(tuple(neighbor),float('inf')) + self.__h(neighbor, end_location, epsilon)
                            if tuple(neighbor) not in open_set:
                                open_set.add(tuple(neighbor))
        return []

    def __gen_path_a_star_epsilon(self, start_location: list, max_len: float, epsilon: float = 1.0):
        path = []
        cur_start = start_location
        while True:
            while True:
                rand_x = random.randrange(self._dsm.shape[0])
                rand_y = random.randrange(self._dsm.shape[1])
                #TODO: change later to <=
                if self._dsm[rand_x][rand_y] >= self._flight_height:
                    break
            path += self.a_star_epsilon(cur_start, [rand_x, rand_y], epsilon)
            cur_start = path[-1]
            if len(path) * self._pixel_dist > max_len:
                return path[:int(max_len / self._pixel_dist)]

    def gen_paths(self, flag, constrain, path_type, start_location, num=1, to_print=False, epsilon=1.0):
        """
                Enhancing the dsm resolution by the given multiplier
                 Parameters
                ---------
                flag : str
                    Will hold either 'd' for distance constrain or 't' for time constrain.
                constrain : float
                    The paths' length in meters for distance constrained paths or the travel time for time
                    in seconds constrained paths.
                num : int
                    The number of paths wanted.
                path_type
                    The kind of path generating algorithm used.
                    Will hold either 'prob' for probability random path and 'a_star' for a_star generated path to random spots.
                to_print : bool
                    Pass True to print the resulting paths over the dsm map.
        """

        if flag != 'd' and flag != 't':
            print('Wrong flag option!')
            return
        if constrain <= 0:
            print('Constrain should be positive')
            return
        orig_pixel_dist = self._pixel_dist
        if flag == 't':
            self._pixel_dist = orig_pixel_dist * self._velocity
        if path_type == 'prob':
            path = self.__gen_path_probability(start_location= start_location, max_len=constrain)
        elif path_type == 'a_star':
            path = self.__gen_path_a_star_epsilon(start_location= start_location, max_len= constrain, epsilon= epsilon)
        else:
            print('Path type is not one of the correct path generating ways')
            return
        self.print_path(path)
        self._pixel_dist = orig_pixel_dist

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
        x_list = []
        y_list = []
        for point in points:
            x_list += [point[0]]
            y_list += [point[1]]
        plot_style = path_color + path_style
        plt.figure(1)
        im = plt.imshow(self._dsm)
        if points is not None and len(points) >= 2:
            plt.plot(x_list, y_list, plot_style)
        plt.show()


def expansion_test():
    dsm_ = [[0, 1, 2, 3], [4, 5, 6, 7], [8, 9, 10, 11], [12, 13, 14, 15]]
    for x in range(len(dsm_)):
        print(dsm_[x])

    multi = 2
    _map_side = multi * len(dsm_)
    new_dsm = [[0 for j in range(_map_side)] for i in range(_map_side)]
    for x in range(0, len(new_dsm)):
        for y in range(0, len(new_dsm)):
            new_dsm[x][y] = dsm_[int(x / multi)][int(y / multi)]
    # self._pixel_dist = self._pixel_dist / multi
    dsm_ = new_dsm

    print('After expansion: ')
    for x in range(len(new_dsm)):
        print(new_dsm[x])

    prev_side = _map_side
    _map_side = int(prev_side / multi) if prev_side % multi == 0 else 1 + int(prev_side / multi)
    new_dsm = [[0 for j in range(_map_side)] for i in range(_map_side)]
    for x in range(0, len(new_dsm)):
        for y in range(0, len(new_dsm)):
            maxi = -1
            for i in range(0, multi):
                for j in range(0, multi):
                    x_idx = x * multi + i
                    y_idx = y * multi + j
                    val = 0 if x_idx >= prev_side or y_idx >= prev_side else dsm_[x_idx][y_idx]
                    maxi = max(maxi, val)
            new_dsm[x][y] = maxi
    # self._pixel_dist = self._pixel_dist * multi

    print('After reverting expansion: ')
    for x in range(len(new_dsm)):
        print(new_dsm[x])


if __name__ == "__main__":
    Inputpath = Path(__file__).parent.absolute()
    FileName = 'dsm_binary'
    dsm_ = create_map(Inputpath, FileName)
    pg = PathGenerator(velocity=50, flight_height=-250, dsm=dsm_, pixel_dist=2)
    pg.gen_paths(flag='d', constrain=1000, path_type='a_star', start_location=[150, 150], to_print=True, epsilon=2)



    """
    Inputpath = 'C:/Users/alond/PycharmProjects/DSM_Paths'
    FileName = 'dsm_binary'
    dsm = create_map(Inputpath, FileName)
    # path = [(100, 200), (300, 200), (300, 300), (300, 500)]
    path = [[100, 300, 300, 300], [200, 200, 300, 500]]
    print_path(dsm, path)
    """

