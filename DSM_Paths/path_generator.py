import os
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import math
import cv2
import time
from DSM_Paths.DsmParser import create_map


class PathGenerator:
    """
    This is the main class. To create paths, create an instance of it with the desired parameters.
    """
    def __init__(self, velocity, flight_height, dsm=None, pixel_dist=2):
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
            The distance of a pixel's width in real life.
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
                            val = 0 if x_idx >= prev_side or y_idx >= prev_side else self._dsm[x_idx][y_idx]
                            maxi = max(maxi, val)
                    new_dsm[x][y] = maxi
            self._pixel_dist = self._pixel_dist*multi

        # self._dsm.clear()
        self._dsm = new_dsm
        assert self._map_side == len(self._dsm)
        self._map_side = len(self._dsm)

    def print_map_size(self):
        print(f'self._map_side: {self._map_side}')
        print(f'len(self._dsm): {len(self._dsm)}')
        print(f'len(self._dsm[0]): {len(self._dsm[0])}')

    def gen_path_dist(self):
        # TODO: implement
        pass

    def gen_path_time(self):
        # TODO: implement
        pass

    def gen_paths(self, flag, constrain, num=1, to_print=False):
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
                to_print : bool
                    Pass True to print the resulting paths over the dsm map.
        """
        if flag != 'd' and flag != 't':
            print('Wrong flag option!')
            return

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
        plot_style = path_color + path_style
        plt.figure(1)
        im = plt.imshow(self._dsm)
        if points is not None and len(points) >= 2:
            plt.plot(points[0], points[1], plot_style)
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
    Inputpath = 'C:/Users/alond/PycharmProjects/DSM_Paths'
    FileName = 'dsm_binary'
    dsm_ = create_map(Inputpath, FileName)
    # dsm_ = np.array([[0, 1, 2, 3], [4, 5, 6, 7], [8, 9, 10, 11], [12, 13, 14, 15]])
    pg = PathGenerator(50, 100, dsm_)
    print('Initial map: ')
    # for x in range(len(pg._dsm)):
    #    print(pg._dsm[x])
    pg.print_path([[], []])
    pg.resize_dsm(2)
    print('After expansion: ')
    # for x in range(len(pg._dsm)):
    #    print(pg._dsm[x])
    pg.print_path([[], []])
    pg.resize_dsm(2, False)
    print('After reverting expansion: ')
    # for x in range(len(pg._dsm)):
    #    print(pg._dsm[x])
    pg.print_path([[], []])

    # pg2 = PathGenerator(50, 100, dsm_)
    # pg2.print_path()


    """
    Inputpath = 'C:/Users/alond/PycharmProjects/DSM_Paths'
    FileName = 'dsm_binary'
    dsm = create_map(Inputpath, FileName)
    # path = [(100, 200), (300, 200), (300, 300), (300, 500)]
    path = [[100, 300, 300, 300], [200, 200, 300, 500]]
    print_path(dsm, path)
    """

