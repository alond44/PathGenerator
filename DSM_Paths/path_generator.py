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
        if enhance <= 0:
            return
        self._map_side *= multi
        new_dsm = [[0 for j in range(self._map_side)] for i in range(self._map_side)]
        if enhance:
            for x in range(0, len(new_dsm)):
                for y in range(0, len(new_dsm)):
                    new_dsm[x][y] = self._dsm[int(x/multi)][int(y/multi)]
            self._pixel_dist = self._pixel_dist/multi
        else:
            # TODO: make sure its correct
            for x in self._dsm:
                curr_line = []
                for y in self._dsm[x]:
                    curr_line += [self._dsm[x][y] * multi]
                new_dsm.append(curr_line)
            self._pixel_dist = self._pixel_dist*multi

        self._dsm.clear()
        self._dsm = new_dsm
        self._map_side = len(self._dsm)



    def gen_path_dist(self):
        # TODO: implement
        pass


def print_path(dsm, points, path_color='r', path_style='--'):
    """
    Outputs the map with the given path on it.

     Parameters
    ----------
    dsm : 2D matrix
        The DSM map.
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
    im = plt.imshow(dsm)
    plt.plot(points[0], points[1], plot_style)
    plt.show()


if __name__ == "__main__":
    Inputpath = 'C:/Users/alond/PycharmProjects/DSM_Paths'
    FileName = 'dsm_binary'
    dsm = create_map(Inputpath, FileName)
    print_path(dsm, [])
    print(len(dsm))
    t = time.time()
    new_dsm = [[0 for j in range(len(dsm)*4)] for i in range(len(dsm)*4)]
    for x in range(0, len(new_dsm)):
        for y in range(0, len(new_dsm)):
            new_dsm[x][y] = dsm[int(x / 4)][int(y / 4)]
    print(f"Initiation time: {time.time() - t}")
    t = time.time()
    print_path(new_dsm, [])
    print(f"Print time: {time.time() - t}")
    """
    Inputpath = 'C:/Users/alond/PycharmProjects/DSM_Paths'
    FileName = 'dsm_binary'
    dsm = create_map(Inputpath, FileName)
    # path = [(100, 200), (300, 200), (300, 300), (300, 500)]
    path = [[100, 300, 300, 300], [200, 200, 300, 500]]
    print_path(dsm, path)
    """

