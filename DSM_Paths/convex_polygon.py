# TODO: add scipy to the requirements
from scipy.spatial.qhull import ConvexHull
import numpy as np


class ConvexPolygon:

    def __init__(self, points: set):
        np_points = np.array(list(points))
        hull = ConvexHull(np_points)
        self.sorted_points = np_points[hull.vertices, :]
        self.min_x, self.max_x = min(self.sorted_points[:, 0]), max(self.sorted_points[:, 0])
        self.min_y, self.max_y = min(self.sorted_points[:, 1]), max(self.sorted_points[:, 1])

    def line_intersect(self, p1: tuple, p2: tuple):
        pass


