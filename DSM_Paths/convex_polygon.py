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

    # TODO: add circles surrounding the checked points.
    def line_intersect(self, p1: list, p2: list):
        sign = -1 if self._calc_signed_area(p1, p2, self.sorted_points[0]) < 0 else 1
        for i in range(1, len(self.sorted_points)):
            if self._calc_signed_area(p1, p2, self.sorted_points[i]) * sign < 0:
                return True
        return False

    @staticmethod
    def _calc_signed_area(p1, p2, p3):
        return 0.5 * ((p2[0] - p3[0]) * (p3[1] - p1[1]) + (p2[1] - p3[1]) * (p1[0] - p3[0]))


