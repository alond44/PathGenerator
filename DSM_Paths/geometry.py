# TODO: add scipy to the requirements
from scipy.spatial.qhull import ConvexHull
import numpy as np


class Point:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __str__(self):
        return f"({self.x}, {self.y})"

    def __repr__(self):
        return self.__str__()


class ConvexPolygon:

    def __init__(self, points: set):
        np_points = np.array(list(points))
        hull = ConvexHull(np_points)
        self.sorted_points = []
        self.min_x, self.max_x, self.min_y, self.max_y = np.inf, -np.inf, np.inf, -np.inf
        for point in np_points[hull.vertices, :]:
            p = Point(point[0], point[1])
            self.min_x, self.max_x = min(self.min_x, p.x), max(self.max_x, p.x)
            self.min_y, self.max_y = min(self.min_y, p.y), max(self.max_y, p.y)
            self.sorted_points.append(p)

    # TODO: document this method in our report.
    def line_intersect(self, p1: Point, p2: Point):
        """
            This method return True iff a line formed by the given two points intersects with the convex polygon.
        """
        sign = -1 if self._calc_signed_area(p1, p2, self.sorted_points[0]) < 0 else 1
        for i in range(1, len(self.sorted_points)):
            p3 = self.sorted_points[i]
            if self._calc_signed_area(p1, p2, p3) * sign < 0:
                return True
        return False

    @staticmethod
    def _calc_signed_area(p1: Point, p2: Point, p3: Point):
        return 0.5 * ((p2.x - p3.x) * (p3.y - p1.y) + (p2.y - p3.y) * (p1.x - p3.x))


