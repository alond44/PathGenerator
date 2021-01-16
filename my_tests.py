import time
from pathlib import Path
from scipy.spatial.qhull import ConvexHull
import matplotlib.pyplot as plt

from DSM_Paths.DsmParser import DSMParcer
from DSM_Paths.path_generator import PathGenerator, PathType, ConstraintType

import numpy as np


def test_convex_hull():
    # generating a random point set.
    points = list(np.random.rand(30, 2))
    np_points = np.array(list(points))
    # calculating the convex hull.
    hull = ConvexHull(np_points)
    plt.plot(np_points[:, 0], np_points[:, 1], 'o')
    plt.show()

    sorted_points = np_points[hull.vertices, :]
    plt.plot(sorted_points[:, 0], sorted_points[:, 1], 'o')
    plt.show()

    plt.plot(sorted_points[:, 0], sorted_points[:, 1], 'r--', lw=2)
    plt.plot(np_points[:, 0], np_points[:, 1], 'o')
    plt.show()

    min_x, max_x = min(sorted_points[:, 0]), max(sorted_points[:, 0])
    min_y, max_y = min(sorted_points[:, 1]), max(sorted_points[:, 1])
    print(f"{min_x} <= x <= {max_x}")
    print(f"{min_y} <= y <= {max_y}")


if __name__ == "__main__":
    # test_convex_hull()
    Inputpath = Path(__file__).parent.absolute()
    FileName = 'dsm_binary'
    _, _, _, x_org, y_org, z_org, Wx, Wy, dWx, dWy, dsm_ = DSMParcer(Inputpath, FileName, False)

    initial_time = time.time()
    pg = PathGenerator(velocity=7, flight_height=30, dsm=dsm_, origin=(x_org, y_org, z_org),
                       map_dimensions=(Wx, Wy), pixel_dimensions=(dWx, dWy))
    print(f"PathGenerator initiation time is: {time.time() - initial_time}")
    pg.print_path()
