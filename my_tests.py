import time
import math
from pathlib import Path
from scipy.spatial.qhull import ConvexHull
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure

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


def test_class_usage():
    Inputpath = Path(__file__).parent.absolute()
    FileName = 'dsm_binary'
    _, _, _, x_org, y_org, z_org, Wx, Wy, dWx, dWy, dsm_ = DSMParcer(Inputpath, FileName, False)

    initial_time = time.time()
    pg = PathGenerator(velocity=7, flight_height=30, dsm=dsm_, origin=(x_org, y_org, z_org),
                       map_dimensions=(Wx, Wy), pixel_dimensions=(dWx, dWy))
    print(f"PathGenerator initiation time is: {time.time() - initial_time}")
    pg.print_path()


def test_python_modulo():
    # tan 30 value for zero degrees
    Radians30 = math.radians(30)
    tan30 = math.tan(Radians30)
    tan30Rounded = round(tan30, 2)

    radians = math.radians(15)
    negative_radians = math.radians(-15)
    print(((400 + 360) % 360))
    print(f"15 radians: {radians}, -15 radians: {negative_radians}")


def test_math_cos():
    # cos θ value for zero degrees
    radians0 = math.radians(0)
    cos0 = math.tan(radians0)
    cos0_rounded = round(cos0, 2)
    # cos 30 value for zero degrees
    radians30 = math.radians(30)
    cos30 = math.tan(radians30)
    cos30_rounded = round(cos30, 2)
    # cos 45 value for zero degrees
    radians45 = math.radians(45)
    cos45 = math.tan(radians45)
    cos45_rounded = round(cos45, 2)
    # cos 60 value for zero degrees
    radians60 = math.radians(60)
    cos60 = math.tan(radians60)
    cos60_rounded = round(cos60, 2)
    # cos 90 value for zero degrees
    radians90 = math.radians(90)
    cos90 = math.tan(radians90)
    cos90_rounded = round(cos90, 2)
    # math.tan(radians) : Python Example: Printing tangent θ values for common angles
    print("cos 0: {}".format(cos0_rounded))
    print("cos 30: {}".format(cos30_rounded))
    print("cos 45: {}".format(cos45_rounded))
    print("cos 60: {}".format(cos60_rounded))
    print("cos 90: {}".format(cos90_rounded))

    indices = [float(i-3600)/10 for i in range(3*3600)]
    cos_list = [math.cos(math.radians(indices[i])) for i in range(len(indices))]
    plt.figure(1)
    plt.plot(indices, [0 for i in range(len(indices))], 'k-')
    plt.plot(indices, cos_list, 'g-')
    plt.show()
    sin_list = [math.sin(math.radians(indices[i])) for i in range(len(indices))]
    plt.figure(1)
    plt.plot(indices, [0 for i in range(len(indices))], 'k-')
    plt.plot(indices, sin_list, 'g-')
    plt.show()


def test_math_acos_asin():
    indices = [float(i)/10 for i in range(3600)]
    cos_list = [math.cos(math.radians(indices[i])) for i in range(len(indices))]
    acos_list = [math.degrees(math.acos(cos_list[i])) for i in range(len(indices))]
    plt.figure(1)
    plt.plot(indices, cos_list, 'g-')
    plt.show()
    plt.figure(2)
    plt.plot(cos_list, acos_list, 'b-')
    plt.show()


def test_math_atan2():
    print(math.degrees(math.atan2(1, 1)))
    print(math.degrees(math.atan2(1, -1)))
    print(math.degrees(math.atan2(-1, -1)))
    print(math.degrees(math.atan2(-1, 1)))

    print(math.cos(math.atan2(1, 1)))
    print(math.cos(math.atan2(1, -1)))
    print(math.cos(math.atan2(-1, -1)))
    print(math.cos(math.atan2(-1, 1)))


def yielding_method():
    list1 = [0, 1, 2, 3, 4, 5, 6]
    for idx in list1:
        print(f"yielding {idx}")
        yield idx


def test_yield():
    for item in yielding_method():
        print(item)


def test_possible_degrees():
    DEGREE_DIVISOR = 8
    cur_deg = 0.0
    max_angle = 180.0
    possible_degrees = [cur_deg] + [(cur_deg + max_angle) * i / DEGREE_DIVISOR for i in range(1, DEGREE_DIVISOR + 1)] +\
                       [(cur_deg - max_angle) * i / DEGREE_DIVISOR for i in range(1, DEGREE_DIVISOR + 1)]
    print(possible_degrees)
    return possible_degrees


def test_possible_strides():
    possible_degrees = test_possible_degrees()
    strides = []
    for degree in possible_degrees:
        strides.append([math.cos(math.radians(degree)), math.sin(math.radians(degree))])
    print(strides)
    np_points = np.array(strides)
    plt.plot(np_points[:, 0], np_points[:, 1], 'o')
    plt.show()


def test_list():
    l = [0]
    print(l[0])
    print(l[-1])


def test_list_copy():
    l1 = [0, 1, 2, 3]
    l2 = l1.copy()
    print(l1)
    print(l2)
    del l2[2]
    print(l1)
    print(l2)


def zip_test():
    pos1 = [-3, 4]
    pos2 = [77, 12]
    print([sum(x) for x in zip(pos1, pos2)])


def slicing_test(idx):
    l1 = [0, 1, 2, 3]
    l2 = l1[0: idx] + l1[idx + 1:]
    print(l1)
    print(l2)
    del l1[0]
    print(l1)
    print(l2)

# TODO: test rtree intersections.


if __name__ == "__main__":
    slicing_test(3)
