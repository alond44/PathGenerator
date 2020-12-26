import time
import numpy as np
from pathlib import Path

from DSM_Paths.DsmParser import create_map
from DSM_Paths.path_generator import PathGenerator, PathType, ConstraintType


def expansion_test():
    print(f"*************** expansion_test ***************")
    dsm_ = [[0, 1, 2, 3, 4], [5, 6, 7, 8, 9], [10, 11, 12, 13, 14], [15, 16, 17, 18, 19], [20, 21, 22, 23, 24]]
    pg = PathGenerator(50, 8, dsm_)
    print("Starting dsm:")
    pg.print_path()
    pg.map_zoom_out(2)
    print("After zooming out:")
    pg.print_path()
    pg.map_zoom_in(2)
    print("After zooming back in:")
    pg.print_path()
    print(f"\n\n\n")


def path_cost_test_1(pg, flag, path_type, desired_cost=1000, path_number=100, print_paths=False):
    if (flag == ConstraintType.DISTANCE or flag == ConstraintType.TIME) and\
            (path_type == PathType.MAP_ROAM or path_type == PathType.AREA_EXPLORE):
        print(f"\n************** Path Length Test 1  **************")
        print(f"                Path Type: \'{path_type}\'\n")
        print(f"Ran with desired_cost = {desired_cost} and path_number = {path_number}")

        paths = pg.gen_paths(flag=flag, constraint=desired_cost, path_type=path_type, to_print=print_paths,
                             path_num=path_number, weight=2)

        cost_sum = 0
        error_sum = 0
        for i in range(len(paths)):
            if flag == ConstraintType.DISTANCE:
                path_cost = pg.calc_path_distance(paths[i])
            else:
                path_cost = pg.calc_path_travel_time(paths[i])
            cost_sum += path_cost
            error_sum += abs(desired_cost - path_cost)
        avg_distance = cost_sum / path_number
        avg_error = error_sum / path_number
        if flag == ConstraintType.DISTANCE:
            cost_type = 'distance'
            unit = 'meters'
        else:
            cost_type = 'time'
            unit = 'seconds'
        print(f"The average {cost_type} of a path is: {avg_distance} {unit}")
        print(f"The average error is: {avg_error}")
    else:
        print("Wrong flag or path type value")


def path_generating_time_test(pg, flag, path_type, desired_cost=1000, path_number=100, print_paths=False):
    if (flag == 'd' or flag == 't') and (path_type == 'a_star' or path_type == 'prob'):
        print(f"\n************** Path Generating Time Test  **************")
        print(f"                 Path Type: \'{path_type}\'\n")
        print(f"Ran with desired_cost = {desired_cost} and path_number = {path_number}")

        start_time = time.time()

        paths = pg.gen_paths(flag=flag, constraint=desired_cost, path_type=path_type, to_print=print_paths,
                             path_num=path_number, weight=2)
        total_time = time.time() - start_time

        avg_time = total_time / path_number
        print(f"The average calculation time of a path of type \'{path_type}\' is: {avg_time} seconds")
    else:
        print("Wrong flag or path type value")


if __name__ == "__main__":
    Inputpath = Path(__file__).parent.absolute()
    FileName = 'dsm_binary'
    dsm_ = create_map(Inputpath, FileName)

    pg = PathGenerator(velocity=50, flight_height=50, dsm=dsm_, pixel_dist=2)
    # path_cost_test_1(pg, flag='d', path_type='prob', desired_cost=2000, print_paths=False, path_number=10)
    path_cost_test_1(pg, flag=ConstraintType.DISTANCE, path_type=PathType.MAP_ROAM, desired_cost=2000,
                     print_paths=True, path_number=2)
    pg.map_zoom_out(5)
    path_cost_test_1(pg, flag=ConstraintType.DISTANCE, path_type=PathType.MAP_ROAM, desired_cost=2000,
                     print_paths=True, path_number=2)

