from DSM_Paths.path_generator import PathGenerator
from DSM_Paths.DsmParser import create_map
import time
from pathlib import Path


def expansion_test():
    print(f"*************** expansion_test ***************")
    print("Using the methods logic:")
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

    print("Using the methods logic 2nd test:")
    dsm_ = [[0, 1, 2, 3, 4], [5, 6, 7, 8, 9], [10, 11, 12, 13, 14], [15, 16, 17, 18, 19], [20, 21, 22, 23, 24]]
    for x in range(len(dsm_)):
        print(dsm_[x])

    _map_side = len(dsm_)

    multi = 2
    prev_side = _map_side
    _map_side = int(prev_side / multi) if prev_side % multi == 0 else 1 + int(prev_side / multi)
    print(_map_side)
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

    print('After zooming out with a multiplier with value 2: ')
    for x in range(len(new_dsm)):
        print(new_dsm[x])
    print(f"\n\n\n")


def expansion_test_2():
    print(f"*************** expansion_test_2 ***************")
    dsm_ = [[0, 1, 2, 3, 4], [5, 6, 7, 8, 9], [10, 11, 12, 13, 14], [15, 16, 17, 18, 19], [20, 21, 22, 23, 24]]
    pg = PathGenerator(50, 8, dsm_)
    print("Starting dsm:")
    pg.print_map_size()
    pg.print_path()
    pg.resize_dsm(2, False)
    print("After zooming out:")
    pg.print_map_size()
    pg.print_path()
    pg.resize_dsm(2)
    print("After zooming out again:")
    pg.print_map_size()
    pg.print_path()
    print(f"\n\n\n")


def zero_value_test():
    Inputpath = Path(__file__).parent.absolute()
    FileName = 'dsm_binary'
    dsm_ = create_map(Inputpath, FileName)
    print(int(dsm_[0][0]) == -243)


def path_cost_test_1(dsm, flag, path_type, desired_cost=1000, path_number=100, print_paths=False):
    if (flag == 'd' or flag == 't') and (path_type == 'a_star' or path_type == 'prob'):
        print(f"\n************** Path Length Test 1  **************")
        print(f"                Path Type: \'{path_type}\'\n")
        print(f"Ran with desired_cost = {desired_cost} and path_number = {path_number}")
        pg = PathGenerator(velocity=50, flight_height=50, dsm=dsm, pixel_dist=2)
        paths = pg.gen_paths(flag=flag, constrain=desired_cost, path_type=path_type, to_print=print_paths,
                             path_num=path_number, epsilon=2)

        cost_sum = 0
        error_sum = 0
        for i in range(len(paths)):
            if flag == 'd':
                path_cost = pg.calc_path_distance(paths[i])
            else:
                path_cost = pg.calc_path_travel_time(paths[i])
            cost_sum += path_cost
            error_sum += abs(desired_cost - path_cost)
        avg_distance = cost_sum / path_number
        avg_error = error_sum / path_number
        if flag == 'd':
            cost_type = 'distance'
            unit = 'meters'
        else:
            cost_type = 'time'
            unit = 'meters per second'
        print(f"The average {cost_type} of a path is: {avg_distance} {unit}")
        print(f"The average error is: {avg_error}")
    else:
        print("Wrong flag or path type value")


def path_generating_time_test(dsm, flag, path_type, desired_cost=1000, path_number=100, print_paths=False):
    if (flag == 'd' or flag == 't') and (path_type == 'a_star' or path_type == 'prob'):
        print(f"\n************** Path Generating Time Test  **************")
        print(f"                 Path Type: \'{path_type}\'\n")
        print(f"Ran with desired_cost = {desired_cost} and path_number = {path_number}")

        pg = PathGenerator(velocity=50, flight_height=50, dsm=dsm, pixel_dist=2)
        start_time = time.time()
        paths = pg.gen_paths(flag=flag, constrain=desired_cost, path_type=path_type, to_print=print_paths,
                             path_num=path_number, epsilon=2)
        total_time = time.time() - start_time

        avg_time = total_time / path_number
        print(f"The average calculation time of a path of type \'{path_type}\' is: {avg_time} seconds")
    else:
        print("Wrong flag or path type value")


if __name__ == "__main__":

    Inputpath = Path(__file__).parent.absolute()
    FileName = 'dsm_binary'
    dsm_ = create_map(Inputpath, FileName)

    path_cost_test_1(dsm_, flag='t', path_type='prob', desired_cost=50, print_paths=True, path_number=2)
    path_cost_test_1(dsm_, flag='t', path_type='a_star', desired_cost=50, print_paths=True, path_number=2)


'''
    path_cost_test_1(dsm_, 'd', 'prob')
    path_cost_test_1(dsm_, 'd', 'a_star')
    path_cost_test_1(dsm_, 't', 'a_star')
    path_cost_test_1(dsm_, 't', 'a_star')

    path_generating_time_test(dsm_, 'd', 'prob')
    path_generating_time_test(dsm_, 'd', 'a_star')

    path_generating_time_test(dsm_, 't', 'prob')
    path_generating_time_test(dsm_, 't', 'a_star')
'''
