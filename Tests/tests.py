import time
from pathlib import Path

from DSM_Paths.DsmParser import DSMParcer
from DSM_Paths.path_generator import PathGenerator, PathType, ConstraintType


""" ***************  Result Display  *************** """


def simple_example(pg: PathGenerator):
    print("****************  Simple Example  ****************")
    # Probabilistic path
    pg.gen_paths(flag=ConstraintType.DISTANCE, constraint=1500, path_type=PathType.MAP_ROAM, start_location=None,
                 path_num=1, to_print=True, weight=2)
    # Probabilistic path with random turns
    pg.gen_paths(flag=ConstraintType.DISTANCE, constraint=1500, path_type=PathType.AREA_EXPLORE, start_location=None,
                 path_num=1, to_print=True, weight=2)
    # Weighted A* path
    # TODO: fix the A* run with the Point class.
    # pg.gen_paths(flag=ConstraintType.DISTANCE, constraint=1500, path_type=PathType.A_STAR, start_location=None,
    #              path_num=1, to_print=True, weight=2)


def get_paths_constraint_error(pg, flag, path_type, desired_cost=1000.0, path_number=100, print_paths=False, w=2.0):
    if (flag == ConstraintType.DISTANCE or flag == ConstraintType.TIME) and \
            (path_type == PathType.MAP_ROAM or path_type == PathType.AREA_EXPLORE):

        paths = pg.gen_paths(flag=flag, constraint=desired_cost, path_type=path_type, to_print=print_paths,
                             path_num=path_number, weight=w)
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
        return avg_error, avg_distance
    else:
        print("Wrong flag or path type value")


def path_generating_error_test(pg: PathGenerator, flag: ConstraintType, desired_cost: float, path_num: int):
    if flag == ConstraintType.DISTANCE:
        constraint = "distance"
    else:
        constraint = "time"

    print(f"\n************** Path Error Test  **************\n")
    print("\nInitial averages:")
    initial_Astar_error_avg, initial_Astar_distance_avg = get_paths_constraint_error(pg, flag=flag,
                                                                                     path_type=PathType.MAP_ROAM,
                                                                                     desired_cost=desired_cost,
                                                                                     path_number=path_num, w=2.0)
    initial_Prob_error_avg, initial_Prob_distance_avg = get_paths_constraint_error(pg, flag=flag,
                                                                                   path_type=PathType.AREA_EXPLORE,
                                                                                   desired_cost=desired_cost,
                                                                                   path_number=path_num)
    print(f"Probabilistic:")
    print(f"Average error: {initial_Astar_error_avg}\nAverage {constraint}: {initial_Astar_distance_avg}")
    print(f"Probabilistic with turns:")
    print(f"Average error: {initial_Prob_error_avg}\nAverage {constraint}: {initial_Prob_distance_avg}")


def get_path_generating_duration(pg, flag, path_type, desired_cost=1000.0, path_number=100, print_paths=False, w=2.0):
    if (flag == ConstraintType.DISTANCE or flag == ConstraintType.TIME) and \
            (path_type == PathType.MAP_ROAM or path_type == PathType.AREA_EXPLORE):

        start_time = time.time()
        pg.gen_paths(flag=flag, constraint=desired_cost, path_type=path_type, to_print=print_paths,
                     path_num=path_number, weight=w)
        total_time = time.time() - start_time

        avg_time = total_time / path_number
        return avg_time
    else:
        print("Wrong flag or path type value")


def path_generating_calculation_time_test(pg: PathGenerator, flag: ConstraintType, desired_cost: float, path_num: int):
    if flag == ConstraintType.DISTANCE:
        constraint = "distance"
    else:
        constraint = "time"
    print(f"\n************** Path Calculation Time Test  **************\n")
    print("\nInitial time average:")
    initial_Astar_time_average = get_path_generating_duration(pg, flag=flag, path_type=PathType.MAP_ROAM,
                                                              desired_cost=desired_cost, path_number=path_num, w=2.0)
    initial_Prob_time_average = get_path_generating_duration(pg, flag=flag, path_type=PathType.AREA_EXPLORE,
                                                             desired_cost=desired_cost, path_number=path_num)
    print(f"A*:")
    print(f"Average calculation time under {constraint} constraint: {initial_Astar_time_average}")
    print(f"Probabilistic:")
    print(f"Average calculation time under {constraint} constraint: {initial_Prob_time_average}")


if __name__ == "__main__":
    Inputpath = Path(__file__).parent.absolute()
    FileName = 'dsm_binary'
    _, _, _, x_org, y_org, z_org, Wx, Wy, dWx, dWy, dsm_ = DSMParcer(Inputpath, FileName, False)

    pg = PathGenerator(velocity=7.0, flight_height=-50.0, dsm=dsm_, origin=(x_org, y_org, z_org),
                       map_dimensions=(Wx, Wy), pixel_dimensions=(dWx, dWy), max_angle=35.0)

    # simple_example(pg)
    # path_generating_error_test(pg, ConstraintType.TIME, 100.0, path_num=100)
    # path_generating_calculation_time_test(pg, ConstraintType.TIME, 100.0, 100)
    paths = pg.gen_paths(ConstraintType.TIME, 100, PathType.A_STAR, path_num=5, to_print=True)
