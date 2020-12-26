import time
from pathlib import Path

from DSM_Paths.DsmParser import create_map
from DSM_Paths.path_generator import PathGenerator, PathType, ConstraintType


def simple_example(pg: PathGenerator):
    # Weighted A* path
    pg.gen_paths(flag=ConstraintType.DISTANCE, constraint=1500, path_type=PathType.MAP_ROAM, start_location=None,
                 path_num=1, to_print=True, weight=2)
    # Probabilistic path
    pg.gen_paths(flag=ConstraintType.DISTANCE, constraint=1500, path_type=PathType.AREA_EXPLORE, start_location=None,
                 path_num=1, to_print=True, weight=2)


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

    print(f"\n************** Path Distance Test  **************\n")
    print("\nInitial averages:")
    initial_Astar_error_avg, initial_Astar_distance_avg = get_paths_constraint_error(pg, flag=flag,
                                                                                     path_type=PathType.MAP_ROAM,
                                                                                     desired_cost=desired_cost,
                                                                                     path_number=path_num, w=2.0)
    initial_Prob_error_avg, initial_Prob_distance_avg = get_paths_constraint_error(pg, flag=flag,
                                                                                   path_type=PathType.AREA_EXPLORE,
                                                                                   desired_cost=desired_cost,
                                                                                   path_number=path_num)
    print(f"A*:")
    print(f"Average error: {initial_Astar_error_avg}\nAverage {constraint}: {initial_Astar_distance_avg}")
    print(f"Probabilistic:")
    print(f"Average error: {initial_Prob_error_avg}\nAverage {constraint}: {initial_Prob_distance_avg}")
    pg.map_zoom_out(2)
    print("\n\nAfter zooming out (2 times the initial map):")
    Astar_error_avg_zoom_out, Astar_distance_avg_zoom_out = get_paths_constraint_error(pg, flag=flag,
                                                                                       path_type=PathType.MAP_ROAM,
                                                                                       desired_cost=desired_cost,
                                                                                       path_number=path_num, w=2.0)
    Prob_error_avg_zoom_out, Prob_distance_avg_zoom_out = get_paths_constraint_error(pg, flag=flag,
                                                                                     path_type=PathType.AREA_EXPLORE,
                                                                                     desired_cost=desired_cost,
                                                                                     path_number=path_num)
    print(f"A*:")
    print(f"Average error: {Astar_error_avg_zoom_out}\nAverage {constraint}: {Astar_distance_avg_zoom_out}")
    print(f"Probabilistic:")
    print(f"Average error: {Prob_error_avg_zoom_out}\nAverage {constraint}: {Prob_distance_avg_zoom_out}")
    pg.map_zoom_in(4)
    print("\n\nAfter zooming in (2 times the initial map):")
    Astar_error_avg_zoom_in, Astar_distance_avg_zoom_in = get_paths_constraint_error(pg, flag=flag,
                                                                                     path_type=PathType.MAP_ROAM,
                                                                                     desired_cost=desired_cost,
                                                                                     path_number=path_num, w=2.0)
    Prob_error_avg_zoom_in, Prob_distance_avg_zoom_in = get_paths_constraint_error(pg, flag=flag,
                                                                                   path_type=PathType.AREA_EXPLORE,
                                                                                   desired_cost=desired_cost,
                                                                                   path_number=path_num)
    print(f"A*:")
    print(f"Average error: {Astar_error_avg_zoom_in}\nAverage {constraint}: {Astar_distance_avg_zoom_in}")
    print(f"Probabilistic:")
    print(f"Average error: {Prob_error_avg_zoom_in}\nAverage {constraint}: {Prob_distance_avg_zoom_in}")


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
    # simple_example(pg)
    # path_generating_error_test(pg, flag=ConstraintType.DISTANCE, desired_cost=2001, path_num=4)
    # path_generating_error_test(pg, flag=ConstraintType.TIME, desired_cost=50, path_num=4)

