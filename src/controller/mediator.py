from collections import defaultdict
from copy import deepcopy
from typing import List, Dict, Set, Iterable

import numpy as np


class Mediator:
    """
    Class to control paths of all robot
    """

    def __init__(
            self,
            global_paths: List[List[int]] = None,
            lookahead: int = 5,
            bias_intersection: int = 1
    ):
        """
        Args:
            global_paths (List[List[int]]): global paths
            lookahead (int): number of steps in local path
            bias_intersection (int): bias of steps while checking intersection
        """
        self.global_paths = global_paths
        self.lookahead = lookahead
        self.bias_intersection = bias_intersection
        self.padding_value = -1

    def set_global_paths(self, global_paths: List[List[int]]):
        """
        Set global paths function
        Args:
            global_paths (List[List[int]]): global paths
        """
        self.global_paths = global_paths

    def pad(self, path: List[int]) -> np.ndarray:
        """
        Padding path by self.padding_value
        Args:
            path (List[int]): path to be padded

        Returns:
            (np.ndarray): padded path
        """
        to_pad = self.lookahead - len(path)
        return np.pad(path, (0, to_pad), "constant", constant_values=self.padding_value)

    def cut_lookahead(self, global_paths: List[List[int]]) -> np.ndarray:
        """
        Cut off path to lookahead length
        Args:
            global_paths (List[List[int]]): global paths

        Returns:
            (np.ndarray): cut paths by lookahead distance
        """
        container = []
        for path in global_paths:
            container.append(self.pad(path[:self.lookahead]))
        return np.array(container)

    def cut_if_meet_other_robot(self, local_paths: np.ndarray) -> np.ndarray:
        """
        Cut off path if collision with other robot on rail
        New path should have stopping point behind obstacle
        Args:
            local_paths (np.ndarray): local path of robots

        Returns:
            (np.ndarray): new local paths
        """

        starting_points = local_paths[:, 0]
        forwarding_points = local_paths[:, 1:]
        mask = np.isin(forwarding_points, starting_points)
        accu = np.cumsum(mask, axis=1)
        # only get path that does not collision with other's starting point
        forwarding_points[accu != 0] = self.padding_value
        return local_paths

    def cut_in_intersection(self, local_paths) -> np.ndarray:
        """
        Cut off path if meeting in intersection
        Set priority by rule: the nearest to intersection will go, the others have to wait behind intersection
        """

        def _get_intersection_points() -> Dict[int, Set]:
            """
            Get all intersection points
            Returns:
                (Dict[int, Set]):
                    key: index of intersection
                    value: set of path's index
            """
            diff = np.expand_dims(local_paths, axis=1) - local_paths

            container = defaultdict(set)
            for i, res_mat in enumerate(diff):
                for j, v in enumerate(res_mat):
                    if j <= i:
                        continue

                    index_zeros = np.where(v == 0)[0]
                    if len(index_zeros) > 0:
                        first_intersection = index_zeros[0]
                        container[first_intersection].add(i)
                        container[first_intersection].add(j)
            return container

        def _get_priority(local_paths: np.ndarray, intersection_points: Dict[int, Set]) -> Iterable:
            """
            Get the priority of each path at intersection
            Args:
                local_paths (np.ndarray): local paths
                intersection_points (Dict[int, Set]): intersection points, each point have set of path's index

            Returns:
                (Iterable): node, i_node_on_path, sorted path's index by ascending step left to goal order
            """
            for i_local_intersect, set_of_id_paths in intersection_points.items():
                if len(set_of_id_paths) == 0:
                    continue

                priority = []
                node = None
                i_node_on_path = -1
                for i_path in set_of_id_paths:
                    node = local_paths[i_path][i_local_intersect]
                    glob_path = self.global_paths[i_path]
                    i_node_on_path = np.where(glob_path == node)[0][0]
                    len_left = len(glob_path) - i_node_on_path - 1
                    priority.append((i_path, len_left))

                yield node, i_node_on_path, sorted(priority, key=lambda data: data[1])

        # get intersections
        intersection_points = _get_intersection_points()
        # get priority at intersection by its left steps to goal
        new_local_paths = deepcopy(local_paths)
        for priority in _get_priority(local_paths=local_paths, intersection_points=intersection_points):
            node, i_node_on_path, i_paths = priority
            i_paths = [item[0] for item in i_paths]
            # the nearest to goal can go
            # the other will stop before intersection
            for i_other_path in i_paths[1:]:
                new_local_paths[i_other_path][i_node_on_path:] = self.padding_value
        return new_local_paths

    def get_local_paths(self):
        """
        Main function to get local paths, follows by steps:
            - cut by lookahead step
            - cut if meet other robot in rail
            - cut if collision with others in intersection
        Returns:

        """
        local_paths = self.cut_lookahead(global_paths=self.global_paths)
        local_paths = self.cut_if_meet_other_robot(local_paths=local_paths)
        local_paths = self.cut_in_intersection(local_paths=local_paths)
        return local_paths