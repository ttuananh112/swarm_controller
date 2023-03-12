import warnings
from collections import defaultdict
from copy import deepcopy
from typing import List, Dict, Set, Iterable, Union

import numpy as np
import pandas as pd

from src.common import logger
from src.common.constants import INFINITY, LOCAL_PATH_LENGTH, RobotsColumns
from src.common.utils import roll
from src.controller import AStar
from src.graph import Graph
from src.models import Vector, State
from src.models.robot import Robot

warnings.filterwarnings("ignore", category=RuntimeWarning)


class Mediator:
    """
    Class to control paths of all robot
    """

    def __init__(
            self,
            robot_container: Dict[int, Robot] = None,
            lookahead: int = LOCAL_PATH_LENGTH,
            bias_intersection: int = 1
    ):
        """
        Args:
            robot_container (Dict[int, Robot]): dict of robots
            lookahead (int): number of steps in local path
            bias_intersection (int): bias of steps while checking intersection
        """
        self.graph = None
        self.astar = None

        self.robot_container: Dict[int, Robot] = robot_container or {}
        self.global_paths: List[List[int]] = self.get_global_paths()

        self.lookahead = lookahead
        self.bias_intersection = bias_intersection
        self.padding_value = INFINITY

    def load_graph(self, vertices_path: str, edges_path: str):
        """
        Load graph from files
        Args:
            vertices_path:
            edges_path:

        Returns:

        """
        self.graph = Graph().load_data(vertices_path=vertices_path, edges_path=edges_path)
        self.astar = AStar(graph=self.graph)

    def update_graph(self, graph: Graph):
        """
        Update graph
        Args:
            graph:

        Returns:

        """
        self.graph = graph
        self.astar = self.astar.set_graph(self.graph)

    def get_graph(self):
        return self.graph

    def add_robots_from_file(self, robots_path: str):
        """
        Add robots from file
        Args:
            robots_path:

        Returns:

        """
        df = pd.read_csv(robots_path)
        for i, row in df.iterrows():
            self.add_robot(
                Robot(id_robot=int(row[RobotsColumns.ID]),
                      state=State(position=self.graph.get_node_by_id(row[RobotsColumns.START_NODE]).position,
                                  velocity=row[RobotsColumns.VELOCITY],
                                  heading=row[RobotsColumns.HEADING],
                                  omega=row[RobotsColumns.OMEGA]),
                      global_path=[int(i) for i in self.astar.find_path(
                          row[RobotsColumns.START_NODE], row[RobotsColumns.STOP_NODE])])
            )

    def get_robot_container(self) -> Dict[int, Robot]:
        """
        Get robot container
        Returns:
            (Dict[int, Robot])
        """
        return self.robot_container

    def get_robot_by_id(self, id: int):
        """
        Get robot by id
        Args:
            id (int): robot id

        Returns:

        """
        return self.robot_container[id]

    def add_robot(self, robot: Robot):
        """
        Add new robot to self.robot_container
        Args:
            robot (Robot): new robot
        """
        if robot.id_robot not in self.robot_container:
            self.robot_container[robot.id_robot] = robot
            self.global_paths = self.get_global_paths()
        else:
            logger.error(f"Robot with id = {robot.id_robot} exists")

    def remove_robot_by_id(self, id_robot: int):
        """
        Remove robot from self.robot_container by its id_robot
        Args:
            id_robot (int): id of robot
        """
        if id_robot in self.robot_container:
            del self.robot_container[id_robot]
            self.global_paths = self.get_global_paths()
        else:
            logger.error(f"Robot with id = {id_robot} not found. Can not remove")

    def get_global_paths(self) -> List[List[int]]:
        """
        Get list of global paths
        Returns:
            (List[List[int]])
        """
        return [robot.get_current_to_goal() for robot in self.robot_container.values()]

    def update_global_path(self, id_robot: int, global_path: List[int]):
        """
        Update global path of robot by id_robot
        Args:
            id_robot (int): id of robot to be updated global path
            global_path (List[int]): new global path
        """
        if id_robot in self.robot_container:
            self.robot_container[id_robot].set_global_path(global_path)
            self.global_paths = self.get_global_paths()
        else:
            logger.error(f"Robot with id = {id_robot} not found. Can not update global path")

    def pad(self, path: List[int]) -> np.ndarray:
        """
        Padding path by self.padding_value
        Args:
            path (List[int]): path to be padded

        Returns:
            (np.ndarray): padded path
        """
        to_pad = self.lookahead - len(path)
        return np.pad(
            np.asarray(path, dtype=np.float32),
            pad_width=(0, to_pad),
            mode="constant",
            constant_values=self.padding_value
        )

    def cut_lookahead(self, global_paths: List[List[int]]) -> np.ndarray:
        """
        Cut off path to lookahead length
        Args:
            global_paths (List[List[int]]): global paths

        Returns:
            (np.ndarray): cut paths by lookahead distance
        """
        container = [self.pad(path[:self.lookahead]) for path in global_paths]
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
            Handled delay response from robot case (considered bias_intersection)
            Returns:
                (Dict[int, Set]):
                    key: node of intersection
                    value: set of path's index
            """
            container = defaultdict(set)
            for shift in range(-self.bias_intersection, self.bias_intersection + 1):
                expanded = np.expand_dims(local_paths, axis=1)
                rolled = roll(local_paths,
                              shift=shift, axis=1,
                              padding_value=self.padding_value)
                diff = expanded - rolled

                i_path1, i_path2, i_intersect = np.where(diff == 0)
                mask = i_path1 != i_path2
                intersection = pd.DataFrame({
                    "i_path1": i_path1[mask],
                    "i_path2": i_path2[mask],
                    "i_intersect": i_intersect[mask]
                }).groupby(by=["i_path1", "i_path2"]).agg({"i_intersect": "first"}).reset_index()

                for i, row in intersection.iterrows():
                    i_path1, i_path2, first_index_intersect = row
                    intersect_node = local_paths[i_path1][first_index_intersect]
                    container[intersect_node].add(i_path1)
                    container[intersect_node].add(i_path2)
            return container

        def _get_priority(intersections: Dict[int, Set]) -> Iterable:
            """
            Get the priority of each path at intersection
            Args:
                intersections (Dict[int, Set]): node of intersection, each point have set of path's index

            Returns:
                (Iterable): node, i_node_on_path, sorted path's index by ascending step left to goal order
            """
            for intersect_node, set_of_id_paths in intersections.items():
                if len(set_of_id_paths) == 0:
                    continue

                list_priority = []
                for i_path in set_of_id_paths:
                    glob_path = self.global_paths[i_path]
                    i_node = np.where(glob_path == intersect_node)[0][0]
                    len_left = len(glob_path) - i_node - 1
                    list_priority.append((i_path, len_left))

                yield intersect_node, sorted(list_priority, key=lambda data: data[1])

        # get intersections
        intersection_points = _get_intersection_points()
        # get priority at intersection by its left steps to goal
        new_local_paths = deepcopy(local_paths)
        for priority in _get_priority(intersection_points):
            intersect_node, i_paths = priority
            i_paths = [item[0] for item in i_paths]
            # the nearest to goal can go
            # the other will stop before intersection
            for i_other_path in i_paths[1:]:
                i_node_on_path = np.where(local_paths[i_other_path] == intersect_node)[0][0]
                new_local_paths[i_other_path][i_node_on_path:] = self.padding_value
        return new_local_paths

    def find_local_paths(self) -> Dict[int, Union[List[int], np.ndarray]]:
        """
        Main function to get local paths, follows by steps:
            - cut by lookahead step
            - cut if meet other robot in rail
            - cut if collision with others in intersection

        Returns:
            Dict: (id_robot, local_path)
        """
        local_paths = self.cut_lookahead(global_paths=self.get_global_paths())
        local_paths = self.cut_if_meet_other_robot(local_paths=local_paths)
        local_paths = self.cut_in_intersection(local_paths=local_paths)

        for i, id_robot in enumerate(self.robot_container.keys()):
            self.robot_container[id_robot].set_local_path(local_paths[i])

        # simplify output
        result = {}
        for id_robot, robot in self.robot_container.items():
            result[id_robot] = [int(node) for node in robot.get_local_path() if node != np.inf]
        return result

    def update(self, dt: float):
        local_paths = self.find_local_paths()
        for id_robot in self.robot_container.keys():
            if len(local_paths[id_robot]) > 1:
                curr_id = local_paths[id_robot][0]
                curr_pos = self.graph.get_node_by_id(curr_id).position
                next_id = local_paths[id_robot][1]
                next_pos = self.graph.get_node_by_id(next_id).position
                curr_to_next_pos = next_pos - curr_pos
                i_pos = Vector(x=1, y=0)
                up_coming_heading = curr_to_next_pos.angle_between(i_pos) * np.sign(curr_to_next_pos.y or 1)

                # update state of robot
                self.robot_container[id_robot].update_state(up_coming_heading, dt)

                # update current position if reaching point
                if self.robot_container[id_robot].state.position == next_pos:
                    self.robot_container[id_robot].set_current_position(next_id)
                    self.robot_container[id_robot].state.position = next_pos

        return local_paths
