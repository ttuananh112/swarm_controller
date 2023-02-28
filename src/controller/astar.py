from __future__ import annotations

from dataclasses import dataclass
from heapq import heappush, heappop
from typing import Iterable, Union

from src.common.constants import INFINITY
from src.graph.graph import Graph


@dataclass
class NodeStatus:
    """
    Status of node
    """
    data: int  # index of node
    g_score: float = INFINITY  # real distance from start to this node
    f_score: float = INFINITY  # g_score + heuristic score to goal node
    closed: bool = False  # flag to denote this node is in the shortest path
    out_open_set: bool = True  # flag to denote this node is searchable in iteration
    came_from: int = None  # previous node id

    def __lt__(self, b: NodeStatus) -> bool:
        """
        To consider which one is nearer
        Args:
            b (NodeStatus):

        Returns:
            (bool)
        """
        return self.f_score < b.f_score


class NodeStatusDict(dict):
    """
    Search node container
    """

    def __missing__(self, k: int) -> NodeStatus:
        """
        Assign value if missing key
        Args:
            k (int):

        Returns:
            (NodeStatus)
        """
        v = NodeStatus(k)
        self.__setitem__(k, v)
        return v


class AStar:
    """
    Astar algorithm
    """

    def __init__(self, graph: Graph):
        """
        Init graph
        Args:
            graph (Graph): graph
        """
        self.graph = graph

    def set_graph(self, graph: Graph):
        """
        Set new graph
        Args:
            graph (Graph): new graph
        """
        self.graph = graph

    def heuristic_cost_estimate(self, current_id: int, goal_id: int) -> float:
        """
        Get heuristic distance from current node to goal node
        Args:
            current_id (int): id of current node
            goal_id (int): id of goal node

        Returns:
            (float): heuristic distance
        """
        return self.graph.get_heuristic_distance(current_id, goal_id)

    def distance_between(self, n1_id: int, n2_id: int) -> float:
        """
        Get weight (distance) between two nodes
        Args:
            n1_id (int): id of node 1
            n2_id (int): id of node 2

        Returns:
            (float) distance between two nodes
        """
        return self.graph.get_weight(n1_id, n2_id)

    def neighbors(self, node_id: int) -> Iterable[int]:
        """
        Get list neighbors of node
        Args:
            node_id (int): id of node

        Returns:
            (Iterable) list id of neighbors
        """
        return self.graph.get_neighbors(node_id)

    @staticmethod
    def is_goal_reached(current_id: int, goal_id: int) -> bool:
        """
        Check whether reached goal yet
        Args:
            current_id (int): id of current node
            goal_id (int): id of goal node

        Returns:
            (bool)
        """
        return current_id == goal_id

    @staticmethod
    def reconstruct_path(last: NodeStatus, is_reverse_path=False) -> Iterable[int]:
        """
        Traverse back the trail from start to goal
        Args:
            last (NodeStatus): last node to trace back
            is_reverse_path (bool): is reverse path

        Returns:
            (Iterable) list of node id
        """

        def _gen():
            current = last
            while current:
                yield current.data
                current = current.came_from

        if is_reverse_path:
            return _gen()
        else:
            return reversed(list(_gen()))

    def find_path(self, start: int, goal: int, is_reverse_path: bool = False) -> Union[Iterable[int], None]:
        """
        Main function to find path from start to goal
        using controller algorithm
        Args:
            start (int): id of start node
            goal (int): id of goal node
            is_reverse_path (bool): whether to get reversed path

        Returns:
            List of id node from start to goal
            None if there is no feasible path
        """
        if self.is_goal_reached(start, goal):
            return [start]

        nodes_status = NodeStatusDict()
        start_node = nodes_status[start] = NodeStatus(
            start,
            g_score=0.0,
            f_score=self.heuristic_cost_estimate(start, goal)
        )

        open_set: list = []
        heappush(open_set, start_node)
        while open_set:
            current = heappop(open_set)
            if self.is_goal_reached(current.data, goal):
                return self.reconstruct_path(current, is_reverse_path)

            current.out_open_set = True
            current.closed = True
            for neighbor in map(lambda n: nodes_status[n], self.neighbors(current.data)):
                # ignore if traversed node
                if neighbor.closed:
                    continue
                tentative_gscore = current.g_score + self.distance_between(current.data, neighbor.data)
                # ignore if tentative score higher than current estimation
                if tentative_gscore >= neighbor.g_score:
                    continue
                # update node if have better score
                neighbor.came_from = current
                neighbor.g_score = tentative_gscore
                neighbor.f_score = tentative_gscore + self.heuristic_cost_estimate(neighbor.data, goal)
                if neighbor.out_open_set:
                    neighbor.out_open_set = False
                    heappush(open_set, neighbor)
                else:
                    # re-add the node in order to re-sort the heap
                    open_set.remove(neighbor)
                    heappush(open_set, neighbor)
        return None
