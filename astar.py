from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from heapq import heappush, heappop
from typing import Iterable, Union, List

import numpy as np
import pandas as pd

INFINITY = float("inf")


class NodeNotFound(KeyError):
    def __init__(self, node_id=None):
        self.message = f"Node {node_id} not found. " \
                       f"Please use function self.add_vertex(node) to add node into graph"

    def __str__(self):
        return self.message


class VerticesColumns:
    """
    Class to define columns of vertices.csv file
    """
    ID = "id"  # id of node
    X = "x"  # coordinate x of node
    Y = "y"  # coordinate y of node


class EdgesColumns:
    """
    Class to define columns of edges.csv file
    """
    U = "u"  # start node of edge
    V = "v"  # end node of edge
    WEIGHT = "weight"  # weight of edge


@dataclass(frozen=True)
class Position:
    """
    Position of node
    """
    x: float  # coordinate x of this position
    y: float  # coordinate y of this position

    def numpy(self):
        """
        Get position in numpy [x, y]
        Returns:

        """
        return np.array([self.x, self.y])


@dataclass(frozen=True)
class Node:
    """
    Node in graph
    """
    id: int  # index of node
    position: Position  # position of node

    def dist(self, other: Node):
        """
        Heuristic distance of this node to other
        using Euclidian distance
        Args:
            other (Node): other node

        Returns:
            (float) distance
        """

        return np.linalg.norm(self.position.numpy() - other.position.numpy())


class Graph:
    """
    Directional graph
    """

    def __init__(self):
        self.digraph = defaultdict(defaultdict)
        self.nodes = defaultdict()

    def load_data(self, vertices_path: str, edges_path: str):
        """
        Load data of graph from file
        Args:
            vertices_path (str): path to vertices data
            edges_path (str): path to edges data

        """
        vertices = pd.read_csv(vertices_path)
        edges = pd.read_csv(edges_path)

        # add vertices
        for _, row in vertices.iterrows():
            node = Node(
                id=row[VerticesColumns.ID],
                position=Position(x=row[VerticesColumns.X], y=row[VerticesColumns.Y])
            )
            self.add_vertex(node)

        # add edges
        for _, row in edges.iterrows():
            self.add_edge(
                u_id=row[EdgesColumns.U],
                v_id=row[EdgesColumns.V],
                weight=row[EdgesColumns.WEIGHT]
            )

    def add_vertex(self, node: Node):
        """
        Add a node as vertex into graph
        Args:
            node (Node): node to be added
        """
        self.nodes[node.id] = node

    def add_edge(self, u_id: int, v_id: int, weight: float):
        """
        Add an edge into graph
        Args:
            u_id (int): id of start node
            v_id (int): id of end node
            weight (float):
        """
        if u_id not in self.nodes:
            raise NodeNotFound(node_id=u_id)
        if v_id not in self.nodes:
            raise NodeNotFound(node_id=v_id)
        self.digraph[u_id][v_id] = weight

    def remove_node(self, node_id: int):
        """
        Remove node from graph
        Args:
            node_id (int): id of node
        """
        if node_id not in self.nodes:
            raise NodeNotFound(node_id=node_id)

        # delete in graph
        del self.digraph[node_id]
        for u, adj in self.digraph.items():
            for v, w in adj.items():
                if v == node_id:
                    del self.digraph[u][v]
                    break

        # delete in nodes container
        del self.nodes[node_id]

    def remove_edge(self, u_id: int, v_id: int):
        """
        Remove edge from graph
        Args:
            u_id (int): id of start node
            v_id (int): id of end node
        """
        del self.digraph[u_id][v_id]

    def get_heuristic_distance(self, u_id: int, v_id: int):
        """
        Get heuristic distance from start to end node
        Using euclidian distance
        Args:
            u_id (int): id of start node
            v_id (int): id of end node

        Returns:
            (float)
        """
        return self.nodes[u_id].dist(self.nodes[v_id])

    def get_weight(self, u_id: int, v_id: int):
        """
        Get weight/distance from start to end node
        Args:
            u_id (int): id of start node
            v_id (int): id of end node

        Returns:
            (float)
        """
        return self.digraph[u_id][v_id]

    def get_neighbors(self, node_id: int) -> List:
        """
        Get all neighbor of this node
        Args:
            node_id (int): node to find its neighbors

        Returns:
            (List) list of neighbors
        """
        return [v for v in self.digraph[node_id].keys()]

    def get_node_by_id(self, node_id) -> Node:
        """
        Get node by id
        Args:
            node_id (int): id of node

        Returns:
            (Node)
        """
        return self.nodes[node_id]


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
            b:

        Returns:

        """
        return self.f_score < b.f_score


class NodeStatusDict(dict):
    """
    Search node container
    """

    def __missing__(self, k):
        """
        Assign value if missing key
        Args:
            k:

        Returns:

        """
        v = NodeStatus(k)
        self.__setitem__(k, v)
        return v


class AStar:
    def __init__(self, graph: Graph):
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
        using astar algorithm
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
            print(current.data)
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


class Mediator:
    """
    Class to control paths of all robot
    """

    def __init__(self, lookahead: int = 5, bias_intersection: int = 1):
        """
        Args:
            lookahead (int): number of steps in local path
            bias_intersection (int): bias of steps while checking intersection
        """
        self.lookahead = lookahead
        self.bias_intersection = bias_intersection
        self.padding_value = -1

    def pad(self, path: List):
        to_pad = self.lookahead - len(path)
        return np.pad(path, (0, to_pad), "constant", constant_values=self.padding_value)

    def cut_lookahead(self, list_of_paths):
        # TODO: fasten this func
        container = []
        for path in list_of_paths:
            container.append(self.pad(path[:self.lookahead]))
        return np.array(container)

    def cut_if_meet_other_robot(self, local_paths: np.ndarray):
        starting_points = local_paths[:, 0]
        forwarding_points = local_paths[:, 1:]
        mask = np.isin(forwarding_points, starting_points)
        accu = np.cumsum(mask, axis=1)
        # only get path that does not collision with other's starting point
        forwarding_points[accu != 0] = self.padding_value
        return local_paths

    def get_local_paths(self, list_of_paths):
        local_paths = self.cut_lookahead(list_of_paths)
        local_paths = self.cut_if_meet_other_robot(local_paths)
        # TODO: cut if intersection (shift)
        return local_paths


if __name__ == "__main__":
    graph = Graph()
    graph.load_data(vertices_path="vertices.csv", edges_path="edges.csv")

    astar = AStar(graph=graph)

    # this will be global path
    path1 = [i for i in astar.find_path(1, 9)]
    print(path1)
    path2 = [i for i in astar.find_path(6, 10)]
    print(path2)
