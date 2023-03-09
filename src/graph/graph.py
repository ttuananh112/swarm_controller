from __future__ import annotations

from collections import defaultdict
from typing import List

import pandas as pd

from src.common.constants import VerticesColumns, EdgesColumns
from src.graph.exception import NodeNotFound
from src.models import Node, Vector

INFINITY = float("inf")


class Graph:
    """
    Directional graph
    """

    def __init__(self):
        """
        Init function
        """
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
                position=Vector(x=row[VerticesColumns.X], y=row[VerticesColumns.Y])
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
