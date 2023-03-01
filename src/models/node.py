from __future__ import annotations

from dataclasses import dataclass

import numpy as np


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
