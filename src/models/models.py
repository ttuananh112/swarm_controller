from __future__ import annotations

from dataclasses import dataclass

import numpy as np

RADIAN_EPS = 1e-1  # [rad]
DISTANCE_EPS = 0.5  # [m]


@dataclass(frozen=False)
class Vector:
    """
    Position of node
    """
    x: float = 0.  # coordinate x of this position
    y: float = 0.  # coordinate y of this position

    def numpy(self) -> np.ndarray:
        """
        Get position in numpy [x, y]
        Returns:

        """
        return np.array([self.x, self.y])

    @staticmethod
    def project(magnitude: float, angle: float) -> Vector:
        """
        Project to x and y
        Args:
            magnitude (float): magnitude
            angle (float): angle to project, in rad

        Returns:
            Vector
        """
        x = magnitude * np.cos(angle)
        y = magnitude * np.sin(angle)
        return Vector(x=x, y=y)

    @staticmethod
    def from_numpy(arr: np.ndarray) -> Vector:
        """
        Cast to Vector from numpy
        Args:
            arr:

        Returns:

        """
        arr = arr.squeeze()
        if len(arr) != 2:
            raise ValueError("Array should have x, y dim only")
        x, y = arr
        return Vector(x=x, y=y)

    def magnitude(self) -> float:
        """
        Magnitude of bector
        Returns:

        """
        return np.linalg.norm(self.numpy())

    def angle_between(self, other: Vector):
        """
        Estimate the angle between 2 vectors
        Args:
            other:

        Returns:

        """
        a = np.dot(self.numpy(), other.numpy())
        b = self.magnitude() * other.magnitude()
        return np.arccos(a / b)

    def __sub__(self, other):
        return Vector.from_numpy(self.numpy() - other.numpy())

    def __add__(self, other):
        return Vector.from_numpy(self.numpy() + other.numpy())

    def __mul__(self, other):
        return Vector.from_numpy(self.numpy() * other.numpy())

    def __eq__(self, other):
        return (self - other).magnitude() <= DISTANCE_EPS


@dataclass(frozen=True)
class Node:
    """
    Node in graph
    """
    id: int  # index of node
    position: Vector  # position of node

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


@dataclass
class State:
    """
    State of a robot
    """
    position: Vector = Vector()
    velocity: float = 0.  # magnitude of velocity
    acceleration: float = 0.  # magnitude of acceleration

    heading: float = 0.  # [-pi, pi]
    omega: float = 0.  # rotation velocity

    def step_forward(self, dt: float):
        """
        Go ahead
        Args:
            dt:

        Returns:

        """
        vel = Vector.project(magnitude=self.velocity, angle=self.heading)
        acc = Vector.project(magnitude=self.acceleration, angle=self.heading)

        # heuristic
        s = np.array([
            [self.position.x, vel.x, acc.x],
            [self.position.y, vel.y, acc.y],
            [vel.x, acc.x, 0],
            [vel.y, acc.y, 0]
        ])
        t = np.array([1, dt, dt ** 2])
        res = np.dot(s, t)

        # update state
        self.position = Vector.from_numpy(res[:2])
        self.velocity = Vector.from_numpy(res[2:]).magnitude()

    @staticmethod
    def standardize_heading(heading):
        """
        Standardizes an input heading angle in radians to the range of -π to π.
        Args:
            heading (float): a float representing an input heading angle in radians

        Returns:
            a float representing the standardized heading angle in radians
            """
        res = (heading + np.pi) % (2 * np.pi) - np.pi
        return res

    @staticmethod
    def rotation_direction(start_angle, stop_angle):
        """
        Get the rotation direction from start_angle to stop_angle
        Args:
            start_angle:
            stop_angle:

        Returns:

        """
        return -np.copysign(1, (stop_angle - start_angle) % (2 * np.pi) - np.pi)

    def rotate(self, rotation_direction: float, dt: float):
        """
        Rotate
        Args:
            rotation_direction: float
            dt:

        Returns:

        """
        self.heading = self.standardize_heading(self.heading + rotation_direction * self.omega * dt)

    def update(self, up_coming_heading: float, dt: float):
        """
        Main function to update status
        Returns:

        """
        if abs(self.heading - up_coming_heading) <= (np.pi / 8 - RADIAN_EPS):
            # step ahead if got the same heading as up_coming_heading
            self.heading = up_coming_heading
            self.step_forward(dt)
        else:
            # change rotation direction of omega
            self.rotate(self.rotation_direction(self.heading, up_coming_heading), dt)


if __name__ == "__main__":
    state = State(position=Vector(), velocity=1., omega=np.pi / 8)
    up_coming_headings = [0, 0, 0, 0, 0,
                          np.pi / 2, np.pi / 2, np.pi / 2, np.pi / 2, np.pi / 2, np.pi / 2, np.pi / 2,
                          np.pi, np.pi, np.pi, np.pi, np.pi, np.pi,
                          -np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2,
                          0, 0, 0, 0, 0, 0]
    for up_coming in up_coming_headings:
        state.update(up_coming_heading=up_coming, dt=1)
        print(state)
