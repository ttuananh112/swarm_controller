from typing import List

from src.common.constants import INFINITY
from src.models import State


class Robot:
    """
    Robot class
    """

    def __init__(self, id_robot: int, state: State, global_path: List[int] = ()):
        """
        Init function
        Args:
            id_robot (int): id of robot
            state (State): state of robot
            global_path (List[int]): global path
        """
        self.id_robot = id_robot
        self.state = state

        # discrete position
        self.current_position = -1
        self.local_path = []
        self.global_path = []
        self.current_to_goal = []
        self.set_global_path(global_path)

    def set_state(self, state: State):
        """
        Set state of robot
        Args:
            state:

        Returns:

        """
        self.state = state

    def update_state(self, up_coming_heading: float, dt: float):
        """
        Update state of robot
        Args:
            up_coming_heading:
            dt:

        Returns:

        """
        self.state.update(up_coming_heading=up_coming_heading, dt=dt)

    def is_at_goal(self):
        """
        Check whether robot reaching goal
        Returns:

        """
        return len(self.current_to_goal) == 1

    def set_global_path(self, global_path: List[int]):
        """
        Set new global path
        Reinit params
        Args:
            global_path (List[int]): new global path
        """
        self.global_path = global_path
        self.current_position = self.global_path[0] if len(self.global_path) > 0 else INFINITY
        self.local_path = []
        self.current_to_goal = global_path

    def get_global_path(self) -> List[int]:
        """
        Get global path
        Returns:
            (List[int])
        """
        return self.global_path

    def set_current_position(self, current_position: int):
        """
        Set new current position of robot
        Args:
            current_position (int): position as node id
        """
        self.current_position = current_position
        self.update_current_to_goal()

    def get_current_position(self) -> int:
        """
        Get current position
        Returns:
            (int)
        """
        return self.current_position

    def set_local_path(self, local_path: List[int]):
        """
        Set new local path
        Args:
            local_path (List[int]): new local path
        """
        self.local_path = local_path

    def get_local_path(self) -> List[int]:
        """
        Get local path
        Returns:
            (List[int])
        """
        return self.local_path

    def update_current_to_goal(self):
        """
        Update current_to_goal list
        """
        i = 0
        for i, pos in enumerate(self.current_to_goal):
            if pos == self.current_position:
                break
        self.current_to_goal = self.current_to_goal[i:]

    def get_current_to_goal(self) -> List[int]:
        """
        Get current_to_goal list
        """
        return self.current_to_goal

    def __repr__(self):
        """
        Represent for debugging
        """
        return self.__str__()

    def __str__(self):
        """
        To string
        """
        return f"Robot(" \
               f"id_robot={self.id_robot}, " \
               f"current_position={self.current_position}, " \
               f"local_path={self.local_path}, " \
               f"global_path={self.global_path}, " \
               f"current_to_goal={self.current_to_goal})"
