from typing import List

from src.common.constants import INFINITY


class Robot:
    def __init__(self, id_robot: int, global_path: List[int]):
        self.id_robot = id_robot
        self.current_position = -1
        self.local_path = []
        self.global_path = []
        self.current_to_goal = []
        self.set_global_path(global_path)

    def set_global_path(self, global_path):
        self.global_path = global_path
        self.current_position = self.global_path[0] if len(self.global_path) > 0 else INFINITY
        self.local_path = []
        self.current_to_goal = global_path

    def get_global_path(self):
        return self.global_path

    def set_current_position(self, current_position):
        self.current_position = current_position
        self.update_current_to_goal()

    def get_current_position(self):
        return self.current_position

    def set_local_path(self, local_path: List[int]):
        self.local_path = local_path

    def get_local_path(self):
        return self.local_path

    def update_current_to_goal(self):
        i = 0
        for i, pos in enumerate(self.current_to_goal):
            if pos == self.current_position:
                break
        self.current_to_goal = self.current_to_goal[i:]

    def get_current_to_goal(self):
        return self.current_to_goal

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return f"Robot(" \
               f"id_robot={self.id_robot}, " \
               f"current_position={self.current_position}, " \
               f"local_path={self.local_path}, " \
               f"global_path={self.global_path}, " \
               f"current_to_goal={self.current_to_goal})"
