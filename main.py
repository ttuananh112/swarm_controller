from time import sleep

import numpy as np

from src.controller import AStar, Mediator
from src.graph import Graph
from src.models import State
from src.models.robot import Robot

DT = 0.1

if __name__ == "__main__":
    # TODO: unblock mechanism when deadlock

    graph = Graph()
    graph.load_data(vertices_path="data/vertices.csv", edges_path="data/edges.csv")

    astar = AStar(graph=graph)

    mediator = Mediator()
    mediator.add_robot(
        Robot(id_robot=1,
              state=State(position=graph.get_node_by_id(13).position,
                          velocity=0.2,
                          heading=-np.pi / 2,
                          omega=np.pi / 8),
              global_path=[i for i in astar.find_path(13, 156)])
    )
    mediator.add_robot(
        Robot(id_robot=2,
              state=State(position=graph.get_node_by_id(76).position,
                          velocity=0.2,
                          heading=0,
                          omega=np.pi / 8),
              global_path=[i for i in astar.find_path(76, 13)])
    )
    mediator.add_robot(
        Robot(id_robot=3,
              state=State(position=graph.get_node_by_id(41).position,
                          velocity=0.3,
                          heading=-np.pi / 2,
                          omega=np.pi / 8),
              global_path=[i for i in astar.find_path(41, 108)])
    )
    mediator.add_robot(
        Robot(id_robot=4,
              state=State(position=graph.get_node_by_id(129).position,
                          velocity=0.15,
                          heading=0,
                          omega=np.pi / 8),
              global_path=[i for i in astar.find_path(129, 1)])
    )

    while True:
        mediator.update(graph=graph, dt=DT)
        for id_robot, robot in mediator.get_robot_container().items():
            print(robot)
        print("_" * 10)
        sleep(DT)
