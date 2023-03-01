from src.controller import AStar, Mediator
from src.graph import Graph
from src.models.robot import Robot

if __name__ == "__main__":
    # Graph:
    # 1 -> 2 -> 3 -> 4
    # ^    |         v
    # 10   |         5
    # ^    v         v
    # 9 <- 8 <- 7 <- 6

    # TODO: unblock mechanism when deadlock

    graph = Graph()
    graph.load_data(vertices_path="data/vertices.csv", edges_path="data/edges.csv")

    astar = AStar(graph=graph)

    mediator = Mediator()
    mediator.add_robot(Robot(id_robot=1, global_path=[i for i in astar.find_path(1, 10)]))
    mediator.add_robot(Robot(id_robot=2, global_path=[i for i in astar.find_path(6, 9)]))
    mediator.add_robot(Robot(id_robot=5, global_path=[i for i in astar.find_path(7, 9)]))

    mediator.add_robot(Robot(id_robot=10, global_path=[]))
    mediator.update_global_path(id_robot=10, global_path=[i for i in astar.find_path(3, 2)])

    local_paths = mediator.find_local_paths()

    print("Global paths:")
    print({id_robot: robot.get_global_path() for id_robot, robot in mediator.get_robot_container().items()})
    print("Local paths:")
    print(local_paths)
