from src.controller import AStar, Mediator
from src.graph import Graph
from src.models.robot import Robot

if __name__ == "__main__":
    # TODO: unblock mechanism when deadlock

    graph = Graph()
    graph.load_data(vertices_path="data/vertices.csv", edges_path="data/edges.csv")

    astar = AStar(graph=graph)

    mediator = Mediator(lookahead=10, bias_intersection=1)
    mediator.add_robot(Robot(id_robot=1, global_path=[i for i in astar.find_path(13, 156)]))
    mediator.add_robot(Robot(id_robot=2, global_path=[i for i in astar.find_path(76, 13)]))
    mediator.add_robot(Robot(id_robot=3, global_path=[i for i in astar.find_path(41, 108)]))

    mediator.add_robot(Robot(id_robot=4, global_path=[]))
    mediator.update_global_path(id_robot=4, global_path=[i for i in astar.find_path(129, 1)])

    running = True
    while running:
        local_paths = mediator.find_local_paths()
        print("Global paths:")
        print({id_robot: robot.get_global_path() for id_robot, robot in mediator.get_robot_container().items()})
        print("Local paths:")
        print(local_paths)
        print("_" * 10)
        # update robot position

        running = False
        for id_robot in mediator.get_robot_container().keys():
            if mediator.get_robot_container()[id_robot].is_at_goal():
                continue
            else:
                next_position = local_paths[id_robot][1] if len(local_paths[id_robot]) > 1 else local_paths[id_robot][0]
                mediator.get_robot_container()[id_robot].set_current_position(next_position)
                running = True
