from src.controller import AStar, Mediator
from src.graph import Graph

if __name__ == "__main__":
    # TODO:
    #  1. handle slow response from robot (shift local paths)
    #  2. unblock mechanism when deadlock

    graph = Graph()
    graph.load_data(vertices_path="data/vertices.csv", edges_path="data/edges.csv")

    astar = AStar(graph=graph)

    global_paths = [
        [i for i in astar.find_path(1, 9)],
        [i for i in astar.find_path(6, 10)]
    ]

    print("Global paths:")
    print("\n".join([str(path) for path in global_paths]))
    mediator = Mediator(global_paths=global_paths)
    local_paths = mediator.get_local_paths()
    print("Local paths:\n", local_paths)
