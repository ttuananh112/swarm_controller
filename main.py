from src.controller import AStar, Mediator
from src.graph import Graph

if __name__ == "__main__":
    # Graph:
    # 1 -> 2 -> 3 -> 4
    # ^    |         v
    # 10   |         5
    # ^    v         v
    # 9 <- 8 <- 7 <- 6

    # TODO:
    #  1. handle slow response from robot (shift local paths) (DOING, got bug
    #  Expected output: [1, 2, inf, inf, inf]
    #  Got output: [1, inf, inf, inf, inf]
    #  2. unblock mechanism when deadlock

    graph = Graph()
    graph.load_data(vertices_path="data/vertices.csv", edges_path="data/edges.csv")

    astar = AStar(graph=graph)

    global_paths = [
        [i for i in astar.find_path(1, 10)],
        [i for i in astar.find_path(7, 9)]
    ]
    mediator = Mediator(global_paths=global_paths)
    local_paths = mediator.get_local_paths()

    print("Global paths:")
    print("\n".join([str(path) for path in global_paths]))
    print("Local paths:\n", local_paths)
