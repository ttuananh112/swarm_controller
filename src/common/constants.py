class VerticesColumns:
    """
    Class to define columns of vertices.csv file
    """
    ID = "id"  # id of node
    X = "x"  # coordinate x of node
    Y = "y"  # coordinate y of node


class EdgesColumns:
    """
    Class to define columns of edges.csv file
    """
    U = "u"  # start node of edge
    V = "v"  # end node of edge
    WEIGHT = "weight"  # weight of edge


INFINITY = float("inf")
