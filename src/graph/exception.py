class NodeNotFound(KeyError):
    """
    Node not found exception
    """

    def __init__(self, node_id: int = None):
        """
        Init function
        Args:
            node_id (int): node index
        """
        self.message = f"Node {node_id} not found. " \
                       f"Please use function self.add_vertex(node) to add node into graph"

    def __str__(self) -> str:
        """
        Parse exception message
        Returns:
            (str) message
        """
        return self.message
