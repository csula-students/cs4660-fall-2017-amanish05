"""
utils package is for some quick utility methods

such as parsing
"""

from .graph import Node
from .graph import Edge


class Tile(object):
    """Node represents basic unit of graph"""
    def __init__(self, x, y, symbol):
        self.x = x
        self.y = y
        self.symbol = symbol

    def __str__(self):
        return 'Tile(x: {}, y: {}, symbol: {})'.format(self.x, self.y, self.symbol)
    def __repr__(self):
        return 'Tile(x: {}, y: {}, symbol: {})'.format(self.x, self.y, self.symbol)

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.x == other.x and self.y == other.y and self.symbol == other.symbol
        return False
    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        return self.y < other.y or self.x < other.x

    def __hash__(self):
        return hash(str(self.x) + "," + str(self.y) + self.symbol)



def parse_grid_file(graph, file_path):
    """
    ParseGridFile parses the grid file implementation from the file path line
    by line and construct the nodes & edges to be added to graph
    Returns graph object
    """
    # TODO: read the filepath line by line to construct nodes & edges
    # TODO: for each node/edge above, add it to graph

    grid = []
    with open(file_path) as file:
        next(file)
        for line in file:
            line.strip().split("\n")
            grid.append([line[i:i + 2] for i in range(1, len(line[1:-1]), 2)])
        grid = grid[:-1]

    tiles_node = {}
    for x in range(len(grid)):
        for y in range(len(grid[0])):
            tiles_node[(y, x)] = Tile(y, x, grid[x][y])

    for i in range(len(grid)):
        for j in range(len(grid[0])):
            current_tile = Tile(j, i, grid[i][j])

            if current_tile.symbol == "##":
                continue

            if (j, i + 1) in tiles_node:
                lower_tile = tiles_node[(j, i + 1)]
                if lower_tile.symbol != "##":
                    graph.add_edge(Edge(Node(current_tile), Node(lower_tile), 1))

            if (j, i - 1) in tiles_node:
                upper_tile = tiles_node[(j, i - 1)]
                if upper_tile.symbol != "##":
                    graph.add_edge(Edge(Node(current_tile), Node(upper_tile), 1))

            if (j + 1, i) in tiles_node:
                right_tile = tiles_node[(j + 1, i)]
                if right_tile.symbol != "##":
                    graph.add_edge(Edge(Node(current_tile), Node(right_tile), 1))

            if (j - 1, i) in tiles_node:
                left_tile = tiles_node[(j - 1, i)]
                if left_tile.symbol != "##":
                    graph.add_edge(Edge(Node(current_tile), Node(left_tile), 1))

    return graph

def convert_edge_to_grid_actions(edges):
    """
    Convert a list of edges to a string of actions in the grid base tile

    e.g. Edge(Node(Tile(1, 2), Tile(2, 2), 1)) => "S"
    """

    path = ""
    directions = {
        (0, 0): "",
        (1, 0): "W",
        (0, 1): "N",
        (-1, 0): "E",
        (0, -1): "S"
    }
    if edges:
        for edge in edges:
            tile1 = edge.from_node.data
            tile2 = edge.to_node.data
            path = path + directions[(tile1.x - tile2.x, tile1.y - tile2.y)]
    #print("Path is: " + path)
    return path