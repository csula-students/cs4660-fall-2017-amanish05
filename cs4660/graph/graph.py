"""
graph module defines the knowledge representations files

A Graph has following methods:

* adjacent(node_1, node_2)
    - returns true if node_1 and node_2 are directly connected or false otherwise
* neighbors(node)
    - returns all nodes that is adjacency from node
* add_node(node)
    - adds a new node to its internal data structure.
    - returns true if the node is added and false if the node already exists
* remove_node
    - remove a node from its internal data structure
    - returns true if the node is removed and false if the node does not exist
* add_edge
    - adds a new edge to its internal data structure
    - returns true if the edge is added and false if the edge already existed
* remove_edge
    - remove an edge from its internal data structure
    - returns true if the edge is removed and false if the edge does not exist
"""

from io import open
import math
from operator import itemgetter

def construct_graph_from_file(graph, file_path):
    with open(file_path) as file:
        next(file)
        for line in file:
            graph.add_edge(getEdge(line))
    return graph

def getEdge(line):
    value = line.strip().split(":")
    return Edge(Node(int(value[0])), Node(int(value[1])), int(value[2]))

class Node(object):
    """Node represents basic unit of graph"""
    def __init__(self, data):
        self.data = data

    def __str__(self):
        return 'Node({})'.format(self.data)

    def __repr__(self):
        return 'Node({})'.format(self.data)

    def __eq__(self, other_node):
        return self.data == other_node.data

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash(self.data)

class Edge(object):
    """Edge represents basic unit of graph connecting between two edges"""
    def __init__(self, from_node, to_node, weight):
        self.from_node = from_node
        self.to_node = to_node
        self.weight = weight

    def __str__(self):
        return 'Edge(from {}, to {}, weight {})'.format(self.from_node, self.to_node, self.weight)

    def __repr__(self):
        return 'Edge(from {}, to {}, weight {})'.format(self.from_node, self.to_node, self.weight)

    def __eq__(self, other_node):
        return self.from_node == other_node.from_node and self.to_node == other_node.to_node and self.weight == other_node.weight

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self.from_node, self.to_node, self.weight))


class AdjacencyList(object):
    """
    AdjacencyList is one of the graph representation which uses adjacency list to
    store nodes and edges
    """
    def __init__(self):
        # adjacencyList should be a dictonary of node to edges
        self.adjacency_list = {}

    def adjacent(self, node_1, node_2):
        if node_1 in self.adjacency_list:
            allEdges = []
            for edge in self.adjacency_list[node_1]:
                allEdges.append(edge.to_node)
            return node_2 in allEdges
        return False

    def neighbors(self, node):
        neighbors = []
        if node in self.adjacency_list:
            for edge in self.adjacency_list[node]:
                neighbors.append(edge.to_node)
        return neighbors

    def add_node(self, node):
        if node not in self.adjacency_list:
            self.adjacency_list[node] = []
            return True
        return False

    def remove_node(self, node):
        for nodes in self.adjacency_list:
            for edge in self.adjacency_list[nodes]:
                if edge.to_node.__eq__(node):
                    self.remove_edge(edge)

        if node in self.adjacency_list:
            del self.adjacency_list[node]
            return True

        return False

    def add_edge(self, edge):
        if edge.from_node not in self.adjacency_list:
            self.add_node(edge.from_node)

        if edge not in self.adjacency_list[edge.from_node]:
            self.adjacency_list[edge.from_node].append(edge)
            return True

        return False

    def remove_edge(self, edge):
        if edge.from_node in self.adjacency_list:
            if edge in self.adjacency_list[edge.from_node]:
                self.adjacency_list[edge.from_node].remove(edge)
                return  True
        return False


    def distance(self, node_1, node_2):
        if node_1 in self.adjacency_list:
            edges = self.adjacency_list[node_1]
            for edge in edges:
                if(edge.to_node == node_2):
                    return edge.weight
        return math.inf

    def __str__(self):
        return '(List {})'.format(self.adjacency_list)

class AdjacencyMatrix(object):
    def __init__(self):
        # adjacency_matrix should be a two dimensions array of numbers that
        # represents how one node connects to another
        self.adjacency_matrix = []
        # in additional to the matrix, you will also need to store a list of Nodes
        # as separate list of nodes
        self.nodes = []

    def __str__(self):
        return '(Matrix {})'.format(self.adjacency_matrix)

    def adjacent(self, node_1, node_2):
        if node_1 not in self.nodes or node_2 not in self.nodes:
            return False

        index_1 = self.__get_node_index(node_1)
        index_2 = self.__get_node_index(node_2)
        
        if self.adjacency_matrix[index_1][index_2] > 0:
            return True
        else:
            return False


    def neighbors(self, node):
        #print(self.adjacency_matrix)
        neighbors = []
        if node in self.nodes:
            index_1 = self.__get_node_index(node)
            for raw in range(0, len(self.adjacency_matrix[index_1])):
                #print("Node index is : " + index_1.__str__() + " And node is: " +node.__str__())
                #print(self.adjacency_matrix[index_1])
                if self.adjacency_matrix[index_1][raw] > 0:
                    neighbors.append(self.nodes[raw])
                    #print(self.nodes[raw])
        #return neighbors.sort(key=lambda x: x.data)
        return sorted(neighbors, key=lambda x: x.data)


    def add_node(self, node):

        if node in self.nodes:
            return False

        #print("Adding node: " + node.__str__())
        self.nodes.append(node)
        self.adjacency_matrix.extend([[0] * len(self.nodes)])

        for eachRow in self.adjacency_matrix:
            eachRow.extend([0])
        return True

    def remove_node(self, node):
        if node in self.nodes:
            index_1 = self.__get_node_index(node)
            self.nodes.remove(node)
            for col in self.adjacency_matrix:
                del col[index_1]
                #col.pop(index_1)
            del self.adjacency_matrix[index_1]
            return True

        return False

    def add_edge(self, edge):
        if edge.from_node not in self.nodes:
            self.add_node(edge.from_node)

        if edge.to_node not in self.nodes:
            self.add_node(edge.to_node)

        index_1 = self.__get_node_index(edge.from_node)
        index_2 = self.__get_node_index(edge.to_node)

        if self.adjacency_matrix[index_1][index_2] == 0:
            self.adjacency_matrix[index_1][index_2] = edge.weight
            return True
        return False

    def remove_edge(self, edge):
        if edge.from_node not in self.nodes and edge.to_node not in self.nodes:
            return False

        index_1 = self.__get_node_index(edge.from_node)
        index_2 = self.__get_node_index(edge.to_node)

        if self.adjacency_matrix[index_1][index_2] > 0:
            self.adjacency_matrix[index_1][index_2] = 0
            return True

        return False


    def distance(self, node_1, node_2):
        if node_1 in self.nodes:
            return self.adjacency_matrix[self.__get_node_index(node_1)][self.__get_node_index(node_2)]
        return math.inf

    def __get_node_index(self, node):
        return self.nodes.index(node)

class ObjectOriented(object):
    """ObjectOriented defines the edges and nodes as both list"""
    def __init__(self):
        # implement your own list of edges and nodes
        self.edges = []
        self.nodes = []

    def __str__(self):
        return '(ObjectOriented {})'.format(self.nodes)

    def adjacent(self, node_1, node_2):
        if node_1 in self.nodes:
            for edge in self.edges:
                if edge.from_node == node_1 and edge.to_node == node_2:
                    return True
        return False

    def neighbors(self, node):
        neighbors = []
        for edge in self.edges:
            if node.__eq__(edge.from_node):
                if edge.from_node not in neighbors:
                    neighbors.append(edge.to_node)
        return neighbors

    def add_node(self, node):
        if node not in self.nodes:
            self.nodes.append(node)
            return True
        return False

    def remove_node(self, node):
        if node in self.nodes:
            self.nodes.remove(node)
            for edge in self.edges:
                if node.__eq__(edge.to_node) or node.__eq__(edge.from_node):
                    self.remove_edge(edge)
            return True
        return False

    def add_edge(self, edge):
        if edge.from_node not in self.nodes:
            self.add_node(edge.from_node)

        if edge not in self.edges:
            self.edges.append(edge)
            return True
        return False

    def remove_edge(self, edge):
        if edge in self.edges:
            self.edges.remove(edge)
            return True
        return False

    def distance(self, node_1, node_2):
        if node_1 in self.nodes:
            for edge in self.edges:
                if(edge.from_node == node_1 and edge.to_node == node_2):
                    return edge.weight
        return math.inf
