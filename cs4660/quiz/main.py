"""
quiz2!

Use path finding algorithm to find your way through dark dungeon!

Tecchnical detail wise, you will need to find path from node 7f3dc077574c013d98b2de8f735058b4
to f1f131f647621a4be7c71292e79613f9

TODO: implement BFS
TODO: implement Dijkstra utilizing the path with highest effect number
"""
from queue import *
import math
import heapq
import json


# http lib import for Python 2 and 3: alternative 4
try:
    from urllib.request import urlopen, Request
except ImportError:
    from urllib2 import urlopen, Request

GET_STATE_URL = "http://192.241.218.106:9000/getState"
STATE_TRANSITION_URL = "http://192.241.218.106:9000/state"

def get_state(room_id):
    """
    get the room by its id and its neighbor
    """
    body = {'id': room_id}
    return __json_request(GET_STATE_URL, body)

def transition_state(room_id, next_room_id):
    """
    transition from one room to another to see event detail from one room to
    the other.

    You will be able to get the weight of edge between two rooms using this method
    """
    body = {'id': room_id, 'action': next_room_id}
    return __json_request(STATE_TRANSITION_URL, body)

def __json_request(target_url, body):
    """
    private helper method to send JSON request and parse response JSON
    """
    req = Request(target_url)
    req.add_header('Content-Type', 'application/json; charset=utf-8')
    jsondata = json.dumps(body)
    jsondataasbytes = jsondata.encode('utf-8')   # needs to be bytes
    req.add_header('Content-Length', len(jsondataasbytes))
    response = json.load(urlopen(req, jsondataasbytes))
    return response

def construct_graph_from_json(list):
    for node in list:
        mygraph['id'] = getEdge(node)
    return mygraph

def getEdge(line):
    all_edge = []
    print(line['neighbors'])
    ngh = line['neighbors']
    for n in ngh:
        weight = transition_state(line['id'], n['id'])
        edge = Edge(Node(line['id']), Node(n['id']), weight)
        all_edge.append(edge)
    return all_edge


if __name__ == "__main__":
    # Your code starts here
    empty_room = get_state('7f3dc077574c013d98b2de8f735058b4')
    graph = construct_graph_from_json(empty_room)
    print(graph)
    #print(transition_state(empty_room['id'], empty_room['neighbors'][0]['id']))


def bfs(graph, initial_node, dest_node):
    """
    Breadth First Search
    uses graph to do search from the initial_node to dest_node
    returns a list of actions going from the initial node to dest_node
    """

    _visiting_queue = Queue(maxsize=0)
    _visiting_queue.put(initial_node)

    _trace_nodes, _visited_list, result = {}, set(), []

    while _visiting_queue:
        current_node = _visiting_queue.get()
        _visited_list.add(current_node)

        if(current_node == dest_node):
            break;

        for neighbors_node in (graph.neighbors(current_node)):
            if neighbors_node not in _trace_nodes:
                _trace_nodes[neighbors_node] = current_node

            if neighbors_node not in _visited_list:
                _visiting_queue.put(neighbors_node)

    if dest_node in _trace_nodes:

        parent = _trace_nodes[dest_node]
        while parent is not None:
            result.insert(0, g.Edge(parent, dest_node, graph.distance(parent, dest_node)))
            dest_node = parent
            if dest_node in _trace_nodes:
                parent = _trace_nodes[dest_node]
            else:
                parent = None
    return result

def dijkstra_search(graph, initial_node, dest_node):
    """
    Dijkstra Search
    uses graph to do search from the initial_node to dest_node
    returns a list of actions going from the initial node to dest_node
    """

    queue, _trace_nodes, cost, result = [], {}, {}, []
    heapq.heappush(queue, PriorityNode(0, initial_node))
    cost[initial_node] = 0

    while queue:
        current_node = heapq.heappop(queue).node
        #print(current_node)

        if current_node == dest_node:
            break

        for neighbors_node in graph.neighbors(current_node):
            #print("Current node cost is: " +str(cost[current_node]))
            #print("Distance cost is: " + str(graph.distance(current_node, neighbors_node)))
            total_cost = cost[current_node] + graph.distance(current_node, neighbors_node)
            #print("Current distance is: " + str(total_cost))

            if neighbors_node not in cost or total_cost < cost[neighbors_node]:
                cost[neighbors_node] = total_cost
                heapq.heappush(queue, PriorityNode(total_cost, neighbors_node))
                _trace_nodes[neighbors_node] = current_node

    if (dest_node in _trace_nodes):
        parent = _trace_nodes[dest_node]
        while parent is not None:
            result.insert(0, g.Edge(parent, dest_node, graph.distance(parent, dest_node)))
            dest_node = parent
            if dest_node in _trace_nodes:
                parent = _trace_nodes[dest_node]
            else:
                parent = None

        #print(result)
    return result

class PriorityNode(object):
    """Node represents basic unit of graph"""

    def __init__(self, data, node):
        self.priority = data
        self.node  = node

    def __str__(self):
        return "Priority-" + str(self.priority) + " " + self.node.__str__()

    def __eq__(self, other_node):
        return self.node == other_node.node

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash(self.node)

    def __cmp__(self, other):
        if self.priority > other.priority:
            return -1
        elif self.priority == other.priority:
            return 0
        else:
            return 1

    def __lt__(self, other):
        return self.priority > other.priority

    def __gt__(self, other):
        return self.priority < other.priority


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
