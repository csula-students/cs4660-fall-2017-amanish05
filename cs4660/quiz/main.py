"""
quiz2!

Use path finding algorithm to find your way through dark dungeon!

Tecchnical detail wise, you will need to find path from node 7f3dc077574c013d98b2de8f735058b4
to f1f131f647621a4be7c71292e79613f9

TODO: implement BFS
TODO: implement Dijkstra utilizing the path with highest effect number
"""

import json
import heapq
from queue import *
from  graph import graph as g
from search import searches as s


# http lib import for Python 2 and 3: alternative 4
try:
    from urllib.request import urlopen, Request
except ImportError:
    from urllib2 import urlopen, Request

GET_STATE_URL = "http://192.241.218.106:9000/getState"
STATE_TRANSITION_URL = "http://192.241.218.106:9000/state"

def dijkstra_search(graph, initial_node, dest_node):
    """
    Dijkstra Search
    uses graph to do search from the initial_node to dest_node
    returns a list of actions going from the initial node to dest_node
    """
    queue, _trace_nodes, cost, result = [], {}, {}, []
    heapq.heappush(queue, PriorityNode(0, initial_node))
    cost[initial_node] = 0
    visited_set = set()

    while queue:
        current_node = heapq.heappop(queue).node
        visited_set.add(current_node)

        if current_node == dest_node:
            break

        for neighbors_node in graph.neighbors(current_node):

            if neighbors_node not in visited_set:

                total_cost = cost[current_node] + graph.distance(current_node, neighbors_node)
                if neighbors_node not in cost or total_cost > cost[neighbors_node]:
                    cost[neighbors_node] = total_cost
                    heapq.heappush(queue, PriorityNode(-(total_cost), neighbors_node))
                    _trace_nodes[neighbors_node] = current_node

    if (dest_node in _trace_nodes):
        parent = _trace_nodes[dest_node]
        while parent is not None:
            #print("Parent of " + str(parent) + " is" + str(dest_node))
            result.insert(0, g.Edge(parent, dest_node, graph.distance(parent, dest_node)))
            dest_node = parent
            if dest_node in _trace_nodes:
                parent = _trace_nodes[dest_node]
            else:
                parent = None
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
        if self.priority < other.priority:
            return -1
        elif self.priority == other.priority:
            return 0
        else:
            return 1

    def __lt__(self, other):
        return self.priority < other.priority

    def __gt__(self, other):
        return self.priority > other.priority

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

def construct_graph_from_json(my_graph, list):

    neighbors = list['neighbors']

    added_queue = set()
    added_queue.add(list['id'])

    queue = Queue(maxsize=0)
    queue.put(list['neighbors'][0]['id'])

    parent = list['id']
    print("Constructing graph... ")
    while queue.qsize() > 0:
        for nnode in neighbors:
            added_queue.add(nnode['id'])
            weight = transition_state(parent, nnode['id'])
            if (nnode['id'] != '7f3dc077574c013d98b2de8f735058b4'):
                my_graph.add_edge(g.Edge(g.Node(parent),  g.Node(nnode['id']), weight['event']['effect']))
            if 'f1f131f647621a4be7c71292e79613f9' not in added_queue:
                queue.put(nnode['id'])

        print("Queue size is: " + str(queue.qsize()))
        parent = queue.get()
        room = get_state(parent)
        neighbors.clear()
        neighbors = room['neighbors']

    print("Constructed Graph: " + str(my_graph))
    return my_graph

if __name__ == "__main__":
    # Your code starts here
    empty_room = get_state('7f3dc077574c013d98b2de8f735058b4')
    print("Original JSON: " + str(empty_room))
    graph = construct_graph_from_json(g.AdjacencyList(), empty_room)

    #print(transition_state(empty_room['id'], empty_room['neighbors'][0]['id']))
    bfs_path = s.bfs(graph, g.Node('7f3dc077574c013d98b2de8f735058b4'), g.Node('f1f131f647621a4be7c71292e79613f9'))
    if bfs_path:
        print("Bfs_path: "+ str(bfs_path))
    else:
        print("Bfs_path: No Path Found")

    dijkstra_path = dijkstra_search(graph, g.Node('7f3dc077574c013d98b2de8f735058b4'), g.Node('f1f131f647621a4be7c71292e79613f9'))
    if dijkstra_path:
        print("Dijkstra_path: " + str(dijkstra_path))
    else:
        print("Dijkstra_path: No Path Found")