"""
Searches module defines all different search algorithms
"""
from queue import *
from graph import graph as g
import heapq

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

def dfs(graph, initial_node, dest_node):
    """
    Depth First Search
    uses graph to do search from the initial_node to dest_node
    returns a list of actions going from the initial node to dest_node
    """

    _visited_list, stack = [], [initial_node]

    _trace_nodes = {}
    result = []

    while stack:
        current_node = stack.pop()
        _visited_list.append(current_node)

        if(current_node == dest_node):
            break

        neighbors = graph.neighbors(current_node)
        for neighbors_node in neighbors[::-1]:
            if neighbors_node not in _visited_list:
                _trace_nodes[neighbors_node] = current_node
                stack.append(neighbors_node)

    if (dest_node in _trace_nodes):
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
        # print(current_node)

        if current_node == dest_node:
            break

        for neighbors_node in graph.neighbors(current_node):
            # print("Current node cost is: " +str(cost[current_node]))
            # print("Distance cost is: " + str(graph.distance(current_node, neighbors_node)))
            total_cost = cost[current_node] + graph.distance(current_node, neighbors_node)
            # print("Current distance is: " + str(total_cost))

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

        print(result)
    return result

def a_star_search(graph, initial_node, dest_node):
    """
    A* Search
    uses graph to do search from the initial_node to dest_node
    returns a list of actions going from the initial node to dest_node
    """
    queue, _trace_nodes, cost, result = [], {}, {}, []
    heapq.heappush(queue, PriorityNode(0, initial_node))
    cost[initial_node] = 0

    while queue:
        current_node = heapq.heappop(queue).node
        # print(current_node)

        if current_node == dest_node:
            break

        for neighbors_node in graph.neighbors(current_node):
            total_cost = cost[current_node] + graph.distance(current_node, neighbors_node)

            if neighbors_node not in cost or total_cost < cost[neighbors_node]:
                cost[neighbors_node] = total_cost

                priority = total_cost + heuristic(current_node.data, neighbors_node.data)
                heapq.heappush(queue, PriorityNode(priority, neighbors_node))

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

    return result

def heuristic(node, goal):
    #Diagonal distance for heuristic ( Chebyshev distance becuse minimum_cost_1 = 1, minimum_cost_2 = 1)
    minimum_cost_1 = 1
    minimum_cost_2 = 1
    dx = abs(node.x - goal.x)
    dy = abs(node.y - goal.y)
    return minimum_cost_1 * (dx + dy) + (minimum_cost_2 - 2 * minimum_cost_1) * min(dx, dy)

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