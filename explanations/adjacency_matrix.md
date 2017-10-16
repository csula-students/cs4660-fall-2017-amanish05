```
nodes = [1, 2, 3]
adjacency_matrix = [
    [0, 1, 0],
    [1, 0, 1],
    [1, 0, 0]
]
```

```
// remove node2
nodes = [1, 3]
adjacency_matrix = [
    [0, 0],
    [1, 0]
]
// really wrong
return adjacency_matrix[node.data] // adjacency_matrix[3]
```

```
// Wrong
adjacency_list = {
    Node(1): [Node(2), Node(3)],
    Node(2): [],
    Node(3): [Node(1)]
}

adjacency_list = {
    Node(1): [Edge(Node(1), Node(2), 1), Edge(Node(1), Node(3), 1)]
}
adjacency_list = {
    Node(1): [(Node(2), 1), (Node(3), 1)]
}
```
