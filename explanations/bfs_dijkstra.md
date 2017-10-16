```
// working memory for BFS
distances = {
    1: 0,
    2: 1,
    3: 1,
    4: 2,
    6: 2,
    10: 2,
    5: 3,
    7: 3
}
parents = {
    1: None,
    2: 1,
    3: 1,
    4: 2,
    6: 3,
    10: 3,
    5: 4,
    7: 4
}
queue = [ 5, 7 ]
current_node = 10
// found the path in node 10
// use parents to backtrack
10 <- 3 <- 1
// reverse
1 -> 3 -> 10
```








```
// working memory for Dijkstra
distances = {
    0: 0,
    1: (0 + 4),
    3: (0 + 1),
    6: (0 + 3),
    5: (1 + 11),
    4: (4 + (-10)),
    2: (4 + 7)
}
parents = {
    0: None,
    1: 0,
    3: 0,
    6: 0,
    5: 3,
    4: 1,
    2: 1
}
queue = [ (1, 4), (4, 4), (5, 12) ]
current_node = 1
neighbors = [4]
c_n = 4
alternative_distance = 4 + (-10) = -6
1 -> 4: -10
```











