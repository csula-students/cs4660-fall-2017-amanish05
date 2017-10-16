# Quiz 2

Total points: 10 pts

Due date: Oct 14th 10:00.am (2 hours period)

## Context

You woke up in the middle of an empty room. You look around and realize you are
in an empty room of a *dungeon*.

Your job is to find your way out from this empty room to a dark room containing
the portal to outside world.

You can see the rooms visually here: http://192.241.218.106:9000/secret

## Technical Requirement

Implement BFS & Dijkstra search to find path from Empty Room
`7f3dc077574c013d98b2de8f735058b4` to Dark Room `f1f131f647621a4be7c71292e79613f9`

> For Dikjstra search, you will need to take the event effect (edge cost) into
> the consideration. Find out the best path that returns the highest effect value
> with the constraint of each room can only be visited once.

You will have to modify `cs4660/quiz/main.py` to find the path.

Please submit `solution.txt` containing link to your `cs4660/quiz/main.py` and path from
initial state to destination state to CSNS when you are done:

> You should commit your changes to your branch `quiz-2`

Example of `solution.txt`:

```
link to my solution:
https://github.com/csula-students/cs4660-fall-2017-rcliao/blob/quiz-2/cs4660/quiz/main.py

BFS Path:
Empty Room(7f3dc077574c013d98b2de8f735058b4):Hall Way(0d1d67f3e6bf24e4c2acff975025a497):0
…
Total hp: 20

Dijkstra Path:
Empty Room(7f3dc077574c013d98b2de8f735058b4):Hall Way(0d1d67f3e6bf24e4c2acff975025a497):0
…
Total hp: 70
```

**Hint**

* You can and should reuse your `graph` and `search` implementation
* You don't know all the state ahead of time, but you can use the server as the state holder to traverse through states
    * In other word, the server state is set in stone doesn't matter how many time you call the end point.
* [Optional] Network between your computer to the server is expensive, you should store the graph down as JSON format from the server once from the initial request and read from JSON for any following tests
