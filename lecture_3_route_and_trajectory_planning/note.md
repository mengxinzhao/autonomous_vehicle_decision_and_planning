## Note
###  Mission planning
- 功能： start position， goal， lane graph + cost function ==> route planning ==> Route(sequence of lanes / road segments)
- cost function： could penalize lane changes， turns（left / right）， narrow lanes etc
- 经典算法
  - Dijkstra算法原理 single source shorted path。[算法图解](https://www.freecodecamp.org/chinese/news/dijkstras-shortest-path-algorithm-visual-introduction/)
  - A* 用启发式算法 f(n) = g(n) + h(n). where n is the next node on the path, g(n) is the cost of the path from the start node to n, and h(n) is a heuristic function that estimates the cost of the cheapest path from n to the goal. The heuristic function is problem-specific. If the heuristic function is admissible – meaning that it never overestimates the actual cost to get to the goal – A* is guaranteed to return a least-cost path from start to goal. 缺点：不满足运动学约束，难以做路径追踪
  - Hybrid A*： 改进A*难点。节点的拓展基于车辆运动学模型，cost计算基于grid map。 论文：
    - [Practical Search Techniques in Path Planning for Autonomous Driving](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf). 要旨：our hybrid-state A* is not guaranteed
to find the minimal-cost solution, due to its merging of
continuous-coordinate states that occupy the same cell in the
discretized space. However, the resulting path is guaranteed
to be drivable (rather than being piecewise-linear as in the
case of standard A*). Also, in practice, the hybrid-A* solution typically lies in the neighborhood of the global optimum, allowing us to frequently arrive at the globally optimal
solution via the second phase of our algorithmm (which usesg radient descent to locally improve the path）
    - [Medium paper reading note](https://medium.com/@junbs95/gentle-introduction-to-hybrid-a-star-9ce93c0d7869) 
    - Discretized search space, kinematics models (x, y, $\theta$)
    - Node Expansion: 1). In hybrid A*, a propagation of state $s_n$ = ($x_n$, $\theta_n$) opens new cells by simulating the kinematics. 2). cost_so_far g(s) is determined by length of the arc between $s_n$ and $s_{n+1}$. Can impose higher penalty on turning movement. 3). prune is needed when 2+ states share the same cell with the same angular bin. We remove the states if another state in the same discretization (same 2d cell and angle bin) has attained a less cost-so-far.
    - Heuristic h(s). Karl论中用max(unconstrained_cost_for_a_star, constrained_cost_for_a_star). unconstrained_cost 不考虑车辆动力学，只考虑能否到达的最短路径，会出现车辆横漂的路径；constraint_cost考虑车辆动力学，但可能出现沿着现有航向角驾驶撞墙的路径
    - Analytical expansion (a.k.a., shot to goal): We achieve an optimal solution if we can connect this best candidate directly to the goal without any collisions(Dubins or Reeds Shepp) path. It works in sparse regions, but not in dense spots. 
    - [Dubins path](https://en.wikipedia.org/wiki/Dubins_path), [Reeds Shepp](https://lavalle.pl/planning/node822.html). difference between the two is Reeds Shepp allows reverse motion
    - sample implementation
        - [KTH thesis](https://github.com/karlkurzer/path_planner) by Karlkurzer.
        - [nav2 (ROS2)](https://github.com/ros-navigation/navigation2/tree/main/nav2_smac_planner) by Samsung North America research
        - [Unity](https://github.com/Habrador/Self-driving-vehicle)
        - [Python](https://github.com/zhm-real/MotionPlanning)



<details>
<summary>Dijkstra algorithm</summary>

```
1   function Dijkstra(Graph, source):
2       Q ← Queue storing vertex priority
3       
4       dist[source] ← 0                          // Initialization
5       Q.add_with_priority(source, 0)            // associated priority equals dist[·]
6
7       for each vertex v in Graph.Vertices:
8           if v ≠ source
9               prev[v] ← UNDEFINED               // Predecessor of v
10              dist[v] ← INFINITY                // Unknown distance from source to v
11              Q.add_with_priority(v, INFINITY)
12
13
14      while Q is not empty:                     // The main loop
15          u ← Q.extract_min()                   // Remove and return best vertex
16          for each arc (u, v) :                 // Go through all v neighbors of u
17              alt ← dist[u] + Graph.Edges(u, v)
18              if alt < dist[v]:
19                  prev[v] ← u
20                  dist[v] ← alt
21                  Q.decrease_priority(v, alt)
22
23      return (dist, prev)

```
```block
#include <iostream>
#include <vector>
#include <queue>
#include <limits> // For std::numeric_limits

const int INF = std::numeric_limits<int>::max();

// Structure to represent an edge
struct Edge {
    int to;
    int weight;
};

// Function to implement Dijkstra's algorithm
std::vector<int> dijkstra(int V, const std::vector<std::vector<Edge>>& adj, int src) {
    std::vector<int> dist(V, INF);
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;

    dist[src] = 0;
    pq.push({0, src}); // {distance, vertex}

    while (!pq.empty()) {
        int d = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        // If a shorter path to u has already been found, skip
        if (d > dist[u]) {
            continue;
        }

        // Explore neighbors of u
        for (const Edge& edge : adj[u]) {
            int v = edge.to;
            int weight = edge.weight;

            // Relaxation step
            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                pq.push({dist[v], v});
            }
        }
    }
    return dist;
}

int main() {
    int V = 5; // Number of vertices
    std::vector<std::vector<Edge>> adj(V);

    // Add edges to the graph
    adj[0].push_back({1, 10});
    adj[0].push_back({2, 3});
    adj[1].push_back({2, 1});
    adj[1].push_back({3, 2});
    adj[2].push_back({1, 4});
    adj[2].push_back({3, 8});
    adj[2].push_back({4, 2});
    adj[3].push_back({4, 5});

    int source = 0;
    std::vector<int> shortest_distances = dijkstra(V, adj, source);

    std::cout << "Shortest distances from source " << source << ":\n";
    for (int i = 0; i < V; ++i) {
        if (shortest_distances[i] == INF) {
            std::cout << "To vertex " << i << ": Unreachable\n";
        } else {
            std::cout << "To vertex " << i << ": " << shortest_distances[i] << "\n";
        }
    }

    return 0;
}
```
</details>

<details>
<summary>A* algorithm</summary>

```block
// from wiki
function reconstruct_path(cameFrom, current)
    total_path := {current}
    while current in cameFrom.Keys:
        current := cameFrom[current]
        total_path.prepend(current)
    return total_path

// A* finds a path from start to goal.
// h is the heuristic function. h(n) estimates the cost to reach goal from node n.
function A_Star(start, goal, h)
    // The set of discovered nodes that may need to be (re-)expanded.
    // Initially, only the start node is known.
    // This is usually implemented as a min-heap or priority queue rather than a hash-set.
    openSet := {start}

    // For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from the start
    // to n currently known.
    cameFrom := an empty map

    // For node n, gScore[n] is the currently known cost of the cheapest path from start to n.
    gScore := map with default value of Infinity
    gScore[start] := 0

    // For node n, fScore[n] := gScore[n] + h(n). fScore[n] represents our current best guess as to
    // how cheap a path could be from start to finish if it goes through n.
    fScore := map with default value of Infinity
    fScore[start] := h(start)

    while openSet is not empty
        // This operation can occur in O(Log(N)) time if openSet is a min-heap or a priority queue
        current := the node in openSet having the lowest fScore[] value
        if current = goal
            return reconstruct_path(cameFrom, current)

        openSet.Remove(current)
        for each neighbor of current
            // d(current,neighbor) is the weight of the edge from current to neighbor
            // tentative_gScore is the distance from start to the neighbor through current
            tentative_gScore := gScore[current] + d(current, neighbor)
            if tentative_gScore < gScore[neighbor]
                // This path to neighbor is better than any previous one. Record it!
                cameFrom[neighbor] := current
                gScore[neighbor] := tentative_gScore
                fScore[neighbor] := tentative_gScore + h(neighbor)
                if neighbor not in openSet
                    openSet.add(neighbor)

    // Open set is empty but goal was never reached
    return failure
```
sample implementation 
- https://github.com/JDSherbert/A-Star-Pathfinding
- https://rosettacode.org/wiki/A*_search_algorithm#C++
</details>