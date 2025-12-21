libgraph: C++ Graph Library                         {#mainpage}
=============================

A modern, header-only C++11 library for graph construction and pathfinding.

## Features

- **Header-only**: Include and use, no linking required
- **Thread-safe**: Concurrent searches via external SearchContext
- **Generic costs**: Custom cost types with lexicographic comparison
- **Complete algorithms**: A*, Dijkstra, BFS, DFS
- **Production-ready**: Early termination, multi-goal search, rich results

## Quick Start

~~~cpp
#include "graph/graph.hpp"
#include "graph/search/dijkstra.hpp"

using namespace xmotion;

struct Location {
    int id;
    double x, y;
};

int main() {
    Graph<Location> map;

    Location a{0, 0.0, 0.0}, b{1, 5.0, 0.0}, c{2, 2.5, 4.0};
    map.AddVertex(a);
    map.AddVertex(b);
    map.AddVertex(c);

    map.AddEdge(a, c, 2.0);
    map.AddEdge(c, b, 2.0);
    map.AddEdge(a, b, 5.0);

    auto path = Dijkstra::Search(&map, a, b);
    // Returns: a -> c -> b (cost 4.0, shorter than direct route of 5.0)
}
~~~

## Thread-Safe Search

~~~cpp
void worker(const Graph<Location>& map, Location start, Location goal) {
    SearchContext<Location> context;  // Each thread gets its own
    auto path = Dijkstra::Search(&map, context, start, goal);
}
~~~

## Production Features

### Rich Results

~~~cpp
auto result = Dijkstra::SearchMultiGoal(&graph, context, start, goals);
if (result.found) {
    // result.path        - the path to nearest goal
    // result.total_cost  - accumulated cost
    // result.goal_index  - which goal was reached
    // result.nodes_expanded - for diagnostics
}
~~~

### Early Termination

~~~cpp
auto limits = SearchLimits::MaxExpansions(1000);  // Stop after 1000 nodes
// or
auto limits = SearchLimits::Timeout(50);  // Stop after 50ms

auto result = SearchAlgorithm<...>::SearchWithLimits(
    &graph, context, start, goal, strategy, limits);
~~~

### Multi-Goal Search

~~~cpp
std::vector<Location> charging_stations = {s1, s2, s3};
auto result = Dijkstra::SearchMultiGoal(&graph, context, robot_pos, charging_stations);
// Finds path to nearest charging station
~~~

## Algorithms

| Algorithm | Best For | Optimality |
|-----------|----------|------------|
| Dijkstra | Weighted graphs | Optimal |
| A* | With good heuristic | Optimal |
| BFS | Unweighted graphs | Optimal |
| DFS | Reachability | Not optimal |

### Dijkstra

~~~cpp
auto path = Dijkstra::Search(&graph, start, goal);
~~~

### A*

~~~cpp
auto heuristic = [](const Location& a, const Location& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
};
auto path = AStar::Search(&graph, context, start, goal, heuristic);
~~~

### BFS / DFS

~~~cpp
auto path = BFS::Search(&graph, start, goal);
bool reachable = BFS::IsReachable(&graph, start, target);

auto path = DFS::Search(&graph, start, goal);
DFS::TraverseAll(&graph, context, start);  // Visit all reachable
~~~

## Custom Cost Types

~~~cpp
struct RouteCost {
    double time, distance;

    bool operator<(const RouteCost& o) const {
        return std::tie(time, distance) < std::tie(o.time, o.distance);
    }
    RouteCost operator+(const RouteCost& o) const {
        return {time + o.time, distance + o.distance};
    }
};

template<>
struct xmotion::CostTraits<RouteCost> {
    static RouteCost infinity() {
        return {std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max()};
    }
};

Graph<Location, RouteCost> graph;
~~~

## State Indexing

States need unique IDs. The library auto-detects common patterns:

~~~cpp
struct State { int64_t id; };      // Member 'id'
struct State { int64_t id_; };     // Member 'id_'
struct State { int64_t GetId() const; };  // Method 'GetId()'
~~~

Or provide a custom indexer:

~~~cpp
struct MyIndexer {
    int64_t operator()(const MyState& s) const { return s.unique_field; }
};
Graph<MyState, double, MyIndexer> graph;
~~~

## Examples

Working examples in `sample/`:

| File | Description |
|------|-------------|
| `simple_graph_demo.cpp` | Basic graph and pathfinding |
| `thread_safe_search_demo.cpp` | Concurrent searches |
| `lexicographic_cost_demo.cpp` | Multi-criteria costs |
| `tuple_cost_demo.cpp` | std::tuple as cost type |
| `incremental_search_demo.cpp` | Dynamic graph updates |
| `graph_type_demo.cpp` | Different graph configurations |

## API Overview

### Core Classes

- **Graph<State, Transition, Indexer>** - Main graph container
- **SearchContext<State, Transition, Indexer>** - Thread-local search state
- **Dijkstra, AStar, BFS, DFS** - Search algorithm classes

### Result Types

- **Path<State>** - Simple vector of states
- **PathResult<State, Cost>** - Path + cost + nodes_expanded
- **MultiGoalResult<State, Cost>** - PathResult + goal_index
- **SearchLimits** - Early termination configuration

### Key Methods

~~~cpp
// Graph
graph.AddVertex(state);
graph.AddEdge(from, to, cost);
graph.FindVertex(state);
graph.GetNeighbors(state);

// Search
Dijkstra::Search(&graph, start, goal);
Dijkstra::SearchMultiGoal(&graph, context, start, goals);
AStar::Search(&graph, context, start, goal, heuristic);
BFS::IsReachable(&graph, start, target);

// Context
context.Reset();
context.Reserve(n);
context.GetNodesExpanded();
~~~

## License

MIT License
