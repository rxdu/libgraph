# libgraph Documentation

## Overview

libgraph is a modern, header-only C++11 library for graph construction and pathfinding algorithms. It provides high-performance graph operations with thread-safe concurrent searches and support for generic cost types.

## Documentation Structure

### Core Documentation

- **[API Reference](./api.md)** - Complete API documentation for all classes and methods
- **[Getting Started Guide](./getting_started.md)** - Quick introduction and your first graph in 20 minutes
- **[Tutorial Series](./tutorials/)** - Progressive learning path from basics to advanced features

### Design Documentation

- **[Architecture Overview](./architecture.md)** - System design, template patterns, and implementation details
- **[Search Framework](./search_framework.md)** - Unified search algorithm framework using CRTP strategy pattern
- **[Thread Safety Design](./thread_safety_design.md)** - Concurrent search architecture and SearchContext design

### Advanced Topics

- **[Performance Testing](./performance_testing.md)** - Benchmarking framework and optimization targets
- **[Dynamic Priority Queue](./dynamic_priority_queue.md)** - Implementation details of the priority queue with update capability
- **[Large Scale Testing](./large_scale_performance_testing.md)** - Performance analysis with graphs up to 1M+ vertices

### Migration and Updates

- **[Cost Type Removal Summary](./costtype_removal_summary.md)** - Migration guide for generic cost type support
- **[Search Framework Migration](./search_framework.md)** - Guide for transitioning to the unified search framework

## Library Architecture

### Template System

The library is built around three main template parameters:

```cpp
template<typename State, typename Transition = double, typename StateIndexer = DefaultIndexer<State>>
class Graph;
```

- **State**: Your vertex data type (locations, game states, network nodes, etc.)
- **Transition**: Edge weight/cost type (defaults to `double`, supports custom types)
- **StateIndexer**: Functor for generating unique IDs from states (auto-detects `id`, `id_`, or `GetId()`)

### Core Components

#### Graph Data Structure

The graph uses an adjacency list representation with O(m+n) space complexity:

* **Graph** container
  * **Vertex** collection (hash map with O(1) average access)
    * **Edge** list (linked list for each vertex)
    * State data storage
    * Reverse references for efficient operations
  * Thread-safe search support via external SearchContext
  * RAII memory management with `std::unique_ptr`

#### Search Algorithms

Four algorithms implemented with unified framework:

| Algorithm | Use Case | Time Complexity | Optimality |
|-----------|----------|-----------------|------------|
| **Dijkstra** | Shortest paths in weighted graphs | O((m+n) log n) | Guaranteed optimal |
| **A\*** | Heuristic-guided pathfinding | O((m+n) log n)* | Optimal with admissible heuristic |
| **BFS** | Shortest paths by edge count | O(m+n) | Optimal for unweighted |
| **DFS** | Graph traversal, reachability | O(m+n) | Not optimal for paths |

*\*A* performance depends on heuristic quality*

## Quick Example

```cpp
#include "graph/graph.hpp"
#include "graph/search/dijkstra.hpp"

using namespace xmotion;

// Define your state type
struct Location {
    int id;
    std::string name;
    double x, y;  // Coordinates
    
    Location(int i, const std::string& n, double x, double y) 
        : id(i), name(n), x(x), y(y) {}
};

int main() {
    // Create graph
    Graph<Location> map;
    
    // Add vertices
    Location home{0, "Home", 0.0, 0.0};
    Location work{1, "Work", 10.0, 5.0};
    Location store{2, "Store", 3.0, 2.0};
    
    map.AddVertex(home);
    map.AddVertex(work);
    map.AddVertex(store);
    
    // Add weighted edges
    map.AddEdge(home, store, 3.5);   // Distance/cost
    map.AddEdge(store, work, 7.2);
    map.AddEdge(home, work, 12.0);   // Direct route
    
    // Find optimal path
    auto path = Dijkstra::Search(map, home, work);
    
    // Path will be: Home -> Store -> Work (total cost: 10.7)
    // Better than direct route (cost: 12.0)
    
    return 0;
}
```

## Thread Safety

The library supports concurrent read-only searches through SearchContext:

```cpp
// Thread-safe concurrent searches
void worker_thread(const Graph<Location>& map) {
    SearchContext<Location> context;  // Thread-local search state
    auto path = Dijkstra::Search(map, context, start, goal);
    // Process path...
}
```

Graph modifications require external synchronization.

## Advanced Features

### Custom Cost Types

```cpp
struct MultiCriteriaCost {
    double time;
    double distance;
    double toll;
    
    bool operator<(const MultiCriteriaCost& other) const {
        // Lexicographic comparison: time > distance > toll
        if (time != other.time) return time < other.time;
        if (distance != other.distance) return distance < other.distance;
        return toll < other.toll;
    }
    
    MultiCriteriaCost operator+(const MultiCriteriaCost& other) const {
        return {time + other.time, distance + other.distance, toll + other.toll};
    }
};

// Specialize CostTraits for custom type
namespace xmotion {
    template<>
    struct CostTraits<MultiCriteriaCost> {
        static MultiCriteriaCost infinity() {
            return {std::numeric_limits<double>::max(), 
                   std::numeric_limits<double>::max(),
                   std::numeric_limits<double>::max()};
        }
    };
}

Graph<Location, MultiCriteriaCost> multi_criteria_map;
```

### Performance Optimization

```cpp
// Pre-allocate for large graphs
graph.reserve(100000);  // Reserve space for 100k vertices

// Batch operations
std::vector<Location> locations = LoadLocations();
graph.AddVertices(locations);

// Reuse search context
SearchContext<Location> context;
context.PreAllocate(100000);  // Pre-allocate search state
for (const auto& query : queries) {
    context.Reset();  // Clear previous search
    auto path = Dijkstra::Search(graph, context, query.start, query.goal);
}
```

## Building Documentation

### Doxygen Documentation

Generate detailed API documentation:

```bash
cd docs
doxygen doxygen/Doxyfile
# Open docs/doxygen/html/index.html in browser
```

### Online Documentation

- GitHub Repository: [https://github.com/rxdu/libgraph](https://github.com/rxdu/libgraph)
- API Reference: [https://rdu.im/libgraph/](https://rdu.im/libgraph/)

## Getting Help

- **[Issue Tracker](https://github.com/rxdu/libgraph/issues)** - Report bugs or request features
- **[Discussions](https://github.com/rxdu/libgraph/discussions)** - Ask questions and share experiences
- **[Examples](../sample/)** - Working examples demonstrating various features

## License

This library is distributed under the MIT License. See [LICENSE](../LICENSE) for details.