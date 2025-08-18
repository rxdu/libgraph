libgraph: C++ Graph Library                         {#mainpage}
=============================

## Overview

libgraph is a modern, header-only C++11 library for graph construction and pathfinding algorithms. It provides high-performance graph operations with thread-safe concurrent searches and support for generic cost types.

### Key Features

- **High Performance**: O(m+n) space complexity with optimized priority queues
- **Thread-Safe**: Concurrent searches using external SearchContext
- **Generic**: Custom cost types, comparators, and state indexing
- **Complete Algorithm Suite**: A*, Dijkstra, BFS, DFS with unified framework
- **Robust**: Comprehensive exception handling, structure validation, memory safety via RAII
- **Well-Documented**: Extensive API reference, tutorials, and working examples

## Library Architecture

### Template System

The library is built around three main template parameters:

~~~cpp
template<typename State, typename Transition = double, typename StateIndexer = DefaultIndexer<State>>
class Graph;
~~~

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

*A* performance depends on heuristic quality*

## Quick Example

~~~cpp
#include "graph/graph.hpp"
#include "graph/search/dijkstra.hpp"

using namespace xmotion;

// Define your state type
struct Location {
    int id;
    std::string name;
    double x, y;  // Coordinates for heuristics
    
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
~~~

## Advanced Features

### Thread Safety

The library supports concurrent read-only searches through SearchContext:

~~~cpp
// Thread-safe concurrent searches
void worker_thread(const Graph<Location>& map) {
    SearchContext<Location> context;  // Thread-local search state
    auto path = Dijkstra::Search(map, context, start, goal);
    // Process path...
}
~~~

Graph modifications require external synchronization.

### Custom Cost Types

~~~cpp
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
~~~

### Performance Optimization

~~~cpp
// Pre-allocate for large graphs
graph.reserve(100000);  // Reserve space for 100k vertices

// Batch operations
std::vector<Location> locations = LoadLocations();
graph.AddVertices(locations);

// Reuse search context for multiple searches
SearchContext<Location> context;
context.PreAllocate(100000);  // Pre-allocate search state
for (const auto& query : queries) {
    context.Reset();  // Clear previous search
    auto path = Dijkstra::Search(graph, context, query.start, query.goal);
}
~~~

## Algorithm Usage

### Dijkstra Algorithm

For guaranteed optimal shortest paths:

~~~cpp
// Basic usage
auto path = Dijkstra::Search(graph, start, goal);

// Thread-safe usage  
SearchContext<State> context;
auto path = Dijkstra::Search(graph, context, start, goal);

// Custom cost comparator
auto path = Dijkstra::Search(graph, context, start, goal, std::greater<MyCost>());
~~~

### A* Algorithm

For heuristic-guided optimal pathfinding:

~~~cpp
// Euclidean distance heuristic
double EuclideanDistance(const Location& from, const Location& to) {
    double dx = from.x - to.x;
    double dy = from.y - to.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Basic usage
auto path = AStar::Search(graph, start, goal, EuclideanDistance);

// Thread-safe usage
SearchContext<Location> context;
auto path = AStar::Search(graph, context, start, goal, EuclideanDistance);
~~~

### BFS and DFS

For unweighted graphs and traversal:

~~~cpp
// Shortest path by edge count
auto bfs_path = BFS::Search(graph, start, goal);

// Graph traversal and reachability
auto dfs_path = DFS::Search(graph, start, goal);
~~~

## State Indexing

### Default Indexing

The DefaultIndexer automatically works with common patterns:

~~~cpp
struct MyState {
    int64_t id;        // Works automatically
    // OR
    int64_t id_;       // Works automatically  
    // OR
    int64_t GetId() const { return unique_value; }  // Works automatically
};
~~~

### Custom Indexing

For states that don't match default patterns:

~~~cpp
struct MyCustomIndexer {
    int64_t operator()(const MyState& state) const {
        return state.custom_unique_field;
    }
};

// Usage
Graph<MyState, double, MyCustomIndexer> graph;
~~~

## Memory Management

The library uses RAII for automatic memory management:

- **Graph**: Automatically manages vertex/edge memory using `std::unique_ptr`
- **No manual cleanup** required for graph structures
- **Copy/move semantics** work as expected
- **Exception safety** with strong guarantees for most operations

## Error Handling

Comprehensive exception hierarchy:

~~~cpp
try {
    auto& vertex = graph.GetVertexSafe(invalid_id);
} catch (const ElementNotFoundError& e) {
    std::cout << "Vertex not found: " << e.what() << std::endl;
}

try {
    graph.ValidateStructure();
} catch (const DataCorruptionError& e) {
    std::cout << "Graph corruption detected: " << e.what() << std::endl;
}
~~~

## Getting Started

1. **Include the headers**:
   ~~~cpp
   #include "graph/graph.hpp"
   #include "graph/search/dijkstra.hpp"
   ~~~

2. **Define your state type** with unique identification
3. **Create and populate the graph** with vertices and edges  
4. **Use search algorithms** to find optimal paths
5. **Implement thread safety** with SearchContext for concurrent usage

## Documentation

- **Getting Started Guide**: Step-by-step introduction
- **API Reference**: Complete class and method documentation
- **Tutorial Series**: Progressive learning from basics to advanced features
- **Architecture Overview**: Design patterns and implementation details
- **Performance Testing**: Benchmarking framework and optimization guides

## Examples

Working examples are available in the `sample/` directory:

- `simple_graph_demo.cpp` - Basic graph construction and pathfinding
- `thread_safe_search_demo.cpp` - Concurrent search demonstrations
- `lexicographic_cost_demo.cpp` - Multi-criteria optimization
- `incremental_search_demo.cpp` - Dynamic pathfinding

## License

This library is distributed under the MIT License.