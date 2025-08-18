# Getting Started with libgraph

Welcome to libgraph! This guide will get you from zero to your first working graph and pathfinding algorithm in under 20 minutes.

## Quick Start (5 Minutes)

### 1. Installation

**Option A: Header-Only (Fastest)**
```bash
git clone https://github.com/rxdu/libgraph.git
cp -r libgraph/include/graph /path/to/your/project/
```

**Option B: CMake Integration**
```bash
git clone https://github.com/rxdu/libgraph.git
mkdir build && cd build
cmake ..
sudo make install
```

### 2. Your First Graph (60 seconds)

Create a simple 3-vertex triangle:

```cpp
#include "graph/graph.hpp"
#include "graph/search/dijkstra.hpp"
#include <iostream>

using namespace xmotion;

// Step 1: Define your state (what vertices represent)
struct SimpleState {
    int id;
    std::string name;
    
    SimpleState(int i, const std::string& n) : id(i), name(n) {}
};

int main() {
    // Step 2: Create the graph
    Graph<SimpleState> graph;
    
    // Step 3: Add vertices
    SimpleState home{0, "Home"};
    SimpleState work{1, "Work"};  
    SimpleState store{2, "Store"};
    
    graph.AddVertex(home);
    graph.AddVertex(work);
    graph.AddVertex(store);
    
    // Step 4: Connect with weighted edges (distance in km)
    graph.AddEdge(home, work, 5.0);    // Home -> Work: 5km
    graph.AddEdge(home, store, 3.0);   // Home -> Store: 3km  
    graph.AddEdge(store, work, 4.0);   // Store -> Work: 4km
    
    // Step 5: Find shortest path from Home to Work
    auto path = Dijkstra::Search(graph, home, work);
    
    // Step 6: Print the result
    std::cout << "Shortest path from Home to Work:\n";
    for (const auto& state : path) {
        std::cout << "  -> " << state.name << " (ID: " << state.id << ")\n";
    }
    
    return 0;
}
```

**Expected Output:**
```
Shortest path from Home to Work:
  -> Home (ID: 0)
  -> Store (ID: 2)  
  -> Work (ID: 1)
```

Congratulations! You just created a graph, added vertices and edges, and found the optimal path using Dijkstra's algorithm.

---

## Understanding the Basics

### How libgraph Works

libgraph uses **three template parameters** to create flexible, type-safe graphs:

```cpp
Graph<State, Transition, StateIndexer>
```

- **State**: What your vertices represent (locations, game states, etc.)
- **Transition**: Edge weights/costs (default: `double` for distances/costs)
- **StateIndexer**: How to identify unique states (default: uses `id`, `id_`, or `GetId()`)

### State Requirements

Your `State` class needs a unique identifier. The **default indexer** automatically works with:

```cpp
struct MyState {
    int64_t id;        // ✅ Works automatically
    // ... other data
};

// OR
struct MyState {
    int64_t id_;       // ✅ Works automatically  
    // ... other data
};

// OR  
struct MyState {
    int64_t GetId() const { return some_unique_value; }  // ✅ Works automatically
};
```

### Search Algorithms Available

| Algorithm | Best For | Example Use Case |
|-----------|----------|------------------|
| **Dijkstra** | Shortest paths, guaranteed optimal | GPS navigation, network routing |
| **A\*** | Shortest paths with heuristic speedup | Game AI, robotics pathfinding |
| **BFS** | Shortest path by edge count | Social networks, web crawling |
| **DFS** | Graph traversal, reachability | Maze solving, dependency analysis |

---

## Progressive Examples

### Example 1: Grid-Based Game Map

Perfect for game development or robotics:

```cpp
#include "graph/graph.hpp"
#include "graph/search/astar.hpp"
#include <cmath>

struct GridCell {
    int x, y;
    bool walkable;
    
    GridCell(int x, int y, bool walkable = true) 
        : x(x), y(y), walkable(walkable) {}
    
    // Required for default indexer
    int64_t GetId() const { return y * 1000 + x; }  // Assume max 1000x1000 grid
};

// Heuristic function for A* (Manhattan distance)
double ManhattanDistance(const GridCell& from, const GridCell& to) {
    return std::abs(from.x - to.x) + std::abs(from.y - to.y);
}

int main() {
    Graph<GridCell> grid;
    
    // Create 3x3 grid
    for (int y = 0; y < 3; ++y) {
        for (int x = 0; x < 3; ++x) {
            GridCell cell(x, y);
            grid.AddVertex(cell);
        }
    }
    
    // Connect adjacent cells (4-connectivity)
    for (int y = 0; y < 3; ++y) {
        for (int x = 0; x < 3; ++x) {
            GridCell current(x, y);
            
            // Connect to right neighbor
            if (x < 2) {
                GridCell right(x + 1, y);
                grid.AddUndirectedEdge(current, right, 1.0);  // Cost = 1
            }
            
            // Connect to bottom neighbor  
            if (y < 2) {
                GridCell bottom(x, y + 1);
                grid.AddUndirectedEdge(current, bottom, 1.0);  // Cost = 1
            }
        }
    }
    
    // Find path from top-left to bottom-right
    GridCell start(0, 0);
    GridCell goal(2, 2);
    
    auto path = AStar::Search(grid, start, goal, ManhattanDistance);
    
    std::cout << "Path from (0,0) to (2,2):\n";
    for (const auto& cell : path) {
        std::cout << "  (" << cell.x << "," << cell.y << ")\n";
    }
    
    return 0;
}
```

### Example 2: Custom Cost Types

For multi-criteria optimization (transit planning, resource management):

```cpp
#include "graph/graph.hpp"
#include "graph/search/dijkstra.hpp"

// Multi-criteria cost: [time, money, comfort]
struct TravelCost {
    double time_minutes;
    double cost_dollars;
    double comfort_rating;  // 1-10, higher is better
    
    TravelCost(double t = 0, double c = 0, double comfort = 10) 
        : time_minutes(t), cost_dollars(c), comfort_rating(comfort) {}
    
    // Lexicographic comparison: time first, then cost, then comfort
    bool operator<(const TravelCost& other) const {
        if (time_minutes != other.time_minutes) 
            return time_minutes < other.time_minutes;
        if (cost_dollars != other.cost_dollars) 
            return cost_dollars < other.cost_dollars;
        return comfort_rating > other.comfort_rating;  // Higher comfort is better
    }
    
    TravelCost operator+(const TravelCost& other) const {
        return TravelCost(
            time_minutes + other.time_minutes,
            cost_dollars + other.cost_dollars,
            std::min(comfort_rating, other.comfort_rating)  // Worst comfort along path
        );
    }
};

// Specialize CostTraits for our custom cost type
namespace xmotion {
    template<>
    struct CostTraits<TravelCost> {
        static TravelCost infinity() {
            return TravelCost(std::numeric_limits<double>::infinity(), 
                            std::numeric_limits<double>::infinity(), 0);
        }
    };
}

struct Location {
    int id;
    std::string name;
    
    Location(int i, const std::string& n) : id(i), name(n) {}
};

int main() {
    Graph<Location, TravelCost> transport;
    
    Location home{0, "Home"};
    Location work{1, "Work"}; 
    Location downtown{2, "Downtown"};
    
    transport.AddVertex(home);
    transport.AddVertex(work);
    transport.AddVertex(downtown);
    
    // Different travel options with multi-criteria costs
    // Format: TravelCost(time_minutes, cost_dollars, comfort_rating)
    transport.AddEdge(home, work, TravelCost(45, 2.50, 6));      // Bus: slow, cheap, okay
    transport.AddEdge(home, downtown, TravelCost(15, 12.00, 9)); // Taxi: fast, expensive, comfy
    transport.AddEdge(downtown, work, TravelCost(20, 8.00, 8));  // Ride-share: medium all
    
    auto path = Dijkstra::Search(transport, home, work);
    
    std::cout << "Optimal multi-criteria path:\n";
    for (const auto& location : path) {
        std::cout << "  -> " << location.name << "\n";
    }
    
    return 0;
}
```

---

## Thread-Safe Concurrent Searches

For high-performance applications needing multiple simultaneous pathfinding:

```cpp
#include "graph/graph.hpp"
#include "graph/search/dijkstra.hpp"
#include "graph/search/search_context.hpp"
#include <thread>
#include <vector>

struct Node {
    int id;
    Node(int i) : id(i) {}
};

void worker_search(const Graph<Node>& graph, int worker_id) {
    // Each thread gets its own SearchContext for thread safety
    SearchContext<Node> context;
    
    Node start{worker_id * 10};      // Different start points
    Node goal{worker_id * 10 + 5};   // Different goals
    
    // Thread-safe search using context
    auto path = Dijkstra::Search(graph, context, start, goal);
    
    std::cout << "Worker " << worker_id << " found path length: " << path.size() << "\n";
}

int main() {
    Graph<Node> graph;
    
    // Build a larger graph for testing
    for (int i = 0; i < 100; ++i) {
        graph.AddVertex(Node{i});
        if (i > 0) {
            graph.AddEdge(Node{i-1}, Node{i}, 1.0);  // Linear chain
        }
    }
    
    // Launch multiple concurrent searches
    std::vector<std::thread> workers;
    for (int i = 0; i < 4; ++i) {
        workers.emplace_back(worker_search, std::ref(graph), i);
    }
    
    // Wait for all searches to complete
    for (auto& worker : workers) {
        worker.join();
    }
    
    return 0;
}
```

---

## Next Steps

Now that you have the basics, explore these advanced topics:

1. **[Complete API Reference](api.md)** - All classes and methods
2. **[Search Algorithms Guide](search-algorithms.md)** - Deep dive into A*, Dijkstra, BFS, DFS  
3. **[Architecture Overview](architecture.md)** - Understanding the template system
4. **[Advanced Features](advanced-features.md)** - Custom indexers, validation, batch operations
5. **[Performance Testing](performance_testing.md)** - Optimize your graph operations

### Quick Tips for Success

- **Start simple**: Use basic `int` or `string` states before complex custom types
- **Unique IDs matter**: Ensure your states have unique identifiers for the indexer
- **Choose the right algorithm**: Dijkstra for shortest paths, A* when you have good heuristics
- **Thread safety**: Use `SearchContext` for concurrent searches on the same graph
- **Performance**: Pre-allocate with `graph.reserve(n)` for large graphs

### Common Patterns

```cpp
// Pattern 1: Quick prototype with simple states
Graph<int> simple_graph;
simple_graph.AddVertex(1);
simple_graph.AddVertex(2);
simple_graph.AddEdge(1, 2, 5.0);

// Pattern 2: Real application with custom states
struct MyGameState { int x, y, hp; int64_t GetId() const; };
Graph<MyGameState> game_graph;

// Pattern 3: Custom costs for multi-objective optimization  
struct MyCost { double time, energy; /* comparison operators */ };
Graph<Location, MyCost> optimized_graph;
```

Welcome to efficient graph computing with libgraph!