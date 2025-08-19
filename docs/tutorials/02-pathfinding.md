# Tutorial 2: Simple Pathfinding

**Learning Objectives:** Use Dijkstra and A* algorithms to find optimal paths through graphs.

**Estimated Time:** 20 minutes

---

## Overview

Now that you can build graphs, let's learn how to find paths through them. This tutorial covers libgraph's search algorithms: Dijkstra for guaranteed optimal paths and A* for faster searches with heuristics.

## Complete Example

Let's extend our city map from Tutorial 1 with pathfinding capabilities:

```cpp
#include "graph/graph.hpp"
#include "graph/search/dijkstra.hpp"
#include "graph/search/astar.hpp"
#include <iostream>
#include <cmath>

using namespace xmotion;

// Enhanced location with coordinates for A* heuristic
struct Location {
    int id;
    std::string name;
    double x, y;  // Coordinates for heuristic calculation
    
    Location(int i, const std::string& n, double x_coord, double y_coord) 
        : id(i), name(n), x(x_coord), y(y_coord) {}
};

// Heuristic function for A* (Euclidean distance)
double EuclideanDistance(const Location& from, const Location& to) {
    double dx = from.x - to.x;
    double dy = from.y - to.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Alternative heuristic (Manhattan distance - faster to compute)
double ManhattanDistance(const Location& from, const Location& to) {
    return std::abs(from.x - to.x) + std::abs(from.y - to.y);
}

int main() {
    // Step 1: Create a more detailed city map with coordinates
    Graph<Location> city_map;
    
    // Add locations with (x, y) coordinates
    Location home{0, "Home", 0.0, 0.0};
    Location work{1, "Work", 10.0, 5.0};
    Location gym{2, "Gym", 3.0, 4.0};
    Location store{3, "Store", 2.0, 1.0};
    Location park{4, "Park", 8.0, 8.0};
    Location mall{5, "Mall", 12.0, 2.0};
    
    city_map.AddVertex(home);
    city_map.AddVertex(work);
    city_map.AddVertex(gym);
    city_map.AddVertex(store);
    city_map.AddVertex(park);
    city_map.AddVertex(mall);
    
    // Step 2: Add edges with realistic travel times (minutes)
    city_map.AddEdge(home, store, 5.0);    // Home → Store: 5 min
    city_map.AddEdge(home, gym, 8.0);      // Home → Gym: 8 min
    city_map.AddEdge(store, gym, 4.0);     // Store → Gym: 4 min
    city_map.AddEdge(store, work, 15.0);   // Store → Work: 15 min
    city_map.AddEdge(gym, park, 7.0);      // Gym → Park: 7 min
    city_map.AddEdge(gym, work, 12.0);     // Gym → Work: 12 min
    city_map.AddEdge(park, work, 6.0);     // Park → Work: 6 min
    city_map.AddEdge(work, mall, 8.0);     // Work → Mall: 8 min
    city_map.AddEdge(park, mall, 10.0);    // Park → Mall: 10 min
    
    std::cout << "=== City Map Created ===" << std::endl;
    std::cout << "Locations: " << city_map.GetVertexCount() << std::endl;
    std::cout << "Routes: " << city_map.GetEdgeCount() << std::endl;
    
    // Step 3: Find path using Dijkstra (guaranteed optimal)
    std::cout << "\n=== Dijkstra's Algorithm (Optimal Path) ===" << std::endl;
    
    auto dijkstra_path = Dijkstra::Search(city_map, home, mall);
    
    if (!dijkstra_path.empty()) {
        std::cout << "Shortest path from " << home.name << " to " << mall.name << ":" << std::endl;
        
        double total_time = 0.0;
        for (size_t i = 0; i < dijkstra_path.size(); ++i) {
            std::cout << "  " << (i + 1) << ". " << dijkstra_path[i].name;
            
            // Calculate time for this segment
            if (i < dijkstra_path.size() - 1) {
                double segment_time = city_map.GetEdgeWeight(dijkstra_path[i], dijkstra_path[i + 1]);
                total_time += segment_time;
                std::cout << " --(" << segment_time << " min)--> ";
            }
        }
        std::cout << std::endl << "Total travel time: " << total_time << " minutes" << std::endl;
    } else {
        std::cout << "No path found from " << home.name << " to " << mall.name << std::endl;
    }
    
    // Step 4: Find path using A* with Euclidean heuristic
    std::cout << "\n=== A* Algorithm (Heuristic-Guided) ===" << std::endl;
    
    auto astar_path = AStar::Search(city_map, home, mall, EuclideanDistance);
    
    if (!astar_path.empty()) {
        std::cout << "A* path from " << home.name << " to " << mall.name << ":" << std::endl;
        
        double total_time = 0.0;
        for (size_t i = 0; i < astar_path.size(); ++i) {
            std::cout << "  " << (i + 1) << ". " << astar_path[i].name;
            
            if (i < astar_path.size() - 1) {
                double segment_time = city_map.GetEdgeWeight(astar_path[i], astar_path[i + 1]);
                total_time += segment_time;
                std::cout << " --(" << segment_time << " min)--> ";
            }
        }
        std::cout << std::endl << "Total travel time: " << total_time << " minutes" << std::endl;
    }
    
    // Step 5: Compare different heuristics
    std::cout << "\n=== Comparing Heuristics ===" << std::endl;
    
    auto astar_manhattan = AStar::Search(city_map, home, mall, ManhattanDistance);
    
    std::cout << "Euclidean heuristic path length: " << astar_path.size() << " stops" << std::endl;
    std::cout << "Manhattan heuristic path length: " << astar_manhattan.size() << " stops" << std::endl;
    std::cout << "Dijkstra path length: " << dijkstra_path.size() << " stops" << std::endl;
    
    // Step 6: Find multiple paths from one starting point
    std::cout << "\n=== Multiple Destinations ===" << std::endl;
    
    std::vector<Location> destinations = {work, park, mall};
    for (const auto& destination : destinations) {
        auto path = Dijkstra::Search(city_map, home, destination);
        if (!path.empty()) {
            double total_cost = 0.0;
            for (size_t i = 0; i < path.size() - 1; ++i) {
                total_cost += city_map.GetEdgeWeight(path[i], path[i + 1]);
            }
            std::cout << home.name << " → " << destination.name 
                      << ": " << total_cost << " min (" << path.size() << " stops)" << std::endl;
        }
    }
    
    // Step 7: Handle no-path scenarios
    std::cout << "\n=== Unreachable Destination ===" << std::endl;
    
    // Create an isolated location
    Location island{99, "Island", 50.0, 50.0};
    city_map.AddVertex(island);  // No edges to/from island
    
    auto no_path = Dijkstra::Search(city_map, home, island);
    if (no_path.empty()) {
        std::cout << "Cannot reach " << island.name << " from " << home.name << std::endl;
    }
    
    return 0;
}
```

## Step-by-Step Explanation

### 1. Enhanced State with Coordinates
```cpp
struct Location {
    int id;
    std::string name;
    double x, y;  // For heuristic calculations
};
```

**Why Coordinates?**
- A* algorithm needs heuristic function for guidance
- Coordinates enable distance-based heuristics
- More realistic representation of real-world locations

### 2. Heuristic Functions
```cpp
double EuclideanDistance(const Location& from, const Location& to) {
    double dx = from.x - to.x;
    double dy = from.y - to.y;
    return std::sqrt(dx * dx + dy * dy);
}
```

**Heuristic Properties:**
- Must be **admissible** (never overestimate true cost)
- Better heuristics guide search more efficiently
- Euclidean distance works well for geometric problems

### 3. Dijkstra Algorithm Usage
```cpp
auto path = Dijkstra::Search(city_map, home, mall);
```

**Dijkstra Characteristics:**
- **Guaranteed optimal** shortest path
- Works with **non-negative edge weights**
- **No heuristic needed** - explores systematically
- Time complexity: O((V + E) log V)

### 4. A* Algorithm Usage
```cpp
auto path = AStar::Search(city_map, home, mall, EuclideanDistance);
```

**A* Characteristics:**
- **Optimal** if heuristic is admissible
- **Faster than Dijkstra** with good heuristics
- **Requires heuristic function** as third parameter
- Best for problems with clear "goal direction"

### 5. Path Analysis
```cpp
if (!path.empty()) {
    // Calculate total cost
    double total_time = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        total_time += city_map.GetEdgeWeight(path[i], path[i + 1]);
    }
}
```

**Path Structure:**
- Return type is `std::vector<State>` (sequence of states)
- Empty vector indicates no path exists
- Path includes start and goal states

## Algorithm Comparison

### When to Use Each Algorithm

| **Algorithm** | **Best For** | **Advantages** | **Disadvantages** |
|---------------|--------------|----------------|-------------------|
| **Dijkstra** | Guaranteed optimal paths, multiple destinations | Always finds shortest path, no heuristic needed | Slower, explores more nodes |
| **A\*** | Single destination with good heuristic | Faster with good heuristic, still optimal | Requires admissible heuristic |
| **BFS** | Unweighted graphs, shortest hop count | Simple, optimal for unweighted | Ignores edge weights |
| **DFS** | Reachability testing, any path acceptable | Memory efficient | Not optimal, may find long paths |

### Performance Comparison

```cpp
#include <chrono>

// Timing example
auto start = std::chrono::high_resolution_clock::now();
auto path = Dijkstra::Search(large_graph, start_state, goal_state);
auto end = std::chrono::high_resolution_clock::now();

auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
std::cout << "Search took: " << duration.count() << " microseconds" << std::endl;
```

## Advanced Pathfinding Patterns

### 1. Batch Pathfinding
```cpp
// Find paths to multiple destinations efficiently
std::vector<Location> destinations = {work, gym, mall};
std::map<std::string, Path<Location>> all_paths;

for (const auto& dest : destinations) {
    all_paths[dest.name] = Dijkstra::Search(city_map, home, dest);
}
```

### 2. Bidirectional Search Setup
```cpp
// For very large graphs, consider adding reverse edges
city_map.AddUndirectedEdge(locationA, locationB, travel_time);
// This enables more efficient pathfinding in both directions
```

### 3. Path Validation
```cpp
bool ValidatePath(const Graph<Location>& graph, const Path<Location>& path) {
    if (path.size() < 2) return path.size() == 1;  // Single vertex is valid
    
    for (size_t i = 0; i < path.size() - 1; ++i) {
        if (!graph.HasEdge(path[i], path[i + 1])) {
            return false;  // Missing edge in path
        }
    }
    return true;
}
```

## Running the Example

Compile and run the pathfinding example:

```bash
g++ -std=c++11 -I/path/to/libgraph/include pathfinding_tutorial.cpp -o pathfinding_tutorial
./pathfinding_tutorial
```

**Expected Output (excerpt):**
```
=== City Map Created ===
Locations: 6
Routes: 9

=== Dijkstra's Algorithm (Optimal Path) ===
Shortest path from Home to Mall:
  1. Home --(5.0 min)--> 
  2. Store --(15.0 min)--> 
  3. Work --(8.0 min)--> 
  4. Mall
Total travel time: 28.0 minutes

=== A* Algorithm (Heuristic-Guided) ===
A* path from Home to Mall:
  1. Home --(5.0 min)--> 
  2. Store --(15.0 min)--> 
  3. Work --(8.0 min)--> 
  4. Mall
Total travel time: 28.0 minutes
```

## Practice Exercises

### Exercise 1: Custom Heuristic
Create a heuristic that considers both distance and travel time preferences.

<details>
<summary>Solution</summary>

```cpp
double WeightedHeuristic(const Location& from, const Location& to) {
    double distance = EuclideanDistance(from, to);
    double time_estimate = distance * 0.5;  // Assume 0.5 min per distance unit
    return time_estimate;
}

auto path = AStar::Search(city_map, start, goal, WeightedHeuristic);
```
</details>

### Exercise 2: Path Cost Analysis
Write a function to analyze path costs and compare different routes.

<details>
<summary>Solution</summary>

```cpp
struct PathInfo {
    double total_cost;
    size_t hop_count;
    std::vector<std::string> route_names;
};

PathInfo AnalyzePath(const Graph<Location>& graph, const Path<Location>& path) {
    PathInfo info;
    info.total_cost = 0.0;
    info.hop_count = path.size();
    
    for (size_t i = 0; i < path.size(); ++i) {
        info.route_names.push_back(path[i].name);
        if (i < path.size() - 1) {
            info.total_cost += graph.GetEdgeWeight(path[i], path[i + 1]);
        }
    }
    
    return info;
}
```
</details>

### Exercise 3: Alternative Path Finding
Find the second-shortest path by temporarily removing the shortest path edges.

<details>
<summary>Solution</summary>

```cpp
Path<Location> FindAlternativePath(Graph<Location> graph, 
                                  const Location& start, const Location& goal) {
    // Find optimal path first
    auto optimal = Dijkstra::Search(graph, start, goal);
    if (optimal.size() < 2) return {};
    
    // Try removing each edge in optimal path and find best alternative
    Path<Location> best_alternative;
    double best_cost = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < optimal.size() - 1; ++i) {
        // Temporarily remove edge
        double original_weight = graph.GetEdgeWeight(optimal[i], optimal[i + 1]);
        graph.RemoveEdge(optimal[i], optimal[i + 1]);
        
        // Find alternative path
        auto alt_path = Dijkstra::Search(graph, start, goal);
        if (!alt_path.empty()) {
            // Calculate cost and keep if better
            double cost = CalculatePathCost(graph, alt_path);
            if (cost < best_cost) {
                best_cost = cost;
                best_alternative = alt_path;
            }
        }
        
        // Restore edge
        graph.AddEdge(optimal[i], optimal[i + 1], original_weight);
    }
    
    return best_alternative;
}
```
</details>

## Common Pitfalls

### **Non-Admissible Heuristics**
```cpp
// BAD: Heuristic that overestimates (not admissible)
double BadHeuristic(const Location& from, const Location& to) {
    return EuclideanDistance(from, to) * 2.0;  // Overestimates!
}
// This breaks A*'s optimality guarantee
```

### **Ignoring Empty Paths**
```cpp
// BAD: Not checking for empty path
auto path = Dijkstra::Search(graph, start, goal);
double cost = CalculatePathCost(graph, path);  // Crashes if path is empty!

// GOOD: Always check path validity
if (!path.empty()) {
    double cost = CalculatePathCost(graph, path);
}
```

### **Wrong Algorithm Choice**
```cpp
// BAD: Using A* without good heuristic
auto path = AStar::Search(graph, start, goal, [](const State&, const State&) { 
    return 0.0;  // Zero heuristic = Dijkstra but slower
});

// GOOD: Use Dijkstra when no good heuristic exists
auto path = Dijkstra::Search(graph, start, goal);
```

## Key Concepts

### **Algorithm Selection**
- **Dijkstra**: When you need guaranteed optimal paths
- **A\***: When you have good heuristics and need speed
- **Consider graph size and structure** when choosing

### **Heuristic Quality**
- **Admissible**: Never overestimate true cost
- **Consistent**: h(n) ≤ cost(n,n') + h(n') for neighbors
- **Better heuristics** → faster A* search

### **Path Representation**
- Returned as `std::vector<State>`
- Empty vector means no path exists
- Always includes start and goal states

---

## Next Steps

Excellent! You now understand the core pathfinding algorithms in libgraph. In **[Tutorial 3: Working with Different State Types](03-state-types.md)**, you'll learn how to use libgraph with various state types and custom indexing strategies.

### Preview
```cpp
// Coming up in Tutorial 3:
struct GameCharacter {
    std::string name;
    int health, mana;
    Position pos;
    
    // Custom ID generation
    int64_t GetId() const { return std::hash<std::string>{}(name); }
};

Graph<GameCharacter> game_world;
```