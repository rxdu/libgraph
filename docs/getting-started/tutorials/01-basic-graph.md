# Tutorial 1: Basic Graph Operations

**Learning Objectives:** Create your first graph, add vertices and edges, and understand core libgraph concepts.

**Estimated Time:** 15 minutes

---

## Overview

In this tutorial, you'll learn the fundamental operations for building and manipulating graphs with libgraph. We'll create a simple transportation network and explore the basic API.

## Complete Example

Let's build a simple city transportation network:

```cpp
#include "graph/graph.hpp"
#include <iostream>
#include <vector>

using namespace xmotion;

// Define our state type - represents locations in the city
struct Location {
    int id;
    std::string name;
    
    // Constructor for convenience
    Location(int i, const std::string& n) : id(i), name(n) {}
};

int main() {
    // Step 1: Create the graph
    Graph<Location> city_map;
    
    // Step 2: Add vertices (locations in our city)
    Location home{0, "Home"};
    Location work{1, "Work"};
    Location gym{2, "Gym"};
    Location store{3, "Store"};
    Location park{4, "Park"};
    
    city_map.AddVertex(home);
    city_map.AddVertex(work);
    city_map.AddVertex(gym);
    city_map.AddVertex(store);
    city_map.AddVertex(park);
    
    // Step 3: Connect locations with weighted edges (distance in km)
    city_map.AddEdge(home, work, 5.2);    // Home to Work: 5.2km
    city_map.AddEdge(home, gym, 2.8);     // Home to Gym: 2.8km
    city_map.AddEdge(home, store, 1.5);   // Home to Store: 1.5km
    city_map.AddEdge(work, gym, 3.1);     // Work to Gym: 3.1km
    city_map.AddEdge(gym, park, 1.2);     // Gym to Park: 1.2km
    city_map.AddEdge(store, park, 2.0);   // Store to Park: 2.0km
    
    // Step 4: Explore the graph we created
    std::cout << "=== City Transportation Network ===" << std::endl;
    std::cout << "Total locations: " << city_map.GetVertexCount() << std::endl;
    std::cout << "Total connections: " << city_map.GetEdgeCount() << std::endl;
    
    // Step 5: Check connectivity and distances
    std::cout << "\n=== Connectivity Check ===" << std::endl;
    std::cout << "Can go from Home to Work? " << (city_map.HasEdge(home, work) ? "Yes" : "No") << std::endl;
    std::cout << "Distance from Home to Work: " << city_map.GetEdgeWeight(home, work) << " km" << std::endl;
    std::cout << "Can go from Work to Home? " << (city_map.HasEdge(work, home) ? "Yes" : "No") << std::endl;
    
    // Step 6: Explore neighbors
    auto home_neighbors = city_map.GetNeighbors(home);
    std::cout << "\nPlaces reachable from Home:" << std::endl;
    for (const auto& neighbor : home_neighbors) {
        std::cout << "  -> " << neighbor.name << " (ID: " << neighbor.id << ")" << std::endl;
    }
    
    // Step 7: Use iterators to examine all vertices
    std::cout << "\n=== All Locations ===" << std::endl;
    for (auto it = city_map.vertex_begin(); it != city_map.vertex_end(); ++it) {
        const auto& location = it->state;
        size_t out_degree = city_map.GetOutDegree(location.id);
        std::cout << location.name << " (ID: " << location.id 
                  << ", outgoing connections: " << out_degree << ")" << std::endl;
    }
    
    // Step 8: Range-based for loop (modern C++)
    std::cout << "\n=== Using Range-Based For Loop ===" << std::endl;
    for (const auto& vertex : city_map.vertices()) {
        std::cout << "Location: " << vertex.state.name << std::endl;
    }
    
    return 0;
}
```

## Step-by-Step Explanation

### 1. State Definition
```cpp
struct Location {
    int id;
    std::string name;
    Location(int i, const std::string& n) : id(i), name(n) {}
};
```

**Key Points:**
- The `id` field is automatically detected by `DefaultIndexer`
- States can contain any data you need (coordinates, properties, etc.)
- States must be copyable for graph operations

### 2. Graph Creation
```cpp
Graph<Location> city_map;
```

**Template Parameters:**
- `Location`: Our vertex state type
- `double`: Edge weight type (default)
- `DefaultIndexer<Location>`: State indexing (default, uses `id` field)

### 3. Adding Vertices
```cpp
city_map.AddVertex(home);
```

**Important Notes:**
- Each vertex gets a unique internal ID based on your state's ID
- Duplicate states (same ID) will reuse the existing vertex
- Adding vertices is O(1) average time complexity

### 4. Adding Edges  
```cpp
city_map.AddEdge(home, work, 5.2);
```

**Edge Behavior:**
- Creates directed edge from `home` to `work` with weight `5.2`
- If edge already exists, updates the weight
- Automatically creates vertices if they don't exist

### 5. Graph Queries
```cpp
bool connected = city_map.HasEdge(home, work);
double distance = city_map.GetEdgeWeight(home, work);
auto neighbors = city_map.GetNeighbors(home);
```

**Query Methods:**
- `HasEdge()`: Check if direct connection exists
- `GetEdgeWeight()`: Get edge weight (returns default value if no edge)
- `GetNeighbors()`: Get all directly reachable states

### 6. Graph Statistics
```cpp
size_t vertex_count = city_map.GetVertexCount();
size_t edge_count = city_map.GetEdgeCount();
size_t out_degree = city_map.GetOutDegree(location.id);
```

**Statistics Available:**
- Vertex/edge counts for the entire graph
- Degree information per vertex (in-degree, out-degree, total degree)
- Empty check with `city_map.empty()`

### 7. Iteration Patterns
```cpp
// Traditional iterators
for (auto it = city_map.vertex_begin(); it != city_map.vertex_end(); ++it) {
    const Location& loc = it->state;
}

// Range-based for loop (recommended)
for (const auto& vertex : city_map.vertices()) {
    const Location& loc = vertex.state;
}
```

## Key Concepts

### **State Indexing**
- Every state needs a unique identifier for graph operations
- `DefaultIndexer` automatically uses `id`, `id_`, or `GetId()` method
- Custom indexers can be created for complex state types

### **Directed vs Undirected**
- `AddEdge(A, B, weight)` creates A → B (directed)
- `AddUndirectedEdge(A, B, weight)` creates A ↔ B (bidirectional)
- Most real-world scenarios need directed edges with selective undirected connections

### **Memory Management**
- Graph automatically manages vertex/edge memory using RAII
- No manual cleanup required
- Copy/move semantics work as expected

### **Template Flexibility**
- `State` can be any copyable type
- `Transition` (edge weight) can be numeric or custom type
- Type safety prevents mixing incompatible graphs

## Running the Example

Save the code as `basic_graph_tutorial.cpp` and compile:

```bash
# Assuming libgraph is in your include path
g++ -std=c++11 -I/path/to/libgraph/include basic_graph_tutorial.cpp -o basic_graph_tutorial

# Run the program
./basic_graph_tutorial
```

**Expected Output:**
```
=== City Transportation Network ===
Total locations: 5
Total connections: 6

=== Connectivity Check ===
Can go from Home to Work? Yes
Distance from Home to Work: 5.2 km
Can go from Work to Home? No

Places reachable from Home:
  -> Work (ID: 1)
  -> Gym (ID: 2)
  -> Store (ID: 3)

=== All Locations ===
Home (ID: 0, outgoing connections: 3)
Work (ID: 1, outgoing connections: 1)
Gym (ID: 2, outgoing connections: 1)
Store (ID: 3, outgoing connections: 1)
Park (ID: 4, outgoing connections: 0)

=== Using Range-Based For Loop ===
Location: Home
Location: Work
Location: Gym
Location: Store
Location: Park
```

## Practice Exercises

### Exercise 1: Bidirectional Connections
Modify the code to make some connections bidirectional (like between Home and Store for a round trip).

<details>
<summary>Solution</summary>

```cpp
// Replace single direction with bidirectional
city_map.AddUndirectedEdge(home, store, 1.5);   // Both directions
city_map.AddUndirectedEdge(gym, park, 1.2);     // Both directions
```
</details>

### Exercise 2: Custom State Type
Create a graph using a different state type, like `struct Person { int id; std::string name; int age; };`

<details>
<summary>Solution</summary>

```cpp
struct Person {
    int id;
    std::string name;
    int age;
    
    Person(int i, const std::string& n, int a) : id(i), name(n), age(a) {}
};

Graph<Person> social_network;
social_network.AddVertex(Person{1, "Alice", 25});
social_network.AddVertex(Person{2, "Bob", 30});
social_network.AddEdge(Person{1, "Alice", 25}, Person{2, "Bob", 30}, 1.0);  // friendship strength
```
</details>

### Exercise 3: Graph Validation
Add error checking to verify vertices exist before adding edges.

<details>
<summary>Solution</summary>

```cpp
// Check if vertex exists before adding edge
if (city_map.HasVertex(home.id) && city_map.HasVertex(work.id)) {
    city_map.AddEdge(home, work, 5.2);
} else {
    std::cout << "Warning: One or both vertices don't exist!" << std::endl;
}
```
</details>

## Common Pitfalls

### **Inconsistent State IDs**
```cpp
Location loc1{1, "Place"};
Location loc2{1, "Different Place"};  // Same ID!
graph.AddVertex(loc1);
graph.AddVertex(loc2);  // Will overwrite loc1
```

### **Forgetting Edge Direction**
```cpp
graph.AddEdge(A, B, 5.0);  // A → B
// This does NOT create B → A automatically
bool exists = graph.HasEdge(B, A);  // False!
```

### **Best Practices**
- Use meaningful, unique IDs for states
- Be explicit about edge directionality
- Check return values for operations that can fail
- Use const references when iterating to avoid copies

---

## Next Steps

Great job! You've learned the fundamentals of graph construction and basic operations. In **[Tutorial 2: Simple Pathfinding](02-pathfinding.md)**, you'll learn how to find optimal paths through your graphs using Dijkstra's algorithm and A*.

### Preview
```cpp
// Coming up in Tutorial 2:
auto path = Dijkstra::Search(city_map, home, park);
for (const auto& location : path) {
    std::cout << "→ " << location.name << std::endl;
}
```