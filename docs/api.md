# API Reference

## Graph Template Classes

libgraph provides a comprehensive C++11 header-only library for graph construction and pathfinding algorithms. The library is built around template classes that provide type safety and flexibility for different application domains.

### Core Template Parameters

All main classes use consistent template parameters:

```cpp
template<typename State, typename Transition = double, typename StateIndexer = DefaultIndexer<State>>
```

- **State**: The data type stored in graph vertices (your application domain objects)
- **Transition**: The data type for edge weights/costs (defaults to `double`)  
- **StateIndexer**: Functor to generate unique IDs from states (auto-detects `id`, `id_`, or `GetId()`)

---

## Graph Class

The main graph container using adjacency list representation.

### Template Declaration

```cpp
template<typename State, typename Transition = double, typename StateIndexer = DefaultIndexer<State>>
class Graph;
```

### Type Aliases

```cpp
using Edge = xmotion::Edge<State, Transition, StateIndexer>;
using Vertex = xmotion::Vertex<State, Transition, StateIndexer>; 
using GraphType = Graph<State, Transition, StateIndexer>;
```

### Big Five (Constructors and Assignment)

```cpp
// Default constructor (no-throw guarantee)
Graph() = default;

// Copy constructor (strong exception guarantee)  
Graph(const GraphType& other);

// Move constructor (no-throw guarantee)
Graph(GraphType&& other) noexcept;

// Copy assignment operator (strong guarantee via copy-and-swap)
GraphType& operator=(const GraphType& other);

// Move assignment operator (no-throw guarantee)
GraphType& operator=(GraphType&& other) noexcept;

// Destructor (automatic cleanup via RAII)
~Graph();

// Efficient swapping for assignment operations
void swap(GraphType& other) noexcept;
```

### Vertex Operations

#### Core Vertex Management

```cpp
// Add a new vertex to the graph
vertex_iterator AddVertex(State state);

// Remove vertex by ID or state
void RemoveVertex(int64_t state_id);
template<class T = State, typename std::enable_if<!std::is_integral<T>::value>::type* = nullptr>
void RemoveVertex(T state);

// Find vertex by ID or state (returns end() if not found)
vertex_iterator FindVertex(int64_t vertex_id);
const_vertex_iterator FindVertex(int64_t vertex_id) const;
template<class T = State, typename std::enable_if<!std::is_integral<T>::value>::type* = nullptr>
vertex_iterator FindVertex(T state);
template<class T = State, typename std::enable_if<!std::is_integral<T>::value>::type* = nullptr>
const_vertex_iterator FindVertex(T state) const;
```

#### Vertex Query Methods  

```cpp
// Check if vertex exists
bool HasVertex(int64_t vertex_id) const;
template<class T = State, typename std::enable_if<!std::is_integral<T>::value>::type* = nullptr>
bool HasVertex(T state) const;

// Get vertex pointers (returns nullptr if not found)
Vertex* GetVertex(int64_t vertex_id);
const Vertex* GetVertex(int64_t vertex_id) const;
template<class T = State, typename std::enable_if<!std::is_integral<T>::value>::type* = nullptr>
Vertex* GetVertex(T state);
template<class T = State, typename std::enable_if<!std::is_integral<T>::value>::type* = nullptr>
const Vertex* GetVertex(T state) const;

// Safe vertex access (throws ElementNotFoundError if not found)
Vertex& GetVertexSafe(int64_t vertex_id);
const Vertex& GetVertexSafe(int64_t vertex_id) const;

// Vertex degree information
size_t GetVertexDegree(int64_t vertex_id) const;    // in-degree + out-degree
size_t GetInDegree(int64_t vertex_id) const;        // incoming edges
size_t GetOutDegree(int64_t vertex_id) const;       // outgoing edges

// Get neighbor states
std::vector<State> GetNeighbors(State state) const;
std::vector<State> GetNeighbors(int64_t vertex_id) const;
```

### Edge Operations

#### Core Edge Management

```cpp
// Add directed edge (updates weight if edge exists)
void AddEdge(State sstate, State dstate, Transition trans);

// Remove directed edge
bool RemoveEdge(State sstate, State dstate);

// Add/remove undirected edges (bidirectional)
void AddUndirectedEdge(State sstate, State dstate, Transition trans);
bool RemoveUndirectedEdge(State sstate, State dstate);

// Get all edges in the graph
std::vector<edge_iterator> GetAllEdges() const;
```

#### Edge Query Methods

```cpp
// Check if edge exists
bool HasEdge(State from, State to) const;

// Get edge weight (returns Transition{} if edge doesn't exist)
Transition GetEdgeWeight(State from, State to) const;
```

### Graph Information and Statistics

```cpp
// Graph size information  
int64_t GetTotalVertexNumber() const noexcept;
int64_t GetTotalEdgeNumber() const;
size_t GetVertexCount() const noexcept;
size_t GetEdgeCount() const noexcept;

// STL-like interface
bool empty() const noexcept;
size_t size() const noexcept;
void reserve(size_t n);
```

### Batch Operations

```cpp
// Add multiple vertices/edges at once
void AddVertices(const std::vector<State>& states);
void AddEdges(const std::vector<std::tuple<State, State, Transition>>& edges);
void RemoveVertices(const std::vector<State>& states);

// Operations with result reporting (std::map-like interface)
std::pair<vertex_iterator, bool> AddVertexWithResult(State state);
bool AddEdgeWithResult(State from, State to, Transition trans);
bool AddUndirectedEdgeWithResult(State from, State to, Transition trans);
bool RemoveVertexWithResult(int64_t vertex_id);
template<class T = State, typename std::enable_if<!std::is_integral<T>::value>::type* = nullptr>
bool RemoveVertexWithResult(T state);
```

### Graph Validation and Maintenance

```cpp
// Reset all vertex states for new search
void ResetAllVertices();

// Clear entire graph
void ClearAll();

// Structure validation (throws DataCorruptionError if issues found)
void ValidateStructure() const;

// Edge weight validation (throws InvalidArgumentError for invalid weights)
void ValidateEdgeWeight(Transition weight) const;
```

### Iterator Support

#### Vertex Iterators

```cpp
// Iterator types
class vertex_iterator;           // Mutable vertex access
class const_vertex_iterator;     // Read-only vertex access

// Iterator access methods
vertex_iterator vertex_begin();
vertex_iterator vertex_end();
const_vertex_iterator vertex_begin() const;
const_vertex_iterator vertex_end() const;
const_vertex_iterator vertex_cbegin() const;    // C++11 const iterators
const_vertex_iterator vertex_cend() const;

// Range-based for loop support
class vertex_range;
class const_vertex_range;
vertex_range vertices();
const_vertex_range vertices() const;
```

#### Edge Iterators

```cpp
// Edge iterator types (from Vertex class)
using edge_iterator = typename Vertex::edge_iterator;
using const_edge_iterator = typename Vertex::const_edge_iterator;
```

---

## Vertex Class

Independent vertex class storing state and edge information.

### Template Declaration

```cpp
template<typename State, typename Transition, typename StateIndexer>
struct Vertex;
```

### Core Members

```cpp
// Vertex data
State state;                    // User-defined state object
const int64_t vertex_id;       // Unique vertex identifier
StateIndexer GetStateIndex;    // Indexer functor instance

// Edge storage
EdgeListType edges_to;                           // Outgoing edges
std::list<vertex_iterator> vertices_from;       // Incoming edge sources
```

### Constructor and Lifecycle

```cpp
// Constructor (only way to create vertices)
Vertex(State s, int64_t id);

// Big Five (all other operations disabled for memory safety)
~Vertex() = default;
Vertex() = delete;
Vertex(const Vertex& other) = delete;
Vertex& operator=(const Vertex& other) = delete;
Vertex(Vertex&& other) = delete;
Vertex& operator=(Vertex&& other) = delete;
```

### Edge Access Methods

```cpp
// Edge iterators
edge_iterator edge_begin() noexcept;
edge_iterator edge_end() noexcept;
const_edge_iterator edge_begin() const noexcept;
const_edge_iterator edge_end() const noexcept;
```

### Comparison and Identification

```cpp
// Vertex comparison
bool operator==(const Vertex& other) const;

// Vertex ID access
int64_t GetId() const;
```

### Legacy Search Fields (Deprecated)

```cpp
// These fields are deprecated - use SearchContext for thread-safe searches
[[deprecated("Use SearchContext for thread-safe searches")]] 
bool is_checked = false;
[[deprecated("Use SearchContext for thread-safe searches")]]
bool is_in_openlist = false;
[[deprecated("Use SearchContext for thread-safe searches")]]
double f_cost = std::numeric_limits<double>::max();
[[deprecated("Use SearchContext for thread-safe searches")]]
double g_cost = std::numeric_limits<double>::max();
[[deprecated("Use SearchContext for thread-safe searches")]]
double h_cost = std::numeric_limits<double>::max();
[[deprecated("Use SearchContext for thread-safe searches")]]
vertex_iterator search_parent;
```

---

## Edge Class

Independent edge class connecting vertices.

### Template Declaration

```cpp
template<typename State, typename Transition, typename StateIndexer>
struct Edge;
```

### Core Members

```cpp
Vertex* dst;                    // Destination vertex pointer
Transition cost;                // Edge weight/cost
```

### Constructor

```cpp
Edge(Vertex* destination, Transition edge_cost);
```

### Comparison Operations

```cpp
bool operator==(const Edge& other) const;
bool operator!=(const Edge& other) const;
```

---

## Search Algorithms

### SearchContext (Thread-Safe Search State)

Thread-safe container for search algorithm state, enabling concurrent searches on the same graph.

#### Template Declaration

```cpp
template<typename State, typename Transition = double, typename StateIndexer = DefaultIndexer<State>>
class SearchContext;
```

#### Core Methods

```cpp
// Constructor
SearchContext();

// Search state management
void Reset();                                    // Clear all search state
void PreAllocate(size_t expected_vertices);      // Pre-allocate for performance

// Search information access (internal use by algorithms)
SearchVertexInfo& GetVertexInfo(int64_t vertex_id);
const SearchVertexInfo& GetVertexInfo(int64_t vertex_id) const;
bool HasVertexInfo(int64_t vertex_id) const;
```

#### Usage Pattern

```cpp
SearchContext<State, Transition, StateIndexer> context;
auto path = Dijkstra::Search(graph, context, start, goal);
```

### CostTraits (Custom Cost Type Support)

Template specialization system for custom cost types.

#### Default Implementation

```cpp
template<typename T>
struct CostTraits {
    static T infinity();  // Returns std::numeric_limits<T>::max() for arithmetic types
};
```

#### Custom Specialization

```cpp
// For non-arithmetic cost types, specialize CostTraits
template<>
struct CostTraits<MyCustomCost> {
    static MyCustomCost infinity() { 
        return MyCustomCost::max(); 
    }
};
```

### Dijkstra Algorithm

Optimal shortest path algorithm for graphs with non-negative edge weights.

#### Static Interface

```cpp
class Dijkstra {
public:
    // Basic search (uses internal state, not thread-safe)
    template<typename State, typename Transition, typename StateIndexer>
    static Path<State> Search(const Graph<State, Transition, StateIndexer>& graph,
                             State start_state, State goal_state);
    
    // Thread-safe search using external context
    template<typename State, typename Transition, typename StateIndexer>
    static Path<State> Search(const Graph<State, Transition, StateIndexer>& graph,
                             SearchContext<State, Transition, StateIndexer>& context,
                             State start_state, State goal_state);
    
    // Custom cost comparator support
    template<typename State, typename Transition, typename StateIndexer, typename TransitionComparator>
    static Path<State> Search(const Graph<State, Transition, StateIndexer>& graph,
                             SearchContext<State, Transition, StateIndexer>& context,
                             State start_state, State goal_state,
                             const TransitionComparator& comp);
};
```

#### Usage Examples

```cpp
// Basic usage
auto path = Dijkstra::Search(graph, start, goal);

// Thread-safe usage
SearchContext<State> context;
auto path = Dijkstra::Search(graph, context, start, goal);

// Custom cost comparator
auto path = Dijkstra::Search(graph, context, start, goal, std::greater<MyCost>());
```

### A* Algorithm

Optimal shortest path algorithm using heuristic guidance for faster search.

#### Static Interface

```cpp
class AStar {
public:
    // Basic search with heuristic
    template<typename State, typename Transition, typename StateIndexer, typename HeuristicFunc>
    static Path<State> Search(const Graph<State, Transition, StateIndexer>& graph,
                             State start_state, State goal_state, 
                             HeuristicFunc heuristic);
    
    // Thread-safe search
    template<typename State, typename Transition, typename StateIndexer, typename HeuristicFunc>
    static Path<State> Search(const Graph<State, Transition, StateIndexer>& graph,
                             SearchContext<State, Transition, StateIndexer>& context,
                             State start_state, State goal_state, 
                             HeuristicFunc heuristic);
    
    // Custom cost comparator support
    template<typename State, typename Transition, typename StateIndexer, typename HeuristicFunc, typename TransitionComparator>
    static Path<State> Search(const Graph<State, Transition, StateIndexer>& graph,
                             SearchContext<State, Transition, StateIndexer>& context,
                             State start_state, State goal_state, 
                             HeuristicFunc heuristic,
                             const TransitionComparator& comp);
};
```

#### Heuristic Function Requirements

```cpp
// Heuristic function signature
Transition heuristic(const State& from, const State& to);

// Example: Manhattan distance for 2D grid
double ManhattanDistance(const GridCell& from, const GridCell& to) {
    return std::abs(from.x - to.x) + std::abs(from.y - to.y);
}
```

#### Usage Examples

```cpp
// Basic usage
auto path = AStar::Search(graph, start, goal, ManhattanDistance);

// Thread-safe usage  
SearchContext<State> context;
auto path = AStar::Search(graph, context, start, goal, ManhattanDistance);
```

### BFS (Breadth-First Search)

Unweighted shortest path algorithm, optimal for graphs where all edges have equal cost.

#### Static Interface

```cpp
class BFS {
public:
    // Basic search
    template<typename State, typename Transition, typename StateIndexer>
    static Path<State> Search(const Graph<State, Transition, StateIndexer>& graph,
                             State start_state, State goal_state);
    
    // Thread-safe search
    template<typename State, typename Transition, typename StateIndexer>
    static Path<State> Search(const Graph<State, Transition, StateIndexer>& graph,
                             SearchContext<State, Transition, StateIndexer>& context,
                             State start_state, State goal_state);
};
```

### DFS (Depth-First Search)

Graph traversal algorithm for reachability testing and path finding (not necessarily optimal).

#### Static Interface

```cpp
class DFS {
public:
    // Basic search
    template<typename State, typename Transition, typename StateIndexer>
    static Path<State> Search(const Graph<State, Transition, StateIndexer>& graph,
                             State start_state, State goal_state);
    
    // Thread-safe search
    template<typename State, typename Transition, typename StateIndexer>
    static Path<State> Search(const Graph<State, Transition, StateIndexer>& graph,
                             SearchContext<State, Transition, StateIndexer>& context,
                             State start_state, State goal_state);
};
```

---

## DefaultIndexer

Automatic state indexing that works with common patterns.

### Template Declaration

```cpp
template<typename State>
struct DefaultIndexer;
```

### Supported State Patterns

The DefaultIndexer automatically detects and works with:

1. **Member variable `id`**:
   ```cpp
   struct MyState { 
       int64_t id; 
   };
   ```

2. **Member variable `id_`**:
   ```cpp
   struct MyState { 
       int64_t id_; 
   };
   ```

3. **Member function `GetId()`**:
   ```cpp
   struct MyState { 
       int64_t GetId() const { return some_unique_value; } 
   };
   ```

### Custom Indexer

For states that don't match the default patterns:

```cpp
struct MyCustomIndexer {
    int64_t operator()(const MyState& state) const {
        return state.custom_unique_field;
    }
};

// Usage
Graph<MyState, double, MyCustomIndexer> graph;
```

---

## Exception System

Comprehensive exception hierarchy for error handling.

### Exception Types

```cpp
// Base exception class
class GraphException : public std::exception;

// Specific exception types
class InvalidArgumentError : public GraphException;     // Invalid parameters
class ElementNotFoundError : public GraphException;     // Missing vertices/edges
class DataCorruptionError : public GraphException;      // Graph structure corruption
class AlgorithmError : public GraphException;          // Search algorithm failures
```

### Usage Examples

```cpp
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
```

---

## Type Aliases and Utilities

### Common Type Aliases

```cpp
// Path result type
template<typename State>
using Path = std::vector<State>;

// Convenient graph alias
template<typename State, typename Transition = double, typename StateIndexer = DefaultIndexer<State>>
using Graph_t = Graph<State, Transition, StateIndexer>;
```

### Performance Optimization Utilities

```cpp
// Pre-allocate graph capacity for better performance
graph.reserve(expected_vertex_count);

// Pre-allocate search context for repeated searches
context.PreAllocate(expected_vertex_count);

// Batch operations for efficiency
graph.AddVertices(state_vector);
graph.AddEdges(edge_tuple_vector);
```

---

## Complexity Analysis

### Graph Operations

| Operation | Time Complexity | Space Complexity |
|-----------|----------------|------------------|
| Add Vertex | O(1) average, O(n) worst | O(1) |
| Remove Vertex | O(m²) worst case* | O(1) |
| Find Vertex | O(1) average, O(n) worst | O(1) |
| Add Edge | O(1) | O(1) |
| Remove Edge | O(m) per vertex | O(1) |
| Find Edge | O(m) per vertex | O(1) |

*\* Worst case vertex removal is O(m²) due to updating all incoming edge references*

### Search Algorithms

| Algorithm | Time Complexity | Space Complexity |
|-----------|----------------|------------------|
| **Dijkstra** | O((m+n) log n) | O(n) |
| **A\*** | O((m+n) log n)* | O(n) |
| **BFS** | O(m+n) | O(n) |
| **DFS** | O(m+n) | O(n) |

*\* A* best case depends on heuristic quality*

### Memory Layout

- **Graph**: O(m+n) space using adjacency lists
- **SearchContext**: O(n) space for vertex search information  
- **Priority Queues**: O(n) space for open/closed sets

---

## Thread Safety Guarantees

### Thread-Safe Operations

- **Multiple concurrent searches** using separate `SearchContext` instances
- **Read-only graph queries** (vertex/edge lookup, graph statistics)
- **Graph structure validation** and integrity checking

### Non-Thread-Safe Operations  

- **Graph modifications** (adding/removing vertices or edges)
- **Legacy search methods** without `SearchContext` parameter
- **Vertex state modifications** during concurrent access

### Recommended Usage Pattern

```cpp
// Create graph and populate (single-threaded)
Graph<State> graph;
// ... add vertices and edges ...

// Multiple concurrent searches (thread-safe)
void worker_thread(int thread_id) {
    SearchContext<State> context;  // Each thread gets own context
    auto path = Dijkstra::Search(graph, context, start, goal);
    // Process path...
}
```

---

This API reference covers all major classes and methods in libgraph. For working examples and tutorials, see the [Getting Started Guide](getting-started.md) and [Advanced Features Guide](advanced-features.md).