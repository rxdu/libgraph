# API Reference

## Overview

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

## Result Types

### Path

Simple path result as a vector of states.

```cpp
template <typename State>
using Path = std::vector<State>;
```

### PathResult

Rich search result with path, cost, and diagnostics.

```cpp
template <typename State, typename Cost = double>
struct PathResult {
    Path<State> path;           // Sequence of states from start to goal
    Cost total_cost{};          // Total accumulated cost of the path
    size_t nodes_expanded{0};   // Number of vertices expanded during search
    bool found{false};          // True if a valid path was found

    explicit operator bool() const noexcept;  // Implicit conversion to bool
    bool empty() const noexcept;              // Check if path is empty
    size_t size() const noexcept;             // Get path length
};
```

**Usage:**
```cpp
auto result = Dijkstra::SearchWithResult(graph, context, start, goal, strategy);
if (result) {
    std::cout << "Found path with cost: " << result.total_cost << "\n";
    std::cout << "Nodes expanded: " << result.nodes_expanded << "\n";
}
```

### MultiGoalResult

Extended result for multi-goal search, identifying which goal was reached.

```cpp
template <typename State, typename Cost = double>
struct MultiGoalResult {
    Path<State> path;           // Sequence of states from start to reached goal
    Cost total_cost{};          // Total accumulated cost of the path
    size_t nodes_expanded{0};   // Number of vertices expanded during search
    bool found{false};          // True if any goal was reached
    int64_t goal_vertex_id{-1}; // Vertex ID of the reached goal (-1 if not found)
    size_t goal_index{0};       // Index in the goals vector (valid only if found)

    explicit operator bool() const noexcept;
    bool empty() const noexcept;
    size_t size() const noexcept;
};
```

**Usage:**
```cpp
std::vector<Location> goals = {station1, station2, station3};
auto result = Dijkstra::SearchMultiGoal(graph, context, start, goals);
if (result) {
    std::cout << "Reached goal index: " << result.goal_index << "\n";
    std::cout << "Path cost: " << result.total_cost << "\n";
}
```

### SearchLimits

Early termination configuration for real-time systems.

```cpp
struct SearchLimits {
    size_t max_expansions;      // Maximum node expansions (0 = unlimited)
    size_t timeout_ms;          // Maximum search duration in ms (0 = unlimited)

    SearchLimits();                                    // Default: unlimited
    SearchLimits(size_t max_exp, size_t timeout);      // Explicit limits
    bool HasLimits() const noexcept;                   // Check if any limits set

    // Factory methods
    static SearchLimits Unlimited() noexcept;
    static SearchLimits MaxExpansions(size_t n) noexcept;
    static SearchLimits Timeout(size_t ms) noexcept;
};
```

**Usage:**
```cpp
// Limit by expansions
auto limits = SearchLimits::MaxExpansions(1000);

// Limit by time
auto limits = SearchLimits::Timeout(50);  // 50ms max

// Combine limits
auto limits = SearchLimits(1000, 50);  // 1000 expansions OR 50ms
```

### CostTraits

Template specialization system for custom cost types.

```cpp
template<typename T>
struct CostTraits {
    static T infinity();  // Returns max value for type T
};
```

**Custom Specialization:**
```cpp
template<>
struct CostTraits<MyCustomCost> {
    static MyCustomCost infinity() {
        return MyCustomCost::max();
    }
};
```

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

### Constructors and Assignment

```cpp
Graph() = default;                              // Default constructor
Graph(const GraphType& other);                  // Copy constructor
Graph(GraphType&& other) noexcept;              // Move constructor
GraphType& operator=(const GraphType& other);   // Copy assignment
GraphType& operator=(GraphType&& other) noexcept; // Move assignment
~Graph();                                       // Destructor
void swap(GraphType& other) noexcept;           // Efficient swapping
```

### Vertex Operations

```cpp
// Add and remove
vertex_iterator AddVertex(State state);
void RemoveVertex(int64_t state_id);
void RemoveVertex(State state);

// Find vertices
vertex_iterator FindVertex(int64_t vertex_id);
vertex_iterator FindVertex(State state);
const_vertex_iterator FindVertex(int64_t vertex_id) const;
const_vertex_iterator FindVertex(State state) const;

// Check existence
bool HasVertex(int64_t vertex_id) const;
bool HasVertex(State state) const;

// Get vertex pointers (returns nullptr if not found)
Vertex* GetVertex(int64_t vertex_id);
Vertex* GetVertex(State state);
const Vertex* GetVertex(int64_t vertex_id) const;
const Vertex* GetVertex(State state) const;

// Safe access (throws ElementNotFoundError if not found)
Vertex& GetVertexSafe(int64_t vertex_id);
const Vertex& GetVertexSafe(int64_t vertex_id) const;

// Degree information
size_t GetVertexDegree(int64_t vertex_id) const;  // in + out degree
size_t GetInDegree(int64_t vertex_id) const;
size_t GetOutDegree(int64_t vertex_id) const;

// Get neighbors
std::vector<State> GetNeighbors(State state) const;
std::vector<State> GetNeighbors(int64_t vertex_id) const;
```

### Edge Operations

```cpp
// Add and remove directed edges
void AddEdge(State sstate, State dstate, Transition trans);
bool RemoveEdge(State sstate, State dstate);

// Add and remove undirected edges (bidirectional)
void AddUndirectedEdge(State sstate, State dstate, Transition trans);
bool RemoveUndirectedEdge(State sstate, State dstate);

// Query edges
bool HasEdge(State from, State to) const;
Transition GetEdgeWeight(State from, State to) const;
std::vector<edge_iterator> GetAllEdges() const;
```

### Graph Statistics

```cpp
int64_t GetTotalVertexNumber() const noexcept;
int64_t GetTotalEdgeNumber() const;
size_t GetVertexCount() const noexcept;
size_t GetEdgeCount() const noexcept;
bool empty() const noexcept;
size_t size() const noexcept;
void reserve(size_t n);
```

### Batch Operations

```cpp
void AddVertices(const std::vector<State>& states);
void AddEdges(const std::vector<std::tuple<State, State, Transition>>& edges);
void RemoveVertices(const std::vector<State>& states);

// Operations with result reporting
std::pair<vertex_iterator, bool> AddVertexWithResult(State state);
bool AddEdgeWithResult(State from, State to, Transition trans);
bool AddUndirectedEdgeWithResult(State from, State to, Transition trans);
bool RemoveVertexWithResult(int64_t vertex_id);
bool RemoveVertexWithResult(State state);
```

### Validation and Maintenance

```cpp
void ResetAllVertices();       // Reset vertex states for new search
void ClearAll();               // Clear entire graph
void ValidateStructure() const; // Throws DataCorruptionError if issues found
void ValidateEdgeWeight(Transition weight) const; // Throws for invalid weights
```

### Iterator Support

```cpp
// Vertex iterators
vertex_iterator vertex_begin();
vertex_iterator vertex_end();
const_vertex_iterator vertex_begin() const;
const_vertex_iterator vertex_end() const;
const_vertex_iterator vertex_cbegin() const;
const_vertex_iterator vertex_cend() const;

// Range-based for loop support
vertex_range vertices();
const_vertex_range vertices() const;
```

---

## SearchContext Class

Thread-local search context that externalizes search state from vertices, enabling concurrent searches on the same graph.

### Template Declaration

```cpp
template <typename State, typename Transition, typename StateIndexer>
class SearchContext;
```

### Constructors

```cpp
SearchContext();                        // Default with 1000-vertex pre-allocation
explicit SearchContext(size_t reserve_size);  // Custom pre-allocation
```

### Memory Management

```cpp
void Reserve(size_t n);                 // Reserve space for n vertices
size_t Capacity() const noexcept;       // Current capacity
void Clear();                           // Remove all search data
void Reset();                           // Reset values, preserve memory (efficient for reuse)
size_t Size() const;                    // Number of vertices with search info
bool Empty() const;                     // Check if empty
```

### Search Information Access

```cpp
// Get search info for a vertex (creates if doesn't exist)
SearchVertexInfo& GetSearchInfo(int64_t vertex_id);
SearchVertexInfo& GetSearchInfo(vertex_iterator vertex_it);

// Const access (throws ElementNotFoundError if not found)
const SearchVertexInfo& GetSearchInfo(int64_t vertex_id) const;

// Check existence
bool HasSearchInfo(int64_t vertex_id) const;
```

### Diagnostics

```cpp
size_t GetNodesExpanded() const noexcept;       // Nodes expanded in current search
void IncrementNodesExpanded() noexcept;         // Called by algorithms
void ResetNodesExpanded() noexcept;             // Reset counter
```

### Multi-Goal Search Support

```cpp
bool IsMultiGoalSearch() const noexcept;        // Check if multi-goal mode
void SetMultiGoalSearch(bool is_multi_goal) noexcept;  // Set mode
```

### Start Vertex Tracking

```cpp
void SetStartVertexId(int64_t vertex_id);       // Set start vertex
int64_t GetStartVertexId() const noexcept;      // Get start vertex ID
bool HasStartVertex() const noexcept;           // Check if set
```

### Path Reconstruction

```cpp
template<typename PathState = State>
std::vector<PathState> ReconstructPath(const GraphType* graph, int64_t goal_id) const;
```

### Flexible Attribute System

```cpp
// Vertex-level attributes
template<typename T>
void SetVertexAttribute(int64_t vertex_id, const std::string& key, const T& value);

template<typename T>
const T& GetVertexAttribute(int64_t vertex_id, const std::string& key) const;

template<typename T>
T GetVertexAttributeOr(int64_t vertex_id, const std::string& key, const T& default_value) const;

bool HasVertexAttribute(int64_t vertex_id, const std::string& key) const;
std::vector<std::string> GetVertexAttributeKeys(int64_t vertex_id) const;

// Context-level attributes (for algorithm state like DFS counters)
template<typename T>
void SetContextAttribute(const std::string& key, const T& value);

template<typename T>
const T& GetContextAttribute(const std::string& key) const;

template<typename T>
T GetContextAttributeOr(const std::string& key, const T& default_value) const;

bool HasContextAttribute(const std::string& key) const;
int64_t IncrementContextCounter(const std::string& key);  // For counters
```

### SearchVertexInfo

Nested struct containing search state for a single vertex.

```cpp
struct SearchVertexInfo {
    // Boolean flags
    bool GetChecked() const;
    void SetChecked(bool checked);
    bool GetInOpenList() const;
    void SetInOpenList(bool in_list);

    // Cost values (template for custom cost types)
    template<typename T = double> T GetGCost() const;
    template<typename T> void SetGCost(const T& cost);
    template<typename T = double> T GetHCost() const;
    template<typename T> void SetHCost(const T& cost);
    template<typename T = double> T GetFCost() const;
    template<typename T> void SetFCost(const T& cost);

    // Parent tracking
    int64_t GetParent() const;
    void SetParent(int64_t parent);

    void Reset();  // Reset to initial state
};
```

---

## Search Algorithms

### Dijkstra

Optimal shortest path algorithm for graphs with non-negative edge weights.

```cpp
class Dijkstra {
public:
    // Thread-safe search with external context
    template<...>
    static Path<State> Search(
        const Graph<...>* graph,
        SearchContext<...>& context,
        VertexIdentifier start,
        VertexIdentifier goal);

    // Legacy search (non-thread-safe, creates internal context)
    template<...>
    static Path<State> Search(
        const Graph<...>* graph,
        VertexIdentifier start,
        VertexIdentifier goal);

    // Single-source shortest paths to all reachable vertices
    template<...>
    static bool SearchAll(
        const Graph<...>* graph,
        SearchContext<...>& context,
        VertexIdentifier start);

    // Multi-goal search: find path to nearest goal
    template<...>
    static MultiGoalResult<State, Transition> SearchMultiGoal(
        const Graph<...>* graph,
        SearchContext<...>& context,
        VertexIdentifier start,
        const std::vector<VertexIdentifier>& goals);
};
```

**Usage:**
```cpp
// Basic search
auto path = Dijkstra::Search(graph, start, goal);

// Thread-safe search
SearchContext<State> context;
auto path = Dijkstra::Search(graph, context, start, goal);

// Multi-goal search
std::vector<State> goals = {goal1, goal2, goal3};
auto result = Dijkstra::SearchMultiGoal(graph, context, start, goals);
if (result.found) {
    std::cout << "Reached goal " << result.goal_index << "\n";
}
```

### AStar

Optimal shortest path algorithm using heuristic guidance for faster search.

```cpp
class AStar {
public:
    // Thread-safe search with heuristic
    template<..., typename HeuristicFunc>
    static Path<State> Search(
        const Graph<...>* graph,
        SearchContext<...>& context,
        VertexIdentifier start,
        VertexIdentifier goal,
        HeuristicFunc heuristic);

    // Legacy search (non-thread-safe)
    template<..., typename HeuristicFunc>
    static Path<State> Search(
        const Graph<...>* graph,
        VertexIdentifier start,
        VertexIdentifier goal,
        HeuristicFunc heuristic);

    // Multi-goal search with heuristic
    template<..., typename HeuristicFunc>
    static MultiGoalResult<State, Transition> SearchMultiGoal(
        const Graph<...>* graph,
        SearchContext<...>& context,
        VertexIdentifier start,
        const std::vector<VertexIdentifier>& goals,
        HeuristicFunc heuristic);
};
```

**Heuristic Function:**
```cpp
// Heuristic signature: Transition heuristic(const State& from, const State& to)
double EuclideanDistance(const Location& from, const Location& to) {
    double dx = from.x - to.x;
    double dy = from.y - to.y;
    return std::sqrt(dx * dx + dy * dy);
}

auto path = AStar::Search(graph, context, start, goal, EuclideanDistance);
```

### BFS (Breadth-First Search)

Unweighted shortest path algorithm, optimal for graphs where all edges have equal cost.

```cpp
class BFS {
public:
    // Thread-safe search
    template<...>
    static Path<State> Search(
        const Graph<...>* graph,
        SearchContext<...>& context,
        VertexIdentifier start,
        VertexIdentifier goal);

    // Legacy search (non-thread-safe)
    template<...>
    static Path<State> Search(
        const Graph<...>* graph,
        VertexIdentifier start,
        VertexIdentifier goal);

    // Traverse all reachable vertices
    template<...>
    static bool TraverseAll(
        const Graph<...>* graph,
        SearchContext<...>& context,
        VertexIdentifier start);

    // Check reachability
    template<...>
    static bool IsReachable(
        const Graph<...>* graph,
        VertexIdentifier start,
        VertexIdentifier target);
};

// Alias
using BreadthFirstSearch = BFS;
```

### DFS (Depth-First Search)

Graph traversal algorithm for reachability testing and path finding (not necessarily optimal).

```cpp
class DFS {
public:
    // Thread-safe search
    template<...>
    static Path<State> Search(
        const Graph<...>* graph,
        SearchContext<...>& context,
        VertexIdentifier start,
        VertexIdentifier goal);

    // Legacy search (non-thread-safe)
    template<...>
    static Path<State> Search(
        const Graph<...>* graph,
        VertexIdentifier start,
        VertexIdentifier goal);

    // Traverse all reachable vertices
    template<...>
    static bool TraverseAll(
        const Graph<...>* graph,
        SearchContext<...>& context,
        VertexIdentifier start);

    // Check reachability
    template<...>
    static bool IsReachable(
        const Graph<...>* graph,
        VertexIdentifier start,
        VertexIdentifier goal);
};

// Alias
using DepthFirstSearch = DFS;
```

### SearchAlgorithm (Base Template)

Unified search algorithm template used internally by Dijkstra, AStar, BFS, DFS. Can be used directly for advanced scenarios.

```cpp
template<typename SearchStrategy, typename State, typename Transition, typename StateIndexer>
class SearchAlgorithm {
public:
    // Basic search
    static Path<State> Search(
        const GraphType* graph,
        SearchContextType& context,
        vertex_iterator start,
        vertex_iterator goal,
        const SearchStrategy& strategy);

    // Search with rich result
    static PathResult<State, Transition> SearchWithResult(
        const GraphType* graph,
        SearchContextType& context,
        vertex_iterator start,
        vertex_iterator goal,
        const SearchStrategy& strategy);

    // Search with termination limits
    static PathResult<State, Transition> SearchWithLimits(
        const GraphType* graph,
        SearchContextType& context,
        vertex_iterator start,
        vertex_iterator goal,
        const SearchStrategy& strategy,
        const SearchLimits& limits);

    // Multi-goal search
    static MultiGoalResult<State, Transition> SearchMultiGoal(
        const GraphType* graph,
        SearchContextType& context,
        vertex_iterator start,
        const std::vector<vertex_iterator>& goals,
        const SearchStrategy& strategy);
};
```

---

## DefaultIndexer

Automatic state indexing that works with common patterns.

```cpp
template<typename State>
struct DefaultIndexer;
```

### Supported Patterns

```cpp
// Member variable 'id'
struct MyState { int64_t id; };

// Member variable 'id_'
struct MyState { int64_t id_; };

// Member function 'GetId()'
struct MyState { int64_t GetId() const { return some_unique_value; } };
```

### Custom Indexer

```cpp
struct MyCustomIndexer {
    int64_t operator()(const MyState& state) const {
        return state.custom_unique_field;
    }
};

Graph<MyState, double, MyCustomIndexer> graph;
```

---

## Vertex Class

Vertex data structure storing state and edge information.

```cpp
template<typename State, typename Transition, typename StateIndexer>
struct Vertex {
    State state;                    // User-defined state object
    const int64_t vertex_id;        // Unique vertex identifier
    EdgeListType edges_to;          // Outgoing edges

    Vertex(State s, int64_t id);

    edge_iterator edge_begin() noexcept;
    edge_iterator edge_end() noexcept;
    const_edge_iterator edge_begin() const noexcept;
    const_edge_iterator edge_end() const noexcept;

    bool operator==(const Vertex& other) const;
    int64_t GetId() const;
};
```

---

## Edge Class

Edge data structure connecting vertices.

```cpp
template<typename State, typename Transition, typename StateIndexer>
struct Edge {
    Vertex* dst;        // Destination vertex pointer
    Transition cost;    // Edge weight/cost

    Edge(Vertex* destination, Transition edge_cost);

    bool operator==(const Edge& other) const;
    bool operator!=(const Edge& other) const;
};
```

---

## Exception System

```cpp
class GraphException : public std::exception;           // Base exception
class InvalidArgumentError : public GraphException;     // Invalid parameters
class ElementNotFoundError : public GraphException;     // Missing vertices/edges
class DataCorruptionError : public GraphException;      // Graph structure corruption
class AlgorithmError : public GraphException;           // Search algorithm failures
```

**Usage:**
```cpp
try {
    auto& vertex = graph.GetVertexSafe(invalid_id);
} catch (const ElementNotFoundError& e) {
    std::cout << "Vertex not found: " << e.what() << "\n";
}
```

---

## Complexity Analysis

### Graph Operations

| Operation | Time Complexity | Space Complexity |
|-----------|----------------|------------------|
| Add Vertex | O(1) average | O(1) |
| Remove Vertex | O(m) worst case | O(1) |
| Find Vertex | O(1) average | O(1) |
| Add Edge | O(1) | O(1) |
| Remove Edge | O(m) per vertex | O(1) |
| Find Edge | O(m) per vertex | O(1) |

### Search Algorithms

| Algorithm | Time Complexity | Space Complexity | Optimality |
|-----------|----------------|------------------|------------|
| **Dijkstra** | O((m+n) log n) | O(n) | Optimal |
| **A\*** | O((m+n) log n)* | O(n) | Optimal with admissible heuristic |
| **BFS** | O(m+n) | O(n) | Optimal for unweighted |
| **DFS** | O(m+n) | O(n) | Not optimal |

*A* performance depends on heuristic quality*

---

## Thread Safety

### Safe Operations (No Synchronization Required)

- Multiple concurrent searches using separate `SearchContext` instances
- Read-only graph queries (vertex/edge lookup, statistics)
- Graph structure validation

### Unsafe Operations (Require External Synchronization)

- Graph modifications (adding/removing vertices or edges)
- Legacy search methods without `SearchContext` parameter
- Sharing a `SearchContext` between threads

### Recommended Pattern

```cpp
// Create graph and populate (single-threaded)
Graph<State> graph;
// ... add vertices and edges ...

// Multiple concurrent searches (thread-safe)
void worker_thread(int thread_id) {
    SearchContext<State> context;  // Each thread gets own context
    auto path = Dijkstra::Search(&graph, context, start, goal);
}
```

---

## See Also

- [Quick Start Guide](../getting-started/quick-start.md) - Getting started with libgraph
- [Search Algorithms](./search_algorithms.md) - Detailed algorithm explanations
- [Advanced Features](../guides/advanced_features.md) - Custom cost types, thread safety patterns
- [Architecture](../design/architecture.md) - System design and template patterns
