# Architecture Overview

This document provides an in-depth look at the libgraph library architecture, design patterns, and implementation details for contributors and advanced users.

## Table of Contents

- [Design Philosophy](#design-philosophy)
- [Template System Architecture](#template-system-architecture)
- [Core Components](#core-components)
- [Memory Management](#memory-management)
- [Search Framework](#search-framework)
- [Thread Safety Design](#thread-safety-design)
- [Performance Characteristics](#performance-characteristics)
- [Design Patterns](#design-patterns)
- [Extension Points](#extension-points)

## Design Philosophy

libgraph is built on several core principles:

### Header-Only Design
- **Zero compilation overhead** for users
- **Template specialization** resolved at compile time
- **Easy integration** - just include headers
- **No ABI compatibility issues** across different compilers/versions

### Generic Programming
- **Compile-time polymorphism** using templates
- **Type safety** with static assertions and SFINAE
- **Zero-cost abstractions** - no runtime overhead
- **Customizable behavior** through template parameters and traits

### Modern C++ Practices
- **RAII memory management** with smart pointers
- **Exception safety** with strong guarantees
- **Move semantics** for performance
- **STL compatibility** with standard algorithms and containers

## Template System Architecture

### Primary Template Parameters

```cpp
template<typename State, typename Transition = double, typename StateIndexer = DefaultIndexer<State>>
class Graph;
```

#### State Template Parameter
The `State` type represents vertex data and must satisfy:

```cpp
// Option 1: Has public id field
struct MyState {
    int64_t id;  // Used by DefaultIndexer
};

// Option 2: Has GetId() method
struct MyState {
    int64_t GetId() const { return unique_id; }
};

// Option 3: Custom indexer
struct CustomIndexer {
    int64_t operator()(const MyState& state) const {
        return state.custom_field;
    }
};
```

#### Transition Template Parameter
The `Transition` type represents edge weights/costs and must support:

```cpp
// Required operations for search algorithms
bool operator<(const Transition& other) const;
bool operator>(const Transition& other) const; 
Transition operator+(const Transition& other) const;

// Required for initialization
template<> struct CostTraits<MyTransition> {
    static MyTransition infinity() { /* return max value */ }
};
```

#### StateIndexer Template Parameter
Generates unique int64_t IDs for states:

```cpp
struct StateIndexer {
    int64_t operator()(const State& state) const {
        // Generate unique ID from state
    }
};
```

### Template Specialization Strategy

The library uses partial specialization and SFINAE for customization:

```cpp
// Automatic detection of indexing method
template<typename T>
auto GetStateId(const T& state) -> decltype(state.id) {
    return state.id;
}

template<typename T>
auto GetStateId(const T& state) -> decltype(state.GetId()) {
    return state.GetId();
}

template<typename T>
auto GetStateId(const T& state) -> decltype(state.id_) {
    return state.id_;
}
```

## Core Components

### Graph Container

```cpp
class Graph {
private:
    std::unordered_map<int64_t, std::unique_ptr<Vertex<State, Transition>>> vertices_;
    StateIndexer state_indexer_;
    
public:
    // Vertex management
    void AddVertex(const State& state);
    bool RemoveVertex(const State& state);
    Vertex<State, Transition>* GetVertexPtr(const State& state);
    
    // Edge management  
    void AddEdge(const State& from, const State& to, const Transition& cost);
    bool RemoveEdge(const State& from, const State& to);
};
```

#### Design Rationale
- **Hash map for vertices**: O(1) average access time vs O(log n) for ordered containers
- **Unique pointers**: Automatic memory management, exception safety
- **StateIndexer composition**: Flexible ID generation strategy

### Vertex Structure

```cpp
template<typename State, typename Transition>
class Vertex {
    State state_;                              // User data
    std::list<Edge<State, Transition>> edges_; // Outgoing edges
    int64_t id_;                               // Unique identifier
    
public:
    using EdgeIterator = typename std::list<Edge<State, Transition>>::iterator;
    using ConstEdgeIterator = typename std::list<Edge<State, Transition>>::const_iterator;
};
```

#### Edge List Implementation
- **Linked list vs vector**: O(1) insertion/deletion vs O(n) for vectors
- **Iterator stability**: Iterators remain valid during modifications
- **Memory efficiency**: No wasted capacity like vectors

### Edge Structure

```cpp
template<typename State, typename Transition>
class Edge {
    Vertex<State, Transition>* dst_;  // Destination vertex pointer
    Transition cost_;                 // Edge weight/cost
    int64_t id_;                      // Unique edge identifier
    
public:
    const Transition& GetCost() const { return cost_; }
    Vertex<State, Transition>* GetDst() const { return dst_; }
};
```

## Memory Management

### RAII Principles

The library follows strict RAII (Resource Acquisition Is Initialization) principles:

```cpp
class Graph {
private:
    // Automatic cleanup through smart pointers
    std::unordered_map<int64_t, std::unique_ptr<Vertex<State, Transition>>> vertices_;
    
public:
    // Exception-safe vertex addition
    void AddVertex(const State& state) {
        auto vertex = std::make_unique<Vertex<State, Transition>>(state, state_indexer_(state));
        
        // Strong exception safety: either succeeds completely or has no effect
        auto result = vertices_.emplace(vertex->GetId(), std::move(vertex));
        if (!result.second) {
            throw std::invalid_argument("Vertex with this ID already exists");
        }
    }
};
```

### Memory Layout Optimization

```cpp
// Vertex memory layout optimized for cache efficiency
class Vertex {
    State state_;        // Hot data: accessed frequently during searches
    int64_t id_;        // Hot data: used for indexing
    std::list<Edge> edges_; // Cold data: only accessed when exploring edges
};
```

### Smart Pointer Usage

```cpp
// Graph owns vertices exclusively
std::unique_ptr<Vertex<State, Transition>> vertex_ptr;

// Edges hold raw pointers to vertices (non-owning references)
class Edge {
    Vertex<State, Transition>* dst_;  // Raw pointer - graph manages lifetime
};
```

## Search Framework

### Unified Algorithm Interface

All search algorithms implement a common interface using CRTP (Curiously Recurring Template Pattern):

```cpp
template<typename Derived>
class SearchAlgorithmBase {
public:
    template<typename Graph, typename State>
    static std::vector<State> Search(const Graph& graph, 
                                   const State& start, 
                                   const State& goal) {
        return static_cast<Derived*>(nullptr)->SearchImpl(graph, start, goal);
    }
};

class Dijkstra : public SearchAlgorithmBase<Dijkstra> {
public:
    template<typename Graph, typename State>
    std::vector<State> SearchImpl(const Graph& graph, 
                                const State& start, 
                                const State& goal) {
        // Dijkstra-specific implementation
    }
};
```

### Search Context Architecture

```cpp
template<typename State, typename Transition = double>
class SearchContext {
private:
    // Search state storage
    std::unordered_map<int64_t, double> distances_;
    std::unordered_map<int64_t, int64_t> predecessors_;
    DynamicPriorityQueue<State, Transition> priority_queue_;
    
public:
    // Thread-safe state management
    void Reset() { /* Clear all state for reuse */ }
    void PreAllocate(size_t estimated_vertices) { /* Reserve memory */ }
};
```

#### Thread Safety Strategy
- **External search context**: Each thread maintains separate search state
- **Immutable graph during search**: Graph is not modified during read-only operations
- **No shared mutable state**: Eliminates need for synchronization primitives

### Priority Queue Implementation

```cpp
template<typename State, typename Transition, typename Compare>
class DynamicPriorityQueue {
private:
    std::vector<std::pair<Transition, int64_t>> heap_;      // Binary heap
    std::unordered_map<int64_t, size_t> position_map_;     // State ID -> heap position
    Compare comparator_;
    
public:
    void Push(const State& state, const Transition& priority);
    State Pop();
    void UpdatePriority(const State& state, const Transition& new_priority);
    bool Empty() const;
};
```

#### Heap Operations Complexity
- **Push**: O(log n) - standard heap insertion
- **Pop**: O(log n) - extract minimum with heap property maintenance  
- **UpdatePriority**: O(log n) - position tracking enables efficient updates
- **Space**: O(n) - heap storage plus position mapping

## Thread Safety Design

### Read-Only Concurrent Access

```cpp
// Multiple threads can safely perform concurrent searches
void MultiThreadedSearch() {
    const Graph<Location> map = BuildGraph();  // Immutable after construction
    
    std::vector<std::thread> workers;
    for (int i = 0; i < num_threads; ++i) {
        workers.emplace_back([&map, i]() {
            SearchContext<Location> context;  // Thread-local state
            auto path = Dijkstra::Search(map, context, starts[i], goals[i]);
            ProcessPath(path);
        });
    }
    
    for (auto& t : workers) t.join();
}
```

### Graph Modification Safety

Graph modifications require external synchronization:

```cpp
class ThreadSafeGraph {
private:
    Graph<State, Transition> graph_;
    mutable std::shared_mutex mutex_;
    
public:
    // Exclusive write access
    void AddVertex(const State& state) {
        std::lock_guard<std::shared_mutex> lock(mutex_);
        graph_.AddVertex(state);
    }
    
    // Concurrent read access
    template<typename... Args>
    auto Search(Args&&... args) const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return Dijkstra::Search(graph_, std::forward<Args>(args)...);
    }
};
```

## Performance Characteristics

### Time Complexity Analysis

| Operation | Average Case | Worst Case | Notes |
|-----------|--------------|------------|-------|
| AddVertex | O(1) | O(1) | Hash table insertion |
| RemoveVertex | O(d) | O(d) | d = vertex degree |
| AddEdge | O(1) | O(1) | List insertion |
| RemoveEdge | O(d) | O(d) | Linear search in edge list |
| Search | O((m+n) log n) | O((m+n) log n) | Priority queue operations |

### Space Complexity Analysis

```cpp
// Memory usage breakdown for Graph<State, Transition>
// Vertices: n * (sizeof(State) + sizeof(Vertex) + hash_table_overhead)
// Edges: m * (sizeof(Transition) + sizeof(Edge) + list_node_overhead)
// Total: O(n * sizeof(State) + m * sizeof(Transition))
```

### Cache Performance Considerations

```cpp
// Edge list traversal pattern optimized for cache efficiency
for (const auto& edge : vertex->GetEdges()) {
    // Sequential memory access through linked list
    ProcessEdge(edge);
}

// Hash table access pattern for vertex lookup
auto* vertex = graph.GetVertexPtr(state);  // Single hash lookup
```

## Design Patterns

### CRTP (Curiously Recurring Template Pattern)

Used for static polymorphism in search algorithms:

```cpp
template<typename Derived>
class SearchAlgorithm {
public:
    template<typename... Args>
    auto Search(Args&&... args) -> decltype(static_cast<Derived*>(this)->SearchImpl(std::forward<Args>(args)...)) {
        return static_cast<Derived*>(this)->SearchImpl(std::forward<Args>(args)...);
    }
};
```

Benefits:
- **Zero runtime overhead** compared to virtual functions
- **Type safety** at compile time
- **Interface consistency** across algorithm implementations

### Strategy Pattern

Cost comparison and heuristic functions:

```cpp
template<typename State, typename Transition, typename Compare = std::less<Transition>>
class Dijkstra {
private:
    Compare comparator_;  // Strategy for cost comparison
    
public:
    Dijkstra(Compare comp = Compare{}) : comparator_(comp) {}
};
```

### Template Traits

Customization points for user types:

```cpp
// Primary template
template<typename T>
struct CostTraits {
    static T infinity() { return std::numeric_limits<T>::max(); }
};

// User specialization
template<>
struct CostTraits<MyCustomCost> {
    static MyCustomCost infinity() { return MyCustomCost::MaxValue(); }
};
```

### RAII Wrappers

Exception-safe resource management:

```cpp
class SearchContext {
private:
    std::unique_ptr<ContextImpl> impl_;  // RAII for internal state
    
public:
    SearchContext() : impl_(std::make_unique<ContextImpl>()) {}
    ~SearchContext() = default;  // Automatic cleanup
    
    // Non-copyable, movable
    SearchContext(const SearchContext&) = delete;
    SearchContext(SearchContext&&) = default;
};
```

## Extension Points

### Custom State Types

Requirements and best practices:

```cpp
struct CustomState {
    // Required: Unique identification
    int64_t GetId() const { return id_; }
    
    // Recommended: Equality comparison  
    bool operator==(const CustomState& other) const {
        return id_ == other.id_;
    }
    
    // Optional: Hash function for unordered containers
    struct Hash {
        size_t operator()(const CustomState& state) const {
            return std::hash<int64_t>{}(state.GetId());
        }
    };
    
private:
    int64_t id_;
    // User data...
};
```

### Custom Cost Types

Implementation requirements:

```cpp
struct CustomCost {
    // Required for search algorithms
    bool operator<(const CustomCost& other) const;
    bool operator>(const CustomCost& other) const;  
    bool operator<=(const CustomCost& other) const;
    bool operator>=(const CustomCost& other) const;
    bool operator==(const CustomCost& other) const;
    bool operator!=(const CustomCost& other) const;
    
    // Required for path cost accumulation
    CustomCost operator+(const CustomCost& other) const;
    CustomCost& operator+=(const CustomCost& other);
    
    // Required for A* (optional for other algorithms)
    CustomCost operator-(const CustomCost& other) const;
};

// Required: CostTraits specialization
namespace xmotion {
    template<>
    struct CostTraits<CustomCost> {
        static CustomCost infinity() { return CustomCost::Max(); }
    };
}
```

### Custom Heuristic Functions

For A* algorithm:

```cpp
// Function object approach
struct ManhattanDistance {
    double operator()(const GridPoint& from, const GridPoint& to) const {
        return std::abs(from.x - to.x) + std::abs(from.y - to.y);
    }
};

// Lambda approach
auto euclidean = [](const Point& from, const Point& to) -> double {
    double dx = from.x - to.x;
    double dy = from.y - to.y;
    return std::sqrt(dx * dx + dy * dy);
};

// Usage
auto path = AStar::Search(graph, start, goal, ManhattanDistance{});
auto path2 = AStar::Search(graph, start, goal, euclidean);
```

### Custom Priority Queue

For specialized use cases:

```cpp
template<typename State, typename Transition, typename Compare>
class CustomPriorityQueue {
public:
    void Push(const State& state, const Transition& priority);
    State Pop();
    void UpdatePriority(const State& state, const Transition& new_priority);
    bool Empty() const;
    size_t Size() const;
};
```

This architecture provides a solid foundation for high-performance graph operations while maintaining flexibility and type safety through modern C++ template techniques.