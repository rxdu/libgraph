# Thread Safety Design

This document describes the thread safety architecture in libgraph for concurrent graph operations.

## Overview

libgraph supports concurrent read-only searches on shared graphs through external SearchContext. This design enables multiple threads to perform pathfinding operations simultaneously without synchronization overhead.

## Design Rationale

### Problem Analysis

Graph search algorithms require temporary state for each search:
- Distance/cost tracking for each vertex
- Priority queue for frontier expansion
- Predecessor tracking for path reconstruction
- Visited/explored flags

Storing this state inside graph vertices creates race conditions when multiple threads search concurrently.

### Solution: External Search Context

By moving all search state to an external, thread-local SearchContext:

- **Thread Isolation**: Each search context is independent
- **Concurrent Reads**: Multiple threads can search the same const graph
- **Memory Efficiency**: Context only stores data for visited vertices
- **Performance**: Context reuse eliminates repeated allocations

## Architecture

### SearchContext

```cpp
template <typename State, typename Transition, typename StateIndexer>
class SearchContext {
private:
  std::unordered_map<int64_t, SearchVertexInfo> search_data_;

public:
  struct SearchVertexInfo {
    bool is_checked = false;
    bool is_in_openlist = false;
    double f_cost, g_cost, h_cost;
    int64_t parent_id = -1;
  };

  SearchVertexInfo& GetSearchInfo(int64_t vertex_id);
  void Reset();
  void Reserve(size_t estimated_vertices);
};
```

### Thread-Safe Search Pattern

```cpp
void concurrent_search(const Graph<Location>& graph) {
    // Each thread creates its own context
    SearchContext<Location> context;

    // Graph accessed as const - read-only
    auto path = Dijkstra::Search(graph, context, start, goal);
}
```

## Usage Patterns

### Basic Concurrent Searches

```cpp
const Graph<Location> map = BuildGraph();  // Immutable after construction

std::vector<std::thread> workers;
for (int i = 0; i < num_threads; ++i) {
    workers.emplace_back([&map, i]() {
        SearchContext<Location> context;  // Thread-local
        auto path = Dijkstra::Search(map, context, starts[i], goals[i]);
        ProcessPath(path);
    });
}

for (auto& t : workers) t.join();
```

### Context Reuse for Performance

```cpp
SearchContext<Location> context;
context.Reserve(graph.GetVertexCount());  // Pre-allocate

for (const auto& query : search_queries) {
    context.Reset();  // Clear state, preserve memory
    auto path = Dijkstra::Search(graph, context, query.start, query.goal);
}
```

### Using std::async

```cpp
std::vector<std::future<Path<Location>>> futures;
for (const auto& query : queries) {
    futures.push_back(std::async(std::launch::async, [&graph, query]() {
        SearchContext<Location> context;
        return Dijkstra::Search(graph, context, query.start, query.goal);
    }));
}

for (auto& future : futures) {
    auto path = future.get();
    // Process path...
}
```

## Thread Safety Rules

### Safe Operations (No Synchronization Required)

- Multiple concurrent searches on the same const graph
- Each thread using its own SearchContext
- Reading graph structure (vertex count, edge iteration, etc.)

### Unsafe Operations (Require External Synchronization)

- Graph modifications (AddVertex, AddEdge, RemoveVertex, etc.)
- Sharing a SearchContext between threads
- Modifying graph while searches are in progress

### Thread-Safe Graph Wrapper (Optional)

For use cases requiring concurrent modifications:

```cpp
template<typename State, typename Transition = double>
class ThreadSafeGraph {
private:
    Graph<State, Transition> graph_;
    mutable std::shared_mutex mutex_;

public:
    // Exclusive write access
    void AddVertex(const State& state) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        graph_.AddVertex(state);
    }

    // Shared read access (concurrent searches)
    template<typename... Args>
    auto Search(Args&&... args) const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        SearchContext<State> context;
        return Dijkstra::Search(graph_, context, std::forward<Args>(args)...);
    }
};
```

## Performance Characteristics

| Metric | Single-threaded | Multi-threaded |
|--------|-----------------|----------------|
| Search overhead | ~5% for context | Near-linear scaling |
| Memory per search | O(visited vertices) | O(visited vertices) per thread |
| Context reuse | 20-50% faster | Same benefit per thread |

### Scalability

```
Thread Scalability (8-core system, 1000 searches):
Threads: 1    2    4    6    8
Speedup: 1.0x 1.9x 3.7x 5.4x 7.1x
```

Performance plateaus at core count due to memory bandwidth limits.

## Best Practices

1. **Keep graphs immutable during searches** - Build graph first, then search
2. **Use thread-local contexts** - Never share SearchContext between threads
3. **Pre-allocate context memory** - Call `Reserve()` for known graph sizes
4. **Reuse contexts** - Call `Reset()` between searches instead of creating new contexts
5. **Prefer concurrent searches over locking** - Read-only access scales better

## See Also

- [Architecture](./architecture.md) - Overall library design
- [Performance](../guides/performance.md) - Benchmarking concurrent performance
- [Advanced Features](../guides/advanced_features.md) - Thread-safe patterns and producer-consumer examples
