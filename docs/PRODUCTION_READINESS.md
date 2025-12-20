# libgraph Production Readiness (Robotics Navigation)

This document summarizes production readiness features implemented for robotics navigation use cases. The library focuses on determinism, thread safety, bounded latency, and correctness.

## Implementation Summary

| Feature | Status | Location |
|---------|--------|----------|
| Stable edge/vertex storage | Done | `edge.hpp`, `vertex.hpp` - uses `Vertex*` pointers |
| Deterministic tie-breaking | Done | `search_algorithm.hpp` - breaks ties by vertex_id |
| Debug cost assertions | Done | `debug_checks.hpp` - validates non-negative, finite costs |
| PathResult with metadata | Done | `search_context.hpp` - returns cost, nodes expanded |
| Early termination | Done | `SearchLimits` - max_expansions, timeout_ms |
| Multi-goal search | Done | `Dijkstra::SearchMultiGoal()`, `AStar::SearchMultiGoal()` |
| Nodes expanded counter | Done | `context.GetNodesExpanded()` |
| Memory pre-allocation | Done | `context.Reserve(n)` |

## Thread Safety Model

The library supports concurrent read-only searches on shared graphs:

```cpp
const auto* graph = build_graph();  // Graph is immutable after this

// Multiple threads can search concurrently
std::vector<std::thread> threads;
for (int i = 0; i < NUM_THREADS; ++i) {
    threads.emplace_back([graph, start, goal]() {
        SearchContext<State, Transition, Indexer> context;  // Thread-local
        auto path = Dijkstra::Search(graph, context, start, goal);
    });
}
```

**Rules:**
- Graph must be read-only during concurrent searches
- Each thread requires its own `SearchContext`
- Strategy objects are stateless and can be shared

## Rich Search Results

### PathResult

Get path cost and diagnostics:

```cpp
auto strategy = MakeDijkstraStrategy<State, double, Indexer>();
auto result = SearchAlgorithm<...>::SearchWithResult(graph, context, start, goal, strategy);

if (result.found) {
    std::cout << "Path cost: " << result.total_cost << "\n";
    std::cout << "Nodes expanded: " << result.nodes_expanded << "\n";
}
```

### SearchLimits

Bound search execution for real-time systems:

```cpp
// Limit by expansions
auto limits = SearchLimits::MaxExpansions(1000);
auto result = SearchAlgorithm<...>::SearchWithLimits(graph, context, start, goal, strategy, limits);

// Limit by time
auto limits = SearchLimits::Timeout(50);  // 50ms max
```

### Multi-Goal Search

Find path to nearest goal (e.g., nearest charging station):

```cpp
std::vector<State> goals = {station1, station2, station3};
auto result = Dijkstra::SearchMultiGoal(graph, context, start, goals);

if (result.found) {
    std::cout << "Reached goal index: " << result.goal_index << "\n";
    std::cout << "Goal vertex ID: " << result.goal_vertex_id << "\n";
}
```

## Performance Guidelines

### Pre-allocation

Avoid allocations during search:

```cpp
// Reserve graph capacity before bulk inserts
graph.ReserveVertices(expected_vertices);

// Reserve context before search
context.Reserve(graph.GetVertexCount());
```

### Reusing Context

For repeated searches, reuse context with `Reset()`:

```cpp
SearchContext<State, Transition, Indexer> context;
context.Reserve(graph.GetVertexCount());

for (const auto& query : queries) {
    context.Reset();  // Clears state, preserves capacity
    auto path = Dijkstra::Search(graph, context, query.start, query.goal);
}
```

## Debug Assertions

In debug builds (without `NDEBUG`), the library validates:

- Edge costs are non-negative
- Edge costs are finite (not NaN or Inf)
- Heuristic values are non-negative and finite
- G-costs don't overflow

These checks are compiled away in release builds.

## Determinism

Search results are deterministic across runs:

- Priority queue comparators break ties by `vertex_id`
- No reliance on `unordered_map` iteration order
- Same graph + same query = same path

## Deferred Items

The following were evaluated but deferred:

| Item | Reason |
|------|--------|
| Compact mode (no AttributeMap) | Not critical for soft real-time targets |
| std::shared_mutex wrapper | Read-only during search is sufficient |
| Sanitizer CI profiles | Can be added separately |

## API Summary

### New Types

- `PathResult<State, Cost>` - Rich search result with path, cost, nodes_expanded
- `MultiGoalResult<State, Cost>` - Extends PathResult with goal_vertex_id, goal_index
- `SearchLimits` - Early termination configuration

### New Methods

- `SearchAlgorithm::SearchWithResult()` - Returns PathResult
- `SearchAlgorithm::SearchWithLimits()` - Bounded search
- `SearchAlgorithm::SearchMultiGoal()` - Find nearest goal
- `Dijkstra::SearchMultiGoal()` - Convenience wrapper
- `AStar::SearchMultiGoal()` - Convenience wrapper
- `context.GetNodesExpanded()` - Diagnostic counter
