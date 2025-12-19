# LibGraph Production Readiness TODO

**Goal:** Prepare libgraph for production use in robotics navigation stack
**Current Status:** 302 tests passing, Phase 1-4 complete
**Branch:** refactor-production_readiness

---

## Production Readiness Summary

| Category | Status | Blocking Issues |
|----------|--------|-----------------|
| Core algorithms | Ready | None |
| Thread safety (reads) | Ready | None |
| Thread safety (writes) | N/A | Documented limitation |
| DFS thread safety | **FIXED** | None |
| BFS performance | **FIXED** | Uses std::queue O(1) |
| Search efficiency | **FIXED** | Lazy deletion for priority updates |
| Documentation | **COMPLETE** | Thread-safety documented in all headers |
| Test coverage | **COMPLETE** | 302 tests including stress/perf/determinism |

---

## Phase 1: Critical Thread-Safety Fixes

**Priority: BLOCKING for production**

### 1.1 Fix DFS Strategy Thread-Safety Violation - COMPLETED

**File:** `include/graph/search/dfs.hpp:41`
**Issue:** `mutable int64_t timestamp_counter_` was in strategy class, not SearchContext
**Risk:** Two concurrent DFS searches sharing same strategy instance would corrupt counter

**Solution Implemented:**
- [x] Added context-level attribute system to SearchContext (`SetContextAttribute`, `GetContextAttribute`, `IncrementContextCounter`)
- [x] Updated SearchStrategy interface to pass SearchContext to `InitializeVertex` and `RelaxVertex`
- [x] Updated DfsStrategy to store timestamp counter in context via `IncrementContextCounter("dfs_timestamp_counter")`
- [x] Removed `mutable timestamp_counter_` from DfsStrategy
- [x] Updated all strategy implementations (A*, Dijkstra, BFS) to accept context parameter
- [x] Added concurrent DFS test: `ConcurrentSearchesWithSharedStrategy` (10 threads, 20 searches each)
- [x] Added context attribute reset test: `ContextAttributeResetBetweenSearches`

**Files Modified:** `dfs.hpp`, `search_context.hpp`, `search_strategy.hpp`, `search_algorithm.hpp`, `astar.hpp`, `dijkstra.hpp`, `bfs.hpp`, `dfs_comprehensive_test.cpp`
**Verification:** `./bin/utests --gtest_filter=*Dfs*` - 16 tests pass

---

### 1.2 Fix BFS Implementation - Use Proper Queue - COMPLETED
**File:** `include/graph/search/bfs.hpp`
**Issue:** Used priority_queue with "cost increment by 1" - O(log n) overhead, fragile
**Risk:** Performance degradation; breaks with non-arithmetic cost types

**Solution Implemented:**
- [x] Created `QueueBasedSearch()` private method using `std::queue<vertex_iterator>` - O(1) operations
- [x] Track visited vertices using SearchContext (SetChecked/GetChecked flags)
- [x] Updated `BFS::Search()` to call queue-based implementation
- [x] Added `ReconstructPath()` helper for path reconstruction from SearchContext
- [x] Verified `BFS::TraverseAll()` and `BFS::IsReachable()` work correctly
- [x] Added performance tests:
  - `QueueBasedShortestPathOnGrid`: 5x5 grid shortest path verification
  - `NonArithmeticCostType`: StringCost custom type test
  - `LinearTimeComplexityVerification`: O(V+E) scaling test with 10K vertices

**Files Modified:** `bfs.hpp`, `bfs_comprehensive_test.cpp`
**Verification:** `./bin/utests --gtest_filter=*BFS*` - 19 tests pass

---

### 1.3 Add Atomicity to AddUndirectedEdge - COMPLETED
**File:** `include/graph/impl/graph_impl.hpp:193-206`
**Issue:** If first AddEdge succeeds but second fails, graph left inconsistent

**Solution Implemented:**
- [x] Added try-catch block around second AddEdge call
- [x] On exception, rollback first edge with RemoveEdge and re-throw
- [x] Added `AddUndirectedEdgeAtomicity` test verifying:
  - Both edges created for undirected edge
  - Cost updates propagate to both directions
  - Self-loop handling

**Files Modified:** `graph_impl.hpp`, `graph_mod_test.cpp`
**Verification:** `./bin/utests --gtest_filter=*GraphMod*` - 16 tests pass

---

## Phase 2: Performance Optimizations

**Priority: HIGH for real-time robotics**

### 2.1 Fix Priority Queue Relaxation Handling - COMPLETED
**Files:** `search_algorithm.hpp`
**Issue:** std::priority_queue doesn't support priority updates; relaxed vertices kept old priority
**Risk:** Suboptimal vertex expansion order, potential correctness issues

**Solution Implemented:**
- [x] Implemented lazy deletion strategy in `ExpandNeighbors`:
  - Always push when vertex is relaxed (even if already in openlist)
  - Stale entries automatically skipped when popped (is_checked flag)
  - Guarantees correct priority ordering for relaxed vertices
- [x] Added DPQ infrastructure (DPQComparator, VertexIteratorIndexer, ExpandNeighborsDPQ)
  - Full DPQ integration optional - requires comparator initialization changes
  - Current lazy deletion approach is correct and widely used
- [x] Added include for dynamic_priority_queue.hpp

**Files Modified:** `search_algorithm.hpp`
**Verification:** `./bin/utests` - 269 tests pass

**Future Enhancement (Optional):**
- Full DynamicPriorityQueue integration for O(log n) updates vs lazy deletion's O(n) memory
- Requires modifying DPQ to accept comparator at construction time

---

### 2.2 Configurable SearchContext Pre-allocation - COMPLETED
**File:** `include/graph/search/search_context.hpp`
**Issue:** Fixed 1000 vertex reserve; large graphs cause reallocations

**Solution Implemented:**
- [x] Added constructor `SearchContext(size_t reserve_size)` for explicit pre-allocation
- [x] Added `Reserve(size_t n)` method for dynamic resizing
- [x] Added `Capacity()` method to query current bucket count
- [x] Documented: "For real-time robotics, reserve >= expected graph size"
- [x] Added test `ConfigurablePreallocation` verifying functionality

**Files Modified:** `search_context.hpp`, `search_context_comprehensive_test.cpp`
**Verification:** `./bin/utests --gtest_filter=*SearchContext*` - 20 tests pass

---

### 2.3 Explicit Start Vertex Tracking - COMPLETED
**File:** `include/graph/search/search_context.hpp`
**Issue:** Inferred start by checking `parent_id == -1` - inefficient O(n) iteration

**Solution Implemented:**
- [x] Added `start_vertex_id_` member (default -1)
- [x] Added `SetStartVertexId()`, `GetStartVertexId()`, `HasStartVertex()` methods
- [x] Updated `Reset()` to clear start vertex ID
- [x] Updated `ReconstructPath()` to use explicit start vertex for O(1) termination check
- [x] Updated `SearchAlgorithm::PerformSearch()` to call `SetStartVertexId()` after Reset()
- [x] Updated `BFS::QueueBasedSearch()` to call `SetStartVertexId()` after Reset()
- [x] Added tests: `StartVertexTracking`, `PathReconstructionWithStartVertex`

**Files Modified:** `search_context.hpp`, `search_algorithm.hpp`, `bfs.hpp`, `search_context_comprehensive_test.cpp`
**Verification:** `./bin/utests --gtest_filter=*SearchContext*` - 19 tests pass

---

## Phase 3: Code Quality and API Cleanup

**Priority: MEDIUM - improves maintainability**

### 3.1 Remove Deprecated Vertex Fields - COMPLETED
**File:** `include/graph/vertex.hpp:70-81`
**Issue:** Deprecated fields consumed ~48 bytes per vertex, hindered thread safety

**Solution Implemented:**
- [x] Removed deprecated vertex fields: `is_checked`, `is_in_openlist`, `f_cost`, `g_cost`, `h_cost`, `search_parent`
- [x] Removed `ClearVertexSearchInfo()` method from Vertex class
- [x] Removed `ResetAllVertices()` method from Graph class
- [x] Updated tests to use SearchContext instead of vertex fields
- [x] Memory savings: ~48 bytes per vertex

**Files Modified:** `vertex.hpp`, `vertex_impl.hpp`, `graph.hpp`, `graph_impl.hpp`, `vertex_independent_test.cpp`, `pq_with_graph_test.cpp`
**Verification:** All 268 tests pass

---

### 3.2 Fix const_cast in AddEdgeWithResult - COMPLETED
**File:** `include/graph/impl/graph_impl.hpp`
**Issue:** Used `const_cast` to modify edge cost - violated const semantics

**Solution Implemented:**
- [x] Replaced `const auto& edge` iteration with `FindEdge()` which returns non-const iterator
- [x] Use `edge_it->cost = trans` directly without const_cast
- [x] Also optimized edge addition to avoid redundant vertex lookup via `AddEdge()`

**Files Modified:** `graph_impl.hpp`
**Verification:** `./bin/utests --gtest_filter=*Edge*` - 41 tests pass

---

### 3.3 Document Thread-Safety Model - COMPLETED
**Files:** Public headers, README.md

**Solution Implemented:**
- [x] Added "THREAD SAFETY" section to `search_context.hpp` header comment (full model documentation)
- [x] Added "THREAD SAFETY" section to `search_algorithm.hpp` header comment
- [x] Added "THREAD SAFETY" section to `astar.hpp`, `dijkstra.hpp`, `bfs.hpp`, `dfs.hpp`
- [x] Enhanced README.md thread-safety section with explicit model description
- [x] Documented that legacy overloads without SearchContext are NOT thread-safe

**Files Modified:** `search_context.hpp`, `search_algorithm.hpp`, `astar.hpp`, `dijkstra.hpp`, `bfs.hpp`, `dfs.hpp`, `README.md`
**Verification:** All 269 tests pass

---

## Phase 4: Production Testing Suite

**Priority: REQUIRED for robotics deployment**

### 4.1 Performance Scaling Tests - COMPLETED
**File:** `tests/unit_test/performance_scaling_test.cpp`

**Tests Implemented:**
- [x] `Construction_1K` - 32x32 grid (1024 vertices), < 100ms
- [x] `Construction_10K` - 100x100 grid (10000 vertices), < 1s
- [x] `Construction_100K` - 316x316 grid (~100K vertices), < 10s
- [x] `DijkstraSearch_Scaling` - Search time across different graph sizes
- [x] `AStarSearch_Scaling` - A* search time scaling
- [x] `AStarVsDijkstraComparison` - Performance comparison
- [x] `SearchContextReuse` - No degradation with context reuse
- [x] `MemoryEfficiency` - Verify O(V+E) space
- [x] `BFSScaling` - BFS O(V+E) time complexity

---

### 4.2 Timing Bounds Tests - COMPLETED
**File:** `tests/unit_test/timing_bounds_test.cpp`

**Tests Implemented:**
- [x] `SearchLatency_Distribution` - P50, P90, P99 latency measurement
- [x] `AdversarialGraph_LongChain` - 5000 vertex chain
- [x] `AdversarialGraph_DenseCluster` - Fully connected 200 vertex cluster
- [x] `AdversarialGraph_StarTopology` - 1000 leaf star graph
- [x] `WorstCaseHeuristic` - A* with zero heuristic
- [x] `MultipleAlgorithmsComparison` - Dijkstra, A*, BFS, DFS timing
- [x] `EmptyGraphSearch` - Fast failure verification
- [x] `NoPathExists` - Disconnected graph handling

---

### 4.3 Determinism Validation Tests - COMPLETED
**File:** `tests/unit_test/determinism_test.cpp`

**Tests Implemented:**
- [x] `IdenticalPaths_Dijkstra_1000Runs` - 1000 identical searches
- [x] `IdenticalPaths_AStar_1000Runs` - A* determinism
- [x] `IdenticalPaths_BFS_1000Runs` - BFS determinism
- [x] `ContextReusePreservesDeterminism` - Context reuse consistency
- [x] `DifferentStartGoalPairs` - Multiple query determinism
- [x] `ConstructionOrderIndependence` - Edge order doesn't affect results
- [x] `HashIndependence` - Non-sequential IDs work correctly
- [x] `OptimalPathCostConsistency` - Path costs are consistent
- [x] `AllAlgorithmsConsistentWithSelf` - All algorithms self-consistent

---

### 4.4 Concurrent Search Stress Tests - COMPLETED
**File:** `tests/unit_test/concurrent_stress_test.cpp`

**Tests Implemented:**
- [x] `SimultaneousDijkstraSearches_100` - 100 concurrent Dijkstra
- [x] `SimultaneousAStarSearches_100` - 100 concurrent A*
- [x] `DifferentAlgorithmsMixed` - 25 each of 4 algorithms (100 total)
- [x] `DifferentStartGoalPairs` - 50 concurrent different queries
- [x] `SharedStrategyObject` - Strategy sharing verification
- [x] `RapidSuccessiveSearches` - 10 threads x 100 searches each
- [x] `HighContentionScenario` - 200 threads starting simultaneously
- [x] `LongRunningConcurrentSearches` - 2 second sustained load test

**Verification:** All 302 tests pass

---

## Phase 5: Robotics-Specific Enhancements (Optional)

**Priority: LOW - nice-to-have for navigation**

### 5.1 Safety Constraint Framework
- [ ] Vertex/edge validation callbacks
- [ ] Forbidden region support
- [ ] Dynamic obstacle integration

### 5.2 Multi-Goal Search
- [ ] Search to multiple goals, return nearest
- [ ] Efficient multi-target Dijkstra

### 5.3 Anytime Search
- [ ] Best-so-far path on timeout
- [ ] Deadline parameter for Search()

---

## Dependency Graph

```
Phase 1.1 (DFS fix) ──┐
Phase 1.2 (BFS fix) ──┼──► Phase 2.1 (DPQ integration) ──► Phase 4 (Testing)
Phase 1.3 (Atomic)  ──┘              │
                                     ▼
                           Phase 2.2 (Prealloc)
                           Phase 2.3 (Start tracking)
                                     │
                                     ▼
                           Phase 3 (Cleanup) ──► Phase 4 (Testing)
```

**Recommended Order:**
1. Phase 1.1 (DFS) - independent, critical
2. Phase 1.2 (BFS) - independent, critical
3. Phase 1.3 (Atomic) - independent, quick fix
4. Phase 2.3 (Start tracking) - small, useful
5. Phase 2.2 (Prealloc) - small, useful
6. Phase 2.1 (DPQ) - larger, after basics stable
7. Phase 3.1 (Deprecated fields) - after Phase 2 complete
8. Phase 4.x (Tests) - can be added incrementally

---

## Completion Checklist

**Production Ready When:**
- [ ] Phase 1 complete (all critical fixes)
- [ ] Phase 2 complete (performance optimizations)
- [ ] Phase 3 complete (code cleanup)
- [ ] Phase 4 tests passing
- [ ] `./bin/utests` - all tests pass
- [ ] Build with `-Wall -Wextra` - no warnings
- [ ] ThreadSanitizer clean (concurrent tests)
- [ ] AddressSanitizer clean (memory tests)
- [ ] Documentation updated

---

## Quick Reference: Files to Modify

| Task | Files | Est. Lines Changed |
|------|-------|-------------------|
| 1.1 DFS fix | `dfs.hpp`, `search_context.hpp` | ~50 |
| 1.2 BFS fix | `bfs.hpp` | ~80 |
| 1.3 Atomic edge | `graph_impl.hpp` | ~20 |
| 2.1 DPQ integration | `search_algorithm.hpp` | ~150 |
| 2.2 Prealloc | `search_context.hpp` | ~20 |
| 2.3 Start tracking | `search_context.hpp`, `search_algorithm.hpp` | ~30 |
| 3.1 Deprecated fields | `vertex.hpp`, `graph.hpp`, CMake | ~40 |
| 4.x New tests | New files in `tests/unit_test/` | ~400 |

---

## Commands Reference

```bash
# Build with tests
cd build && cmake -DBUILD_TESTING=ON .. && cmake --build . -j8

# Run all tests
./bin/utests

# Run specific test suite
./bin/utests --gtest_filter=*Dfs*
./bin/utests --gtest_filter=*Bfs*
./bin/utests --gtest_filter=*SearchContext*

# Build with sanitizers (for Phase 4)
cmake -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined" ..
cmake -DCMAKE_CXX_FLAGS="-fsanitize=thread" ..

# Check for warnings
cmake -DCMAKE_CXX_FLAGS="-Wall -Wextra -Werror" ..
```

---

## Previous Accomplishments (Reference)

<details>
<summary>Click to expand historical progress</summary>

### Completed Phases

**Phase 1-4: Core Development (Completed)**
- Template-based SearchAlgorithm with CRTP strategy pattern
- Generic cost types with configurable TransitionComparator
- 35% SearchContext performance improvement
- Comprehensive exception hierarchy (7 types)
- STL-compatible iterators
- Enterprise-grade documentation suite
- 260 tests with 100% pass rate

**Recent Improvements (Dec 2024 - Jan 2025)**
- Tree class modernization with thread-safe RemoveSubtree
- API type consistency (size_t standardization)
- Comprehensive BFS/DFS edge case tests
- CI coverage standardization across Ubuntu versions

</details>

---

## Notes

- Each task should be completed and tested before dependent tasks
- Commit after each completed task with descriptive message
- Run full test suite after each change
- Update this TODO.md as tasks complete
