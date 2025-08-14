# LibGraph Development TODO

## 🎯 Current Status

**Test Suite**: 137 tests (43→137, +218% increase)  
**Architecture**: Major refactoring completed (Edge/Vertex separation, interface/implementation separation)  
**Critical Issues**: 3 implementation bugs discovered and documented  
**State Types**: Full support confirmed for value/pointer/shared_ptr types  

---

## 🚨 Priority 1: Critical Bug Fixes

### Implementation Issues Discovered Through Testing

1. **Exception Safety in ObtainVertexFromVertexMap** - CRITICAL
   - **Issue**: Memory leak if State constructor throws after `new Vertex(state, state_id)`
   - **Location**: graph_impl.hpp:236
   - **Test**: `MemoryManagementTest.ExceptionDuringVertexAdditionDoesNotLeak` fails

2. **Copy Assignment Operator Vertex Lookup** - HIGH
   - **Issue**: Vertices not findable after assignment due to State copy semantics  
   - **Location**: graph_impl.hpp:87-90
   - **Test**: `MemoryManagementTest.AssignmentOperatorHandlesMemoryCorrectly` fails

3. **Thread Safety** - HIGH (if concurrent use needed)
   - **Issue**: All write operations are NOT thread-safe
   - **Impact**: Race conditions in multi-threaded environments
   - **Status**: Documented unsafe behavior through comprehensive tests

---

## 🟡 Priority 2: High Priority Issues

### Memory Management
- [ ] Replace raw pointer management with RAII pattern
- [ ] Fix potential memory leaks in vertex creation/deletion

### API Consistency  
- [ ] Standardize return types across similar operations
- [ ] Add const-correctness to all applicable member functions
- [ ] Fix API documentation inconsistencies

### Exception Safety
- [ ] Replace assert() calls with proper exception handling (tree_impl.hpp:49)
- [ ] Define exception safety guarantees for all operations
- [ ] Add noexcept specifications where appropriate

---

## 🔄 Priority 3: Refactoring Opportunities

### Code Organization
- [ ] Move search algorithms to separate files (AStar/Dijkstra could use separate .hpp/.ipp)
- [ ] Extract common search functionality (both algorithms share similar structure)
- [ ] Consolidate duplicate code in search algorithms

### Template Design
- [ ] Extract search algorithm interfaces (common base template)
- [ ] Simplify template parameter lists in search methods
- [ ] Use template aliases for complex types
- [ ] Consider CRTP pattern for search algorithm polymorphism

### Header Structure
- [ ] Further separate interface/implementation
- [ ] Optimize include dependencies
- [ ] Create forward declaration headers

---

## 🟢 Priority 4: Performance Improvements

### Data Structure Optimizations
- [ ] Replace `std::list<vertex_iterator>` with `std::vector` for vertices_from
- [ ] Implement hash-based edge lookup instead of linear search
- [ ] Consider using flat_map for small vertex sets

### Algorithm Efficiency
- [ ] Optimize GetAllEdges() to avoid expensive copy (graph_impl.hpp:158-167)
- [ ] Improve RemoveVertex complexity from O(m²)
- [ ] Add early termination to search algorithms

### Memory Allocation
- [ ] Implement object pooling for vertex allocations
- [ ] Add shrink-to-fit capability for dynamic priority queue
- [ ] Use small-object optimization for edges

---

## 📘 Priority 5: Modernization (C++14+ features)

### Smart Pointers
- [ ] Migrate raw pointers to std::unique_ptr/std::shared_ptr
- [ ] Use std::make_unique for exception safety
- [ ] Implement weak_ptr for cycle prevention

### Modern C++ Features
- [ ] Add constexpr for compile-time constants
- [ ] Use final specifier on non-inheritable classes
- [ ] Implement std::optional for nullable returns
- [ ] Use nullptr consistently instead of NULL/0

---

## 🔍 Priority 6: Missing Features

### Core Graph Algorithms
- [ ] Implement BFS (Breadth-First Search)
- [ ] Implement DFS (Depth-First Search)  
- [ ] Add topological sort
- [ ] Implement Kruskal's and Prim's algorithms for MST
- [ ] Add cycle detection algorithm
- [ ] Implement connected components detection

### Advanced Search Algorithms
- [ ] Implement bidirectional search
- [ ] Add Jump Point Search (JPS)
- [ ] Implement D* Lite for dynamic pathfinding

### Utility Features
- [ ] Add graph serialization (JSON/XML)
- [ ] Implement DOT format export for visualization
- [ ] Add GraphML support
- [ ] Implement graph metrics (diameter, radius, centrality)

---

## 📖 Priority 7: Documentation & Build System

### Documentation
- [ ] Add comprehensive inline documentation
- [ ] Document time/space complexity for all operations
- [ ] Create getting started guide
- [ ] Add API reference with examples

### Build System & CI/CD
- [ ] Add CMake presets
- [ ] Add clang-tidy integration
- [ ] Implement cppcheck in CI
- [ ] Add valgrind memory checks
- [ ] Add compatibility testing across compilers

---

## ✅ Major Achievements Completed

### Architecture & Refactoring
- ✅ **Independent Edge/Vertex Classes** - Moved from nested to independent template classes
- ✅ **Interface/Implementation Separation** - Clean header interfaces, implementations in separate files
- ✅ **Iterator System** - Proper iterator implementations with const-correctness
- ✅ **Backward Compatibility** - Type aliases maintain existing API

### Testing Infrastructure  
- ✅ **137 Comprehensive Tests** - Memory management, thread safety, parameterized state types
- ✅ **Error Condition Testing** - 14 tests for invalid inputs and edge cases
- ✅ **Independent Class Testing** - 15 tests validating Edge/Vertex separation
- ✅ **Parameterized State Type Testing** - 42 tests across value/pointer/shared_ptr types

### Critical Bug Discovery
- ✅ **Implementation Issues Identified** - 3 critical bugs found through comprehensive testing
- ✅ **Test Safety Net** - All edge cases and invalid operations thoroughly tested
- ✅ **State Type Support Validated** - Full shared_ptr support confirmed and documented

---

## 🎯 State Type Support

### ✅ Fully Supported (with Default Indexer)
```cpp
Graph<MyState> value_graph;                    // Direct object storage
Graph<MyState*> pointer_graph;                 // Raw pointer storage  
Graph<std::shared_ptr<MyState>> smart_graph;   // Shared ownership
```

### DefaultIndexer Capabilities
Automatically detects and supports:
- `state.GetId()` / `state->GetId()` member function
- `state.id` / `state->id` member variable
- `state.id_` / `state->id_` member variable

---

## 📊 Current Statistics

### Test Coverage
- **Total Tests**: 137 (originally 43)
- **Test Files**: 17 
- **Test Suites**: 18 (including parameterized types)
- **Coverage**: ~90% for core operations, memory management, and state types

### Progress Tracking
- **Critical Issues**: 3 identified (exception safety, assignment operator, thread safety)
- **Architectural Goals**: ✅ Completed (Edge/Vertex separation, interface/implementation)  
- **Testing Goals**: ✅ Exceeded (137 vs 100+ target)
- **State Type Goals**: ✅ Completed (all types fully supported)

---

## 🎯 Next Actions

1. **Fix Exception Safety Bug** - Highest priority for production use
2. **Fix Copy Assignment Issue** - Important for correct behavior  
3. **Consider Thread Safety** - If concurrent use is needed
4. **Continue Performance Optimization** - Once critical issues resolved
5. **Add Missing Graph Algorithms** - Feature expansion

---

## Known Limitations

- [ ] A* and Dijkstra algorithms assume double type cost (generic costs need proper comparator)
- [ ] Update edges_to and vertices_from data structures for higher efficiency removal  
- [*] Dynamic priority queue improvements needed
- [*] Convenience functions for vertex information access could be added

**Note**: Previous limitations regarding `std::shared_ptr<T>` state types have been **RESOLVED** ✅