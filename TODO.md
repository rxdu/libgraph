# LibGraph Development TODO

## Current Status

**Test Suite**: 160/160 tests passing (100% success rate)  
**Architecture**: Major refactoring completed with modern C++11 patterns  
**Memory Management**: Migrated to `std::unique_ptr` for automatic RAII  
**Thread Safety**: Concurrent read-only searches fully implemented  
**Exception Safety**: Enhanced with proper exception handling and noexcept specifications  

---

## Priority 1: Core Graph Algorithms

### Essential Missing Algorithms
- [ ] **Breadth-First Search (BFS)** - Foundation for many graph operations
  - Follow SearchContext pattern for thread safety
  - Essential for shortest path in unweighted graphs
  - Basis for level-order traversal and connected components
- [ ] **Depth-First Search (DFS)** - Fundamental traversal algorithm
  - Enable cycle detection and topological sorting
  - Support pre/post-order traversal modes
- [ ] **Connected Components Detection** - Graph connectivity analysis
- [ ] **Cycle Detection** - Essential for DAG validation
- [ ] **Topological Sort** - Dependency ordering for DAGs

### Advanced Algorithms
- [ ] **Minimum Spanning Tree**
  - Kruskal's algorithm
  - Prim's algorithm  
- [ ] **Bidirectional Search** - Optimize pathfinding performance
- [ ] **Jump Point Search (JPS)** - Grid-based pathfinding optimization
- [ ] **D* Lite** - Dynamic pathfinding for changing graphs

---

## Priority 2: Performance Optimizations

### Data Structure Improvements
- [ ] Replace `std::list<vertex_iterator>` with `std::vector` for `vertices_from`
  - Blocked by thread safety requirements
  - Needs reader-writer synchronization first
- [ ] Implement hash-based edge lookup to replace O(n) linear search
- [ ] Optimize `GetAllEdges()` to avoid expensive copy operations
- [ ] Improve `RemoveVertex()` complexity from O(m²) to O(m)
- [ ] Consider flat_map for small vertex sets

### Memory Optimization
- [ ] Implement object pooling for vertex allocations
- [ ] Add shrink-to-fit capability for dynamic priority queue
- [ ] Use small-object optimization for edges

---

## Priority 3: Code Organization & Refactoring

### File Structure
- [ ] Move search algorithms to separate files (AStar.hpp, Dijkstra.hpp)
- [ ] Extract common search functionality into base template
- [ ] Create forward declaration headers to reduce compilation time
- [ ] Optimize include dependencies

### Template Design
- [ ] Simplify template parameter lists in search methods
- [ ] Use template aliases for complex types
- [ ] Consider CRTP pattern for search algorithm polymorphism

---

## Priority 4: C++14+ Modernization

### Language Features (When C++14+ is adopted)
- [ ] Use `std::make_unique` instead of `new` (currently C++11 compatible)
- [ ] Add `constexpr` for compile-time constants
- [ ] Use `final` specifier on non-inheritable classes
- [ ] Implement `std::optional` for nullable returns
- [ ] Adopt `auto` return types where appropriate

### Advanced Features
- [ ] Implement `weak_ptr` support for cycle prevention
- [ ] Add concepts (C++20) for better template constraints
- [ ] Use ranges (C++20) for algorithm improvements

---

## Priority 5: Features & Extensions

### Serialization & Export
- [ ] JSON serialization for graph persistence
- [ ] DOT format export for Graphviz visualization
- [ ] GraphML support for interoperability
- [ ] XML serialization option

### Graph Metrics & Analysis
- [ ] Graph diameter and radius calculation
- [ ] Centrality measures (betweenness, closeness, degree)
- [ ] Clustering coefficient computation
- [ ] Community detection algorithms

### Advanced Thread Safety (Phase 2)
- [ ] Reader-Writer synchronization with `std::shared_mutex`
- [ ] Concurrent graph modifications support
- [ ] Lock-free data structures investigation

---

## Priority 6: Documentation & Tooling

### Documentation
- [ ] Add comprehensive inline documentation for all public APIs
- [ ] Document time/space complexity for all operations
- [ ] Create getting started guide with examples
- [ ] Generate complete API reference

### Build System & CI/CD
- [ ] Add CMake presets for common configurations
- [ ] Integrate clang-tidy for static analysis
- [ ] Add cppcheck to CI pipeline
- [ ] Implement valgrind memory checks
- [ ] Add compiler compatibility matrix testing

---

## Completed Milestones

### Architecture & Design ✅
- Independent Edge/Vertex template classes
- Clean interface/implementation separation
- Proper iterator system with const-correctness
- Full backward compatibility maintained

### Memory Management ✅
- Complete migration to `std::unique_ptr` for vertices
- RAII pattern throughout codebase
- Exception-safe vertex creation
- Automatic cleanup in all scenarios

### Thread Safety ✅
- SearchContext-based external state management
- Thread-safe Dijkstra and A* implementations
- Concurrent read-only graph access
- Performance-optimized context reuse

### API Enhancements ✅
- Comprehensive convenience methods (HasVertex, GetNeighbors, etc.)
- STL-like interface (empty, size, reserve)
- Batch operations (AddVertices, RemoveVertices)
- Range-based for loop support
- Standardized return types for consistency

### Exception Safety ✅
- Replaced assert() with proper exceptions
- Added noexcept specifications
- Documented exception guarantees
- Strong exception safety in critical operations

### Testing Infrastructure ✅
- 160 comprehensive unit tests
- Memory management validation
- Thread safety verification
- Parameterized type testing (value/pointer/shared_ptr)
- Edge case and error condition coverage

---

## Known Limitations

- A* and Dijkstra assume `double` type costs (generic comparator needed)
- Dynamic priority queue could benefit from performance improvements
- Concurrent write operations remain intentionally unsupported (Phase 2)
- Template error messages could be improved with better constraints

---

## Recent Updates

- Enhanced exception safety with proper error handling
- Added noexcept specifications for performance
- Attempted vector optimization (reverted due to thread safety)
- Comprehensive documentation improvements
- API consistency and standardization completed