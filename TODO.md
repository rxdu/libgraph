# LibGraph Improvement TODO List

## ✅ Completed in Latest Session

### Major Refactoring Achievements:
1. **Code Architecture Refactoring** ✅ COMPLETED
   - **Independent Edge and Vertex Classes**: Moved from nested classes to independent template classes
   - **Separate Header Files**: Created `include/graph/edge.hpp` and `include/graph/vertex.hpp`
   - **Maintained Backward Compatibility**: Used type aliases in Graph class
   - **Fixed Circular Dependencies**: Proper forward declarations and iterator type management
   
2. **Interface/Implementation Separation** ✅ COMPLETED
   - **Iterator Implementation Moved**: Moved `const_vertex_iterator` and `vertex_iterator` implementations to `graph_impl.hpp`
   - **Clean Header Interface**: `graph.hpp` now contains only declarations
   - **Better Code Organization**: Clear separation between interface and implementation

3. **Modernization Improvements** ✅ COMPLETED
   - **Modern Type Aliases**: Replaced all `typedef` with `using` declarations
   - **Updated Include Structure**: Reorganized includes with `graph/impl/` path structure
   - **Consistent Naming**: Standardized type naming conventions
   - **Fixed Include Path Consistency**: Standardized all includes to use `graph/impl/` instead of mixed `graph/details/`

### Previous Critical Fixes:
4. **Fixed compilation error**: Changed `vertex_id_` to `vertex_id` in search/common.hpp
5. **Removed debug output**: Commented out `std::cout` in graph_impl.hpp:104
6. **Fixed Vertex constructors**: Corrected parameter types from `State&` to `Vertex&`
7. **Fixed iterator invalidation**: 
   - Used `list::remove_if` with value capture in RemoveVertex
   - Used `list::remove` in RemoveEdge for consistency
8. **Prevented infinite loops in ReconstructPath**:
   - Added cycle detection using `unordered_set`
   - Implemented custom Hash and Equal functors for vertex_iterator
   - Added self-loop detection for uninitialized parents
   - Added const `operator->()` to vertex_iterator for hash operations

All 43 unit tests pass successfully after these changes.

## 🟦 Additional Refactoring Opportunities Identified

### Code Organization Improvements
- [ ] **Move search algorithms to separate files**: Currently AStar and Dijkstra are in `search/` but could benefit from separate `.hpp/.ipp` pattern
- [x] ~~**Standardize include paths**: Some includes use `graph/details/` while others use `graph/impl/` - should be consistent~~ ✅ COMPLETED
- [ ] **Extract common search functionality**: Both AStar and Dijkstra share similar structure, could extract base class
- [ ] **Consolidate duplicate code**: Search algorithms have nearly identical PerformSearch structure

### Template Design Improvements  
- [ ] **Extract search algorithm interfaces**: Create common base template for search algorithms
- [ ] **Simplify template parameter lists**: Long template parameter lists in search methods could be simplified
- [ ] **Use template aliases for complex types**: Reduce verbosity of nested template types
- [ ] **Consider CRTP pattern**: For search algorithm polymorphism without virtual functions

### Header Structure Optimization
- [ ] **Further separate interface/implementation**: Some inline functions could be moved to implementation files
- [ ] **Optimize include dependencies**: Reduce compilation dependencies by minimizing includes in headers
- [ ] **Add header guards consistency**: Ensure all headers follow same guard naming pattern
- [ ] **Create forward declaration headers**: For frequently used but complex types

## 🔴 Critical Issues (Priority 1)

### Memory Management
- [ ] Fix potential memory leaks in vertex creation/deletion (graph_impl.hpp:52,91,177)
- [ ] Replace raw pointer management with RAII pattern
- [ ] Fix exception safety in `ObtainVertexFromVertexMap` (graph_impl.hpp:188)
- [ ] Implement proper cleanup in destructors
- [ ] Add exception-safe vertex allocation

### Compilation Errors
- [x] ~~Fix `vertex_id_` vs `vertex_id` mismatch in search/common.hpp:43~~ ✅ COMPLETED
- [x] ~~Verify all member variable names are consistent~~ ✅ COMPLETED

### Debug Code in Production
- [x] ~~Remove `std::cout` debug statements from graph_impl.hpp:104~~ ✅ COMPLETED (commented out)
- [ ] Implement proper logging interface if debug output is needed

### Iterator Invalidation
- [x] ~~Fix unsafe iterator usage in RemoveVertex (graph_impl.hpp:72-76)~~ ✅ COMPLETED (using list::remove_if with value capture)
- [x] ~~Fix manual iterator manipulation in RemoveEdge (graph_impl.hpp:121-131)~~ ✅ COMPLETED (using list::remove)

### Undefined Behavior
- [x] ~~Fix potential infinite loop in Path reconstruction (common.hpp:35)~~ ✅ COMPLETED (added cycle detection with unordered_set)
- [x] ~~Add cycle detection in parent chain traversal~~ ✅ COMPLETED
- [x] ~~Add self-loop detection for uninitialized parents~~ ✅ COMPLETED

## 🟡 High Priority Issues (Priority 2)

### Thread Safety
- [ ] Add mutex protection for concurrent graph access
- [ ] Make search algorithms thread-safe (currently modify vertex state)
- [ ] Document thread safety guarantees
- [ ] Consider lock-free alternatives for performance-critical paths

### API Consistency
- [x] ~~Fix Vertex copy/move constructor parameter types (should be `const Vertex&`)~~ ✅ COMPLETED (changed from State& to Vertex&)
- [ ] Standardize return types across similar operations
- [x] ~~Add const-correctness to vertex_iterator operator->()~~ ✅ COMPLETED (added const version)
- [ ] Add const-correctness to all other applicable member functions
- [ ] Fix API documentation inconsistencies

### Exception Safety
- [ ] Replace assert() calls with proper exception handling (tree_impl.hpp:49)
- [ ] Define exception safety guarantees for all operations
- [ ] Add noexcept specifications where appropriate
- [ ] Implement error recovery strategies

## 🟢 Performance Improvements (Priority 3)

### Data Structure Optimizations
- [ ] Replace `std::list<vertex_iterator>` with `std::vector` for vertices_from
- [ ] Implement hash-based edge lookup instead of linear search
- [ ] Consider using flat_map for small vertex sets
- [ ] Add capacity hints for known graph sizes

### Algorithm Efficiency
- [ ] Optimize GetAllEdges() to avoid expensive copy (graph_impl.hpp:158-167)
- [ ] Improve RemoveVertex complexity from O(m²)
- [ ] Add early termination to search algorithms
- [ ] Implement lazy evaluation where possible

### Memory Allocation
- [ ] Implement object pooling for vertex allocations
- [ ] Add shrink-to-fit capability for dynamic priority queue
- [ ] Use small-object optimization for edges
- [ ] Consider custom allocators for performance-critical paths

## 📘 Modernization to C++17/20 (Priority 4)

### Smart Pointers
- [ ] Migrate all raw pointers to std::unique_ptr or std::shared_ptr
- [ ] Use std::make_unique for exception safety
- [ ] Implement weak_ptr for cycle prevention
- [ ] Add custom deleters where needed

### Modern C++ Features
- [ ] Add constexpr for compile-time constants
- [ ] Use final specifier on non-inheritable classes
- [ ] Implement std::optional for nullable returns
- [ ] Add structured bindings for tuple returns
- [ ] Use if-constexpr for compile-time branching
- [ ] Add concepts (C++20) for better template constraints

### Language Features
- [x] ~~Replace typedef with using aliases~~ ✅ COMPLETED (all `typedef` replaced with `using`)
- [ ] Use nullptr consistently instead of NULL/0
- [ ] Add [[nodiscard]] attributes
- [ ] Use std::string_view for string parameters
- [ ] Implement three-way comparison operator (C++20)

## 📚 Missing Features (Priority 5)

### Core Graph Algorithms
- [ ] Implement BFS (Breadth-First Search)
- [ ] Implement DFS (Depth-First Search)
- [ ] Add topological sort
- [ ] Implement Kruskal's algorithm for MST
- [ ] Implement Prim's algorithm for MST
- [ ] Add cycle detection algorithm
- [ ] Implement connected components detection
- [ ] Add strongly connected components (for directed graphs)

### Advanced Search Algorithms
- [ ] Implement bidirectional search
- [ ] Add Jump Point Search (JPS)
- [ ] Implement D* Lite for dynamic pathfinding
- [ ] Add Theta* for any-angle pathfinding
- [ ] Implement path smoothing algorithms

### Utility Features
- [ ] Add graph serialization (JSON/XML)
- [ ] Add graph deserialization
- [ ] Implement DOT format export for visualization
- [ ] Add GraphML support
- [ ] Implement graph metrics (diameter, radius, centrality)
- [ ] Add subgraph extraction
- [ ] Implement graph isomorphism checking
- [ ] Add graph union/intersection operations

## 🧪 Testing Improvements (Priority 6)

### Test Coverage
- [ ] Add stress tests for large graphs (>10000 vertices)
- [ ] Implement thread safety tests
- [ ] Add property-based testing with QuickCheck
- [ ] Test edge cases (empty graph, single vertex)
- [ ] Add performance benchmarks
- [ ] Implement fuzz testing for robustness

### Test Infrastructure
- [ ] Add continuous benchmarking
- [ ] Implement test fixtures for common scenarios
- [ ] Add memory leak detection tests
- [ ] Implement code coverage reporting
- [ ] Add static analysis integration

## 📖 Documentation (Priority 7)

### Code Documentation
- [ ] Add comprehensive inline documentation
- [ ] Document time/space complexity for all operations
- [ ] Add usage examples for each major feature
- [ ] Create architecture documentation
- [ ] Add design rationale documentation

### User Documentation
- [ ] Create getting started guide
- [ ] Add API reference with examples
- [ ] Create performance tuning guide
- [ ] Add troubleshooting section
- [ ] Create migration guide for version updates

## 🛠️ Build System (Priority 8)

### CMake Improvements
- [ ] Add CMake presets
- [ ] Implement CPack for multiple package formats
- [ ] Add sanitizer build options
- [ ] Create build matrix for CI
- [ ] Add installation tests

### CI/CD
- [ ] Add clang-tidy integration
- [ ] Implement cppcheck in CI
- [ ] Add valgrind memory checks
- [ ] Implement automated release process
- [ ] Add compatibility testing across compilers

## Known Limitations

- [ ] A* and Dijkstra algorithms currently assume double type cost. Generic type cost with proper comparator defined should also be allowed.
- [x] ~~Refactor iterators and fix const_iterator for Vertex and Edge~~ ✅ COMPLETED (moved implementations to graph_impl.hpp)
- [ ] Update edges_to and vertices_from data structure for higher efficiency removal
- [*] Default indexer doesn't work if State is a std::shared_ptr<T> type
- [*] Dynamic priority queue
- [*] Improve unit test coverage
- [*] Issue: state type cannot be std::shared_ptr<T>
- [*] Convenience functions to access vertex information
- [x] ~~Implement iterators for vertex and edge to unify the accessing interface~~ ✅ COMPLETED (proper iterator implementations)
- [*] Update unit tests for basic function test