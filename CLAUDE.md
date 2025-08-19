# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

libgraph is a modern, header-only C++11 library for graph construction and pathfinding algorithms. It provides high-performance graph operations with thread-safe concurrent searches and support for generic cost types. The library implements a Graph class using an adjacency list representation with O(m+n) space complexity and provides a unified search framework with A*, Dijkstra, BFS, and DFS algorithms.

## Build Commands

### Basic Build
```bash
mkdir build && cd build
cmake ..
cmake --build .
```

### Build with Tests
```bash
mkdir build && cd build
cmake -DBUILD_TESTING=ON ..
cmake --build .
```

### Build with Coverage
```bash
mkdir build && cd build
cmake -DBUILD_TESTING=ON -DCOVERAGE_CHECK=ON ..
cmake --build .
```

### Run Tests
```bash
cd build
make test
# or run directly
./bin/utests
```

### Generate Documentation
```bash
cd docs
doxygen doxygen/Doxyfile
```

### Create Installation Package
```bash
cd build
cpack  # Creates .deb package
```

## Code Architecture

### Core Components

The library is organized around three main template classes in the `xmotion` namespace:

1. **Graph<State, Transition, StateIndexer>** (`src/include/graph/graph.hpp`)
   - Main graph class using adjacency list representation
   - Uses `std::unordered_map<int64_t, Vertex*>` for vertex storage
   - Each vertex contains `std::list<Edge>` for edges
   - Supports directed and undirected graphs
   - Provides vertex/edge iterators for traversal

2. **Tree<State, Transition, StateIndexer>** (`src/include/graph/tree.hpp`)
   - Specialized graph structure for tree representations
   - Enforces tree properties (single parent, no cycles)

3. **Search Algorithms** (`src/include/graph/search/`)
   - `AStar`: A* pathfinding with custom heuristics
   - `Dijkstra`: Shortest path algorithm for weighted graphs
   - `BFS`: Breadth-first search for unweighted shortest paths
   - `DFS`: Depth-first search for graph traversal
   - All algorithms use unified framework with `SearchContext` for thread-safe concurrent searches
   - Dynamic priority queue implementation for efficient priority updates

### State Indexing System

The library uses a StateIndexer functor to generate unique indices for graph vertices:
- **DefaultIndexer** (`src/include/graph/impl/default_indexer.hpp`): Automatically works with states that have `GetId()`, `id_`, or `id`
- Custom indexers can be defined by implementing `operator()(State)` returning `int64_t`

### Priority Queue Implementation

The search algorithms rely on specialized priority queues:
- **PriorityQueue** (`src/include/graph/impl/priority_queue.hpp`): Basic priority queue
- **DynamicPriorityQueue** (`src/include/graph/impl/dynamic_priority_queue.hpp`): Supports priority updates, crucial for efficient graph searches

## Testing Structure

- **Unit Tests** (`tests/unit_test/`): Core functionality tests using Google Test
  - Graph construction, modification, iteration
  - Search algorithm correctness
  - Priority queue operations
  - Tree operations
- **Development Tests** (`tests/devel_test/`): Performance and specialized tests

## Important Implementation Details

- The library is header-only; all implementation is in `.hpp` files
- Graph vertices are stored as pointers in an unordered_map for O(1) average access
- Edge lists use std::list for O(1) insertion
- The library exports as `xmotion::graph` when installed via CMake
- Namespace `xmotion` is used throughout to avoid naming conflicts
- Thread-safe concurrent searches using external `SearchContext`
- Support for custom cost types with `CostTraits` specialization
- RAII memory management with `std::unique_ptr` for exception safety
- Modern C++ design patterns including CRTP for zero-overhead polymorphism

## Documentation Structure

### Core Documentation (docs/)
- **getting_started.md**: 20-minute tutorial from installation to first working graph
- **api.md**: Complete API reference covering all 21 header files
- **architecture.md**: In-depth system design, template patterns, and implementation details
- **advanced_features.md**: Custom costs, thread safety, performance optimization
- **search_algorithms.md**: Comprehensive guide to A*, Dijkstra, BFS, DFS with examples
- **real_world_examples.md**: Industry applications across gaming, robotics, GPS, networks

### Tutorial Series (docs/tutorials/)
- Progressive learning path from basic to advanced usage
- Hands-on examples with complete working code
- **01-basic-graph.md**: Fundamental operations
- **02-pathfinding.md**: Search algorithms
- **03-state-types.md**: Custom states and indexing

### Supporting Documentation
- **README.md**: Professional project overview with quick start
- **index.md**: Documentation homepage for Doxygen integration
- **doxygen/mainpage.md**: Main page for API documentation

## Documentation Standards

### File Naming Convention
- Use **underscores** for documentation files (e.g., `getting_started.md`, `advanced_features.md`)
- Maintain consistency across all documentation links

### Content Guidelines
- **Professional tone**: No emojis or casual language in technical documentation
- **Complete code examples**: All code snippets must be compilable and working
- **Progressive complexity**: Start simple, build to advanced concepts
- **Real-world focus**: Emphasize practical applications and use cases
- **Performance awareness**: Include complexity analysis and optimization guidance

### Cross-Reference Standards
- Link to related sections using relative paths
- Maintain up-to-date cross-references between documentation files
- Include file:line_number references for code locations when relevant

## Sample Code Structure

### Working Examples (sample/)
- **simple_graph_demo.cpp**: Basic graph construction and pathfinding
- **thread_safe_search_demo.cpp**: Concurrent search demonstrations
- **lexicographic_cost_demo.cpp**: Multi-criteria optimization with custom cost types
- **tuple_cost_demo.cpp**: std::tuple-based automatic lexicographic comparison
- **incremental_search_demo.cpp**: Dynamic pathfinding scenarios

### Code Quality Standards
- All sample code must compile and run successfully
- Include comprehensive error handling and validation
- Demonstrate best practices for memory management and thread safety
- Provide clear comments explaining design decisions and usage patterns