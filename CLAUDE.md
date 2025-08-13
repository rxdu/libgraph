# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

libgraph is a header-only C++11 library for constructing graphs and performing graph searches (A*, Dijkstra). It implements a Graph class using an adjacency list representation with O(m+n) space complexity and provides dynamic priority queue implementation for efficient searches.

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
   - `Dijkstra`: Shortest path algorithm
   - Both use `DynamicPriorityQueue` for efficient priority updates

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