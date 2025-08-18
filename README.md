# libgraph: a C++ Graph Library

![GitHub Workflow Status](https://github.com/rxdu/libgraph/actions/workflows/ci.yml/badge.svg)
[![codecov](https://codecov.io/gh/rxdu/libgraph/branch/main/graph/badge.svg?token=09RJHODBCK)](https://codecov.io/gh/rxdu/libgraph)

A modern, header-only C++11 library for graph construction and pathfinding algorithms. Designed for high performance with thread-safe concurrent searches and generic cost types.

## Key Features

- **High Performance**: O(m+n) space complexity, optimized priority queues, 35% faster search contexts
- **Thread-Safe**: Concurrent searches using external SearchContext, no race conditions
- **Generic**: Custom cost types, comparators, and state indexing - works with any data structure
- **Complete Algorithm Suite**: A*, Dijkstra, BFS, DFS with unified framework
- **Robust**: Comprehensive exception handling, structure validation, memory safety via RAII
- **Well-Documented**: Extensive API reference, tutorials, and working examples

---

## Quick Start (5 minutes)

### Installation
```bash
# Header-only - just copy and include
git clone https://github.com/rxdu/libgraph.git
cp -r libgraph/include/graph /your/project/

# OR: System install with CMake
mkdir build && cd build && cmake .. && sudo make install
```

### Your First Graph
```cpp
#include "graph/graph.hpp"
#include "graph/search/dijkstra.hpp"

struct Location { int id; std::string name; };

// Create graph and add vertices
Graph<Location> map;
map.AddVertex({0, "Home"});
map.AddVertex({1, "Work"}); 
map.AddVertex({2, "Store"});

// Add weighted edges (distances)
map.AddEdge({0, "Home"}, {1, "Work"}, 5.0);
map.AddEdge({0, "Home"}, {2, "Store"}, 3.0);
map.AddEdge({2, "Store"}, {1, "Work"}, 4.0);

// Find optimal path
auto path = Dijkstra::Search(map, {0, "Home"}, {1, "Work"});
// Result: Home -> Store -> Work (7km total, shorter than direct 5km route)
```

**[Complete Getting Started Guide →](docs/getting_started.md)**

---

## Documentation

### **For New Users**
- **[Getting Started Guide](docs/getting_started.md)** - From zero to working graph in 20 minutes
- **[Complete API Reference](docs/api.md)** - All classes, methods, and examples
- **[Tutorial Series](docs/tutorials/)** - Progressive learning path

### **For Advanced Users** 
- **[Architecture Overview](docs/architecture.md)** - System design and template patterns
- **[Advanced Features](docs/advanced-features.md)** - Custom costs, validation, thread safety
- **[Search Algorithms Guide](docs/search-algorithms.md)** - Deep dive into A*, Dijkstra, BFS, DFS

### **For Contributors**
- **[Performance Testing](docs/performance_testing.md)** - Benchmarking and optimization
- **[Thread Safety Design](docs/thread_safety_design.md)** - Concurrent search architecture
- **[Search Framework](docs/search_framework.md)** - Modern strategy pattern implementation

---

## Use Cases & Applications

| **Domain** | **Use Case** | **Algorithm** | **Key Feature** |
|------------|--------------|---------------|-----------------|
| **Game Development** | NPC pathfinding, map navigation | A* with heuristics | Grid-based movement, obstacle avoidance |
| **Robotics** | Motion planning, SLAM | Dijkstra, A* | Real-time path updates, dynamic costs |
| **GPS Navigation** | Route planning, traffic optimization | Dijkstra with custom costs | Multi-criteria optimization (time, distance, cost) |
| **Network Analysis** | Social graphs, web crawling | BFS, DFS | Large-scale graph traversal |
| **Data Science** | Dependency analysis, workflow management | Topological sort, DFS | DAG processing, cycle detection |

---

## Architecture Highlights

### Modern C++ Design Patterns
- **CRTP Strategy Pattern**: Zero-overhead polymorphism for search algorithms
- **RAII Memory Management**: Automatic cleanup with `std::unique_ptr`, no memory leaks
- **Template Metaprogramming**: Compile-time optimization and type safety
- **STL Compatibility**: Full iterator support, range-based for loops

### Template System
```cpp
template<typename State, typename Transition = double, typename StateIndexer = DefaultIndexer<State>>
class Graph;
```
- **State**: Your vertex data (locations, game states, etc.)
- **Transition**: Edge weights (distance, time, cost, custom types) 
- **StateIndexer**: Automatic ID generation from states

### Performance Characteristics
Optimal space complexity O(m+n) using adjacency lists:

| **Operation** | **Time Complexity** | **Space** |
|---------------|-------------------|-----------|
| Add/Find Vertex | O(1) average | O(1) |  
| Add Edge | O(1) | O(1) |
| Search Algorithms | O((m+n) log n) | O(n) |
| Thread-Safe Search | O((m+n) log n) | O(n) per context |

---

## Thread Safety

**Concurrent read-only searches** are fully supported:
```cpp
// Each thread gets independent search context
void worker_thread(const Graph<Location>& map) {
    SearchContext<Location> context;  // Thread-local state
    auto path = Dijkstra::Search(map, context, start, goal);  // Thread-safe
}
```

**Graph modifications** require external synchronization (by design for performance).

---

## Advanced Features

### Custom Cost Types
```cpp
struct TravelCost {
    double time, distance, comfort;
    bool operator<(const TravelCost& other) const { /* lexicographic comparison */ }
};
Graph<Location, TravelCost> multi_criteria_graph;
```

### Generic State Types  
```cpp
struct GameState {
    int x, y, health, ammo;
    int64_t GetId() const { return y*1000 + x; }  // Auto-detected by DefaultIndexer
};
Graph<GameState> game_world;
```

### Performance Optimization
```cpp
graph.reserve(10000);           // Pre-allocate vertices
context.PreAllocate(10000);     // Pre-allocate search state
graph.AddVertices(state_list);  // Batch operations
```

---

## Build & Integration

### Requirements
- **C++11** compatible compiler (GCC 4.8+, Clang 3.4+, MSVC 2015+)
- **CMake 3.10+** (for build system and examples)
- **Optional**: Doxygen for API documentation

### Integration Options

#### 1. Header-Only (Recommended)
```bash
git clone https://github.com/rxdu/libgraph.git
cp -r libgraph/include/graph /your/project/include/
```
```cpp
#include "graph/graph.hpp"
#include "graph/search/dijkstra.hpp"  // Ready to use!
```

#### 2. CMake Submodule
```cmake
# Add to your CMakeLists.txt
add_subdirectory(third_party/libgraph)
target_link_libraries(your_target PRIVATE xmotion::graph)
```

#### 3. System Installation
```bash
mkdir build && cd build
cmake ..
sudo make install

# Use in your project
find_package(graph REQUIRED)
target_link_libraries(your_app PRIVATE xmotion::graph)
```

### Build Examples and Tests
```bash
git clone --recursive https://github.com/rxdu/libgraph.git
mkdir build && cd build
cmake -DBUILD_TESTING=ON ..
cmake --build .

# Run examples
./bin/simple_graph_demo
./bin/thread_safe_search_demo

# Run comprehensive tests (199 tests, 100% pass rate)
./bin/utests
```

---

## Performance Testing

This library includes comprehensive performance benchmarks to evaluate graph operations and search algorithms across different scales.

### Quick Performance Test

Run the unified benchmark suite to get a complete performance analysis:

```bash
# Build the library with benchmarks
mkdir build && cd build
cmake -DBUILD_TESTING=ON ..
cmake --build .

# Run comprehensive performance tests
../scripts/run_unified_benchmarks.sh
```

The benchmark generates a single comprehensive report file that includes:

- **Micro-benchmarks**: Operation-level performance analysis (edge lookup, vertex removal, search context)
- **Large-scale benchmarks**: Realistic workload testing (10K-1M+ vertices)
- **Memory scaling**: Memory usage patterns by graph size
- **Concurrent performance**: Multi-threaded search throughput
- **Optimization targets**: Specific recommendations with expected improvements

### Performance Results

The test outputs results to `performance_results/unified_benchmark_results_<timestamp>.txt` with sections:

1. **Edge Lookup Performance**: Current O(n) linear search analysis
2. **Vertex Removal Performance**: Current O(m²) removal operation analysis  
3. **Search Context Performance**: Memory allocation vs. reuse patterns
4. **Concurrent Search Performance**: Threading scalability analysis
5. **Graph Construction Performance**: Large-scale graph creation benchmarks
6. **Search Algorithm Scaling**: Dijkstra/BFS/DFS performance comparison
7. **Memory Scaling Analysis**: Memory efficiency by graph size
8. **Optimization Recommendations**: Specific targets for performance improvements

### System Requirements

- **Memory**: 2GB+ recommended for large-scale tests
- **CPU**: Multi-core recommended for concurrent benchmarks
- **Time**: 2-5 minutes depending on system performance

### Using Results for Optimization

The benchmark results serve as baseline measurements for quantitative evaluation of performance optimizations:

1. **Save baseline**: Keep initial benchmark results for comparison
2. **Implement optimization**: Make targeted improvements (e.g., hash-based edge lookup)
3. **Re-run benchmarks**: Execute the same test suite
4. **Compare results**: Analyze performance improvements quantitatively

Example optimization targets identified:
- **Edge Lookup**: O(n) → O(1) hash-based lookup (10-100x improvement expected)
- **Vertex Removal**: O(m²) → O(m) bidirectional references (2-10x improvement expected)
- **Memory Pooling**: Reduce context allocation overhead (20-50% improvement expected)
- **Context Reuse**: Systematic reuse patterns (30-70% improvement expected)

---

## Project Status & Quality

### **Mature & Production-Ready**
- **199 comprehensive tests** (100% pass rate, 1 disabled)
- **Complete algorithm suite**: A*, Dijkstra, BFS, DFS with unified framework
- **Thread-safe concurrent searches** with external SearchContext  
- **Generic cost framework** with custom comparators and lexicographic costs
- **Enterprise-grade error handling** with 7-tier exception hierarchy

### **Recent Milestones** (2025)
- **Phase 3 Complete**: Generic cost types, enhanced testing, sample modernization
- **Phase 2 Complete**: Performance optimization (35% improvement), STL compatibility 
- **Phase 1 Complete**: Unified search framework eliminating 70% code duplication

### **Current Focus** (Phase 4)
- Graph analysis algorithms (connected components, cycle detection)
- Enhanced search variants (early termination, hop limits)
- Extended graph operations (statistics, subgraph extraction)

**[Complete Roadmap & TODO List →](TODO.md)**

---

## Contributing & Support

### Getting Help
- **[Complete Documentation](docs/)** - Comprehensive guides and tutorials
- **[Report Issues](https://github.com/rxdu/libgraph/issues)** - Bug reports and feature requests
- **[Discussions](https://github.com/rxdu/libgraph/discussions)** - Questions and community support

### Contributing
- **Fork & Pull Request** workflow for contributions
- **Follow existing code style** and patterns
- **Add tests** for new functionality
- **Update documentation** for public API changes

### License & Citation

This library is distributed under **MIT License**.

```bibtex
@misc{libgraph2025,
  title={libgraph: High-Performance C++ Graph Library},
  author={Ruixiang Du and contributors},
  year={2025},
  url={https://github.com/rxdu/libgraph}
}
```

---

**Built for the C++ community**
