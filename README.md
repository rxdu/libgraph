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

### Generic State Types
```cpp
struct GameState {
    int x, y, health, ammo;
    int64_t GetId() const { return y*1000 + x; }  // Auto-detected by DefaultIndexer
};
Graph<GameState> game_world;
```

### Custom Cost Types
```cpp
struct TravelCost {
    double time, distance, comfort;
    bool operator<(const TravelCost& other) const { /* lexicographic comparison */ }
};
Graph<Location, TravelCost> multi_criteria_graph;
```

### Performance Optimization
```cpp
graph.reserve(10000);           // Pre-allocate vertices
context.PreAllocate(10000);     // Pre-allocate search state
graph.AddVertices(state_list);  // Batch operations
```

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

### Thread Safety

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

## Build & Integration

### Requirements
- **C++11** compatible compiler (GCC 4.8+, Clang 3.4+, MSVC 2015+)
- **CMake 3.10+** (for build system and examples)
- **Optional**: Doxygen for API documentation

### Integration Options

#### 1. Header-Only
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

#### 4. Debian Package Installation
```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release .. && make -j4 && cpack
sudo dpkg -i graph_3.0.0_amd64.deb  # Adjust version as needed
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
- **[Advanced Features](docs/advanced_features.md)** - Custom costs, validation, thread safety
- **[Search Algorithms Guide](docs/search_algorithms.md)** - Deep dive into A*, Dijkstra, BFS, DFS

### **For Contributors**
- **[Performance Testing](docs/performance_testing.md)** - Benchmarking and optimization
- **[Thread Safety Design](docs/thread_safety_design.md)** - Concurrent search architecture
- **[Search Framework](docs/search_framework.md)** - Modern strategy pattern implementation

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

**[→ Complete Roadmap & TODO List](TODO.md)**

---

**Built for the C++ community**
