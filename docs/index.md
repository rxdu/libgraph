# libgraph Documentation

## Overview

libgraph is a modern, header-only C++11 library for graph construction and pathfinding algorithms. It provides high-performance graph operations with thread-safe concurrent searches and support for generic cost types.

## Key Features

| Feature | Description |
|---------|-------------|
| **Header-only** | Just include and use, no linking required |
| **Thread-safe** | Concurrent searches via external SearchContext |
| **Generic costs** | Custom cost types with lexicographic comparison |
| **Complete algorithms** | A*, Dijkstra, BFS, DFS with unified API |
| **Production-ready** | Deterministic results, early termination, multi-goal search |

## Where to Start

| I want to... | Go to |
|--------------|-------|
| Get started quickly | [Quick Start Guide](./getting-started/quick-start.md) |
| Learn step-by-step | [Tutorials](./getting-started/tutorials/) |
| Use advanced features | [Advanced Features](./guides/advanced_features.md) |
| Look up API details | [API Reference](./design/api.md) |
| Learn about the design | [Architecture](./design/architecture.md) |
| Understand the algorithms | [Search Algorithms](./design/search_algorithms.md) |

## Documentation Structure

```
docs/
├── getting-started/          # Start here if you're new
│   ├── quick-start.md        # 20-minute introduction
│   └── tutorials/            # Step-by-step learning path
│
├── design/                   # Architecture, API, and internals
│   ├── api.md                # Complete API documentation
│   ├── search_algorithms.md  # A*, Dijkstra, BFS, DFS guide
│   ├── architecture.md       # System design and patterns
│   ├── thread-safety.md      # Concurrent search design
│   └── priority-queue.md     # Priority queue internals
│
└── guides/                   # Advanced usage and examples
    ├── advanced_features.md  # Custom costs, threading, production features
    ├── performance.md        # Benchmarking and optimization
    └── examples.md           # Real-world applications
```

## Quick Example

```cpp
#include "graph/graph.hpp"
#include "graph/search/dijkstra.hpp"

using namespace xmotion;

struct Location {
    int id;
    std::string name;
};

int main() {
    Graph<Location> map;

    Location home{0, "Home"};
    Location work{1, "Work"};
    Location store{2, "Store"};

    map.AddVertex(home);
    map.AddVertex(work);
    map.AddVertex(store);

    map.AddEdge(home, store, 3.5);
    map.AddEdge(store, work, 7.2);
    map.AddEdge(home, work, 12.0);

    // Find optimal path: Home -> Store -> Work (cost: 10.7)
    auto path = Dijkstra::Search(map, home, work);

    return 0;
}
```

## Building Documentation

```bash
cd docs
doxygen doxygen/Doxyfile
# Open docs/doxygen/html/index.html
```

## Getting Help

- [GitHub Issues](https://github.com/rxdu/libgraph/issues) - Bug reports and feature requests
- [Examples](../sample/) - Working code demonstrating various features

## License

MIT License. See [LICENSE](../LICENSE) for details.
