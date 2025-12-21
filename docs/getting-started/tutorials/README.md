# libgraph Tutorial Series

A progressive learning path from basic graphs to advanced features.

## Available Tutorials

### **Beginner Level**
1. **[Basic Graph Operations](01-basic-graph.md)** - Your first graph and fundamental operations
2. **[Simple Pathfinding](02-pathfinding.md)** - Using Dijkstra and A* for shortest paths

## Tutorial Goals

Each tutorial builds on previous concepts while introducing new features:

- **Hands-on examples** you can run immediately
- **Progressive complexity** from basic to advanced use cases
- **Real-world applications** showing practical usage patterns
- **Best practices** for performance and maintainability
- **Common pitfalls** and how to avoid them

## Prerequisites

- **Basic C++ knowledge** (classes, templates, STL containers)
- **CMake basics** for building examples
- **Familiarity with graph concepts** (vertices, edges, paths)

## Quick Setup

Before starting the tutorials, set up your environment:

```bash
# Clone and build the library
git clone https://github.com/rxdu/libgraph.git
cd libgraph
mkdir build && cd build
cmake -DBUILD_TESTING=ON ..
cmake --build .

# Run the first example to verify setup
./bin/simple_graph_demo
```

## Tutorial Format

Each tutorial follows a consistent structure:

1. **Learning Objectives** - What you'll accomplish
2. **Complete Example** - Working code you can run
3. **Step-by-Step Explanation** - How each part works
4. **Key Concepts** - Important principles to remember
5. **Exercises** - Practice problems to reinforce learning
6. **Next Steps** - Preview of upcoming topics

---

## Additional Resources

For more advanced topics, see the main documentation:

- **[Quick Start Guide](../quick-start.md)** - Quick introduction and installation
- **[API Reference](../../design/api.md)** - Detailed class and method documentation
- **[Search Algorithms](../../design/search_algorithms.md)** - In-depth algorithm coverage
- **[Architecture](../../design/architecture.md)** - System design and patterns
- **[Advanced Features](../../guides/advanced_features.md)** - Custom cost types, thread safety
- **[Performance](../../guides/performance.md)** - Benchmarking and optimization
- **[Examples](../../guides/examples.md)** - Real-world applications

---

**Ready to start?** Begin with **[Tutorial 1: Basic Graph Operations](01-basic-graph.md)**
