# libgraph Tutorial Series

A progressive learning path from basic graphs to advanced features.

## Learning Path

### **Beginner Level**
1. **[Basic Graph Operations](01-basic-graph.md)** - Your first graph and fundamental operations
2. **[Simple Pathfinding](02-pathfinding.md)** - Using Dijkstra and A* for shortest paths
3. **[Working with Different State Types](03-state-types.md)** - Custom states and indexing

### **Intermediate Level**  
4. **[Custom Cost Types](04-custom-costs.md)** - Multi-criteria optimization and lexicographic costs
5. **[Thread-Safe Searches](05-thread-safety.md)** - Concurrent pathfinding with SearchContext
6. **[Performance Optimization](06-performance.md)** - Pre-allocation, batch operations, and profiling

### **Advanced Level**
7. **[Grid-Based Pathfinding](07-grid-pathfinding.md)** - 2D/3D grids for games and robotics
8. **[Real-World Applications](08-applications.md)** - GPS navigation, game AI, network analysis
9. **[Extending the Library](09-extensions.md)** - Custom algorithms and advanced patterns

---

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
6. **Next Steps** - Preview of upcoming tutorials

---

## Additional Resources

- **[Getting Started Guide](../getting_started.md)** - Quick introduction and installation
- **[Complete API Reference](../api.md)** - Detailed class and method documentation  
- **[Architecture Overview](../architecture.md)** - System design and patterns
- **[Performance Testing](../performance_testing.md)** - Benchmarking and optimization

---

**Ready to start?** Begin with **[Tutorial 1: Basic Graph Operations](01-basic-graph.md)**