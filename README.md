# Graph and Search

![GitHub Workflow Status](https://github.com/rxdu/libgraph/actions/workflows/ci.yml/badge.svg)
[![codecov](https://codecov.io/gh/rxdu/libgraph/branch/main/graph/badge.svg?token=09RJHODBCK)](https://codecov.io/gh/rxdu/libgraph)

C++ class templates for constructing graphs and search. This library is distributed under **MIT license**.

## 1. Design

Assuming a graph *G = (V, E)* contains *n* vertices and *m* edges:

**Data Structure**

A "Graph" class contains a collection of "Vertex" objects (stored in vertex_map_ of type VertexMapType) and each "Vertex" contains a list of edges (stored in edges_to_ of type EdgeListType) connecting to its adjacent vertices.

|     Type      |          STL Data Structure           |  Internal   |
| :-----------: | :-----------------------------------: | :---------: |
| VertexMapType | std::unordered_map<int64_t, Vertex *> | hash table  |
| EdgeListType  |           std::list\<Edge\>           | linked list |

The overall space complexity of this graph implementation is *O(m+n)*.

**Time Complexity**

|   Operation   |      Time Complexity      |
| :-----------: | :-----------------------: |
|  Find Vertex  | Average O(1), Worst O(n)  |
|  Add Vertex   | Average O(1), Worst O(n)  |
| Remove Vertex | Average O(1), Worst O(n)* |
|   Find Edge   |       Worst  O(m*)        |
|   Add Edge    |        Worst  O(1)        |
|  Remove Edge  |        Worst  O(m^2)        |

* here O(1), O(n) only accounts for the operation to remove the vertex from the vertex map. Additionally it may take up to O(m^2) to remove all edges connected to the vertex both from upstream and downstream. Possible improvement can be made to reduce the O(m^2) complexity by using a different data structure for vertics_from and edges_to lists.
* for a sparse graph, the number of edges that a vertex contains should be much less than m

**Graph Search**

The dynamic priority queue is implemented as a binary heap, thus the time complexity of a graph search is O((m+n)*log(n)).

## 2. Dependencies

* A compiler that supports C++11

This is a header-only library. There are multiple ways you can integrate this library to your project:

1. Simply copy content of the "src" folder to your project and include "graph/graph.hpp". 
2. If you're using CMake, you could integrate the library to your project by adding this repository as a git submodule. Then use "add_subdirectory()" in your CMakeLists.txt to add the library to your build tree.
3. You can build and install this library to your system path and use CMake "find_package(graph REQUIRED)" to find the library and add dependency by using "target_link_libraries(your_app PRIVATE xmotion::graph)".

## 3. Build the demo & pack the library

A ".deb" installation package can be generated if you want to install the library to your system. Relevant CMake configuration files will also be installed so that you can easily use "find_package()" command to find and use the library in your project. Note that the library is exported as "rdu::graph" to avoid naming conflicts with other libraries.

```
$ git clone --recursive https://github.com/rxdu/libgraph.git
$ mkdir build && cd build
$ cmake --build .
$ cpack
```

Demo programs will be built and put into "build/bin" folder. A ".deb" package would be generated inside "build" folder. Install the library with the ".deb" package using command "dpkg", for example

```
$ sudo dpkg -i graph_1.1_amd64.deb
```

## 4. Build document

You need to have doxygen to build the document.

```
$ sudo apt-get install doxygen
$ cd docs
$ doxygen doxygen/Doxyfile
```

Outlines of core data structures for the purpose of API reference are given at https://rdu.im/libgraph/ .

## 5. Construct a graph

You can associate any object to a vertex, but you need to provide an index function for the State struct/class. The index function is used to generate a unique index for a state and it's necessary for checking whether two given states are the same so that only one vertex is created for one unique state inside the graph.

**A default indexer is provided and could be used if you have a member function "GetId()" or a member variable "id_" or "id" that provide you with a unique value of the object.**

You can also define your own index function if the default one is not suitable for your application.

```
// struct YourStateIndexFunction is a functor that defines operator "()". 
struct YourStateIndexFunction
{
    // you should have this one if "State" type 
    // of your Graph is a raw pointer type
    int64_t operator()(YourStateType* state)
    {
        // Generate <unique-value> with state
        return <unique-value>;
    }

    // you should have this one if "State" type 
    // of your Graph is a value type
    int64_t operator()(const YourStateType& state)
    {
        // Generate <unique-value> with state
        return <unique-value>;
    }
};
```

See "simple_graph_demo.cpp" in "demo" folder for a working example.

## 6. Performance Testing

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

## 7. Known limitations

* [TODO List](./TODO.md)
