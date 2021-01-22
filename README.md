# Graph and Search

![GitHub Workflow Status](https://github.com/rxdu/libgraph/workflows/CI/badge.svg)
[![codecov](https://codecov.io/gh/rxdu/libgraph/branch/master/graph/badge.svg)](https://codecov.io/gh/rxdu/libgraph)

C++ class templates for constructing graphs and search. This library is distributed under **MIT license**.

## 1. Design

Assuming a graph *G = (V, E)* contains *n* vertices and *m* edges:

**Data Structure**

A "Graph" class contains a collection of "Vertex" objects (stored in vertex_map_ of type VertexMapType) and each "Vertex" contains a list of edges (stored in edges_to_ of type EdgeListType) connecting to its adjacent vertices.

|     Type      |          STL Data Structure           |  Internal   |
| :-----------: | :-----------------------------------: | :---------: |
| VertexMapType | std::unordered_map<int64_t, Vertex *> | hash table  |
| EdgeListType  |           std::list\<Edge\>           | linked list |

The overall space complemxity of this graph implementation is $O(m+n)$.

**Time Complexity**

|   Operation   |     Time Complexity      |
| :-----------: | :----------------------: |
|  Find Vertex  | Average O(1), Worst O(n) |
|  Add Vertex   | Average O(1), Worst O(n) |
| Remove Vertex | Average O(1), Worst O(n) |
|   Find Edge   |       Worst  O(d*)       |
|   Add Edge    |       Worst  O(1)        |
|  Remove Edge  |       Worst  O(d)        |

* d is the number of edges contained by the corresponding vertex

## 2. Dependencies

* A compiler that supports C++11

This is a header-only library. There are multiple ways you can integrate this library to your project:

1. Simply copy content of the "src" folder to your project and include "graph/graph.hpp". 
2. If you're using CMake, you could integrate the library to your project by adding this repository as a git submodule. Then use "add_subdirectory()" in your CMakeLists.txt to add the library to your build tree.
3. You can build and install this library to your system path and use CMake "find_package(graph REQUIRED)" to find the library and add dependency by using "target_link_libraries(your_app PRIVATE rdu::graph)".

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
$ sudo dpkg -i libgraph_1.1_amd64.deb
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

**A default indexer is provided and could be used if you have a member variable "id_" or "id" with a unique value inside the class.** 

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

## 6. Known limitations

* [TODO List](./TODO.md)
