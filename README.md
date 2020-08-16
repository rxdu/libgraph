# Graph and Search

![GitHub Workflow Status](https://github.com/rxdu/libgraph/workflows/CI/badge.svg)

C++ class templates for constructing graphs and search. This library is distributed under **MIT license**.

## 1. Requirement

* A compiler that supports C++11

This is a header-only library. You can simply copy the "graph" folder to your project and include "graph/graph.hpp" to use the library. 

If you're using CMake, the recommended way to integrate the library to your project is to add this repository as a git submodule. Then use "add_subdirectory()" in your CMakeLists.txt to add the library to your build tree.

## 2. Build the demo & pack the library

A ".deb" installation package can be generated if you want to install the library to your system. Relevant CMake configuration files will also be installed so that you can easily use "find_package()" command to find and use the library in your project. Note that the library is exported as "rxdu::graph" to avoid naming conflicts with other libraries.

```
$ git clone --recursive https://github.com/rxdu/libgraph.git
$ mkdir build && cd build
$ cmake --build .
$ cpack
```

Demo programs will be built and put into "build/bin" folder. A ".deb" package would be generated inside "build" folder. Install the library with the ".deb" package using command "dpkg", for example

```
$ sudo dpkg -i libgraph_0.1.1_amd64.deb
```

## 3. Build document

You need to have doxygen to build the document.

```
$ sudo apt-get install doxygen
$ cd docs
$ doxygen doxygen/Doxyfile
```

Outlines of core data structures for the purpose of API reference are given at https://rdu.im/libgraph/ .

## 4. Construct a graph

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

## 5. Known limitations

* [TODO List](./TODO.md)
