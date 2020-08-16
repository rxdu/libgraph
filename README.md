# Graph and Search

![GitHub Workflow Status](https://github.com/rxdu/libgraph/workflows/CI/badge.svg)

C++ class templates for constructing graphs and search. This library is distributed under **MIT license**.

## 1. Requirement

* A compiler that supports C++11

This is a header-only library, so you only need to include the header files in the "**src/graph**" folder to use the data structures. 

The CMake configuration in this repository is only used to compile the demo code and may also serve as an example on how to use this library in a CMake project. Refer to "**src/demo**" for more details. 

## 2. Build Document

You need to have doxygen to build the document.

```
$ sudo apt-get install doxygen
$ cd docs
$ doxygen doxygen/Doxyfile
```

Outlines of core data structures for the purpose of API reference are given at https://rdu.im/libgraph/ .

## 3. Construct a graph

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

## 4. Known Limitations

* [TODO List](./TODO.md)
