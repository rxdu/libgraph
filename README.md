# Graph and Search

C++ class templates for constructing graphs and performing A* search. This library is distributed under **MIT license**.

## 1. Requirement

* A compiler that supports C++11

This is a header-only library, so you only need to include the header files in the "src/libgraph" folder to use the data structures. Refer to the demo for more details.

The CMake configuration in this repository is only used to compile the demo code and may also serve as an example on how to use this library in a CMake project.

## 2. Build Document

You need to have doxygen to build the document.
```
$ sudo apt-get install doxygen
```
```
$ cd docs
$ doxygen doxygen/Doxyfile
```

## 3. Construct a graph

You can associate a state to each vertex and a transition to each edge. 

The only requirement is that you need to define a function inside your state type with the signature

```
int64_t GetUniqueID() const
{
    <generate and return an unique id>
}
```
This function is used to check whether two given states are the same so that only one vertex is created for one unique state inside the graph.

See examples in "demo" folder for more details.

## 4. Known Limitations

* [TODO List](./TODO.md)

