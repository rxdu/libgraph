# Graph and Search

C++ class templates for constructing graphs and search. This library is distributed under **MIT license**.

## 1. Requirement

* A compiler that supports C++11

This is a header-only library, so you only need to include the header files in the "src/graph" folder to use the data structures. Refer to the demo for more details.

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

The only requirement is that you need to define an index function for your State class. **A default indexer is provided and could be used if you have a member variable "id_" inside the class.** 

The index function is used to generate a unique index for a state and it's necessary for checking whether two given states are the same so that only one vertex is created for one unique state inside the graph.

You can define your own index function if the default one is not suitable for your application.

```
// struct YourStateIndexFunction is a functor that defines operator "()". 
struct YourStateIndexFunction
{
    int64_t operator()(YourStateType* state)
    {
        // Generate <unique-value> with state
        return <unique-value>;
    }

    int64_t operator()(const YourStateType& state)
    {
        // Generate <unique-value> with state
        return <unique-value>;
    }
};
```

Note:

* The index function takes a state as input and returns a unique index value. 
* You need to make sure the input parameter type is compatible with your graph type. For example, if you want to create a graph:
    ```
    Graph<YourState*, double, YourStateIndexFunction> graph;
    ```
    then the "State" type is "YourState*". Your indexer function should be readily accept a pointer to a YourState object, with a signature "int64_t operator()(YourStateType* state)".
    Finally inside the graph, it will be used like:
    ```
    YourStateIndexFunction get_index;
    int64_t id = get_index(state_ptr);
    ```

See "simple_graph_demo.cpp" in "demo" folder for a working example.

## 4. Known Limitations

* [TODO List](./TODO.md)

