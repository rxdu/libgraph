## API

Outlines of core data structures are given at this [page](./api). The main purpose of that page is to provide an API reference. Some C++ details are removed for brevity. Get more information of the actual implementation from the doxygen documentation.

## Design

Graph is a type of data structure that can be used to represent pairwise relations between entities. A graph $G$ contains a collection of vertices $V$ and edges $E$, of which an edge corresponds to a connectivity relation and a vertex corresponds to an entity. A matrix or adjacency list is commonly used to implement a graph. In this library, an object-oriented implementation is used for efficient access to edges of each vertex. The structure is illustrated as follows 

* Graph
  * Vertex $v_1$
    * Edge $e_{11}$ 
    * Edge $e_{12}$
    * ...
  * ...
  * Vertex $v_n$
    * Edge $e_{n1}$ 
    * Edge $e_{n2}$ 
    * ...
    * Edge $e_{nm}$ 

where $G = \{V, E\}$, $V = \{v_1, v_2, ..., v_n\}$, $E = \{E_{v_1}, ..., E_{v_n}\} = \{\{e_{11}, e_{12}, ...\}, ..., \{e_{n1}, e_{n2}, ..., e_{nm}\}\}$. 

In practice, we usually want to associate application-specific data structures to the vertices and edges so that the graph can be meaningful for the application. For example, when we use a graph to represent a square grid, a square cell is associated with a vertex, and a connection between two cells is associated with an edge. Thus we implment the graph as a class template **Graph<State, Transition, StateIndexer>**. We uniquely associate a **State** data structure with a vertex and a **Transition** data structure to an edge. The StateIndexer is used to generate an index for the states so that any state can be uniquely identified in the graph.

## Graph Construction

In the current implementation, "State" has to be defined as a class or struct. If a user-defined State class/struct has a member variable "id_" or "id" and the value is unique for each instance, the default state indexer could be used. Otherwise, you have to provide an indexer in the form of a function or functor. By default, the "Transition" type is "double". Inside the graph, a Vertex has the same ID with the State it's associated with. 

Here is an example showing how to use the templates to construct a graph.

I. We first define a State type we want to use for constructing the graph.

~~~cpp
struct StateExample
{
    StateExample(uint64_t _id):id(_id){};

    int64_t id;
};
~~~

II. Then we can create a few objects of class StateExample

~~~cpp
std::vector<StateExample*> nodes;

// create nodes to be bundled with the graph vertices
for(int i = 0; i < 9; i++) {
	nodes.push_back(new StateExample(i));
~~~

III. Now use those nodes to construct a graph. Note that the graph is of type "Graph<StateExample*, double, DefaultStateIndexer<StateExample*>>" in this example. Since the latter two type parameters use the default types, you only need to explicitly specify the first one.

~~~cpp
// create a graph
Graph<StateExample*> graph;

// we only store a pointer in the graph to avoid copying possibly large data
graph.AddEdge(nodes[0], nodes[1], 1.0);
graph.AddEdge(nodes[0], nodes[2], 1.5);
graph.AddEdge(nodes[1], nodes[2], 2.0);
graph.AddEdge(nodes[2], nodes[3], 2.5);
~~~

IV. Now you've got a graph. You can print all edges of this graph in the following way

~~~cpp
auto all_edges = graph.GetAllEdges();

for(auto e : all_edges)
	e->PrintEdge();
~~~

You will get the output

~~~
Edge: start - 0 , end - 1 , cost - 1
Edge: start - 0 , end - 2 , cost - 1.5
Edge: start - 1 , end - 2 , cost - 2
Edge: start - 2 , end - 3 , cost - 2.5
~~~

You can use iterators to access vertices and edges

~~~cpp
for (auto it = graph.vertex_begin(); it != graph.vertex_end(); ++it)
{
  std::cout << "edges of vertex: " << (*it).vertex_id_ << std::endl;
  
  for (auto ite = it->edge_begin(); ite != it->edge_end(); ++ite)
    std::cout << "edge " << (*ite).dst_->vertex_id_ << std::endl;
}
~~~

## Graph Search

You can use A* and Dijkstra algorithms to perform search in the graph.

~~~cpp
// In order to use A* search, you need to specify how to calculate heuristic
auto path_a = AStar::Search(&graph, 0, 13,
        CalcHeuristicFunc_t<SimpleState *>(CalcHeuristic));
for (auto &e : path_a)
  std::cout << "id: " << e->id << std::endl;

// Dijkstra search
auto path_d = Dijkstra::Search(&graph, 0, 13);
for (auto &e : path_d)
  std::cout << "id: " << e->id << std::endl;
~~~

In cases when it's unnecessary to build the entire graph for a search ,you can use the incremental version of A* and Dijkstra. See **"demo/inc_search_demo.cpp"** for a working example.

## Memory Management

When a Graph object goes out of scope, its destructor function will recycle memory allocated for its vertices and edges. **The graph doesn't recycle memory allocated for the bundled "State" data structure if only a pointer to the State is associated with the vertex in the graph**. In the square grid example, the graph doesn't assume the square grid also becomes useless when the graph itself is destructed. Thus you still have a complete square grid data structure after the graph object goes out of scope. The **square grid** should be responsible for recycling the memory allocated for its square cells when it goes out of scope. Thus in the above simple example, we will need to do the following operation to free the memory at the end.

~~~cpp
// delete objects of StateExample
for(auto& e : nodes)
	delete e;
~~~

It's usually preferred to only associate a pointer to a vertex if it's expensive to copy all data over to the graph or if the attributes of your State may change dynamically and you don't want to synchronize data in the graph manually. In such case, user has to make sure the State data structures don't go out of scope before the destruction of the graph. Otherwise the graph vertices are associated with "nothing" and you will get memory errors.

In other cases, you can copy data to graph vertices and you will get a second copy of your original data in the graph once the graph is created. The data copied to the graph will be managed by the graph. You only need to recycle the original data if necessary.

An detailed example of the graph and path search can be found in "demo/simple_graph_demo.cpp". You may also find the unit tests in "tests/unit_test" useful for other possible operations on the graph.
