Main Page                         {#mainpage}
=========

### a. Design

Graph is a type of data structure that can be used to represent pairwise relations between objects. In this library, a graph is modeled as a collection of vertices and edges. The way the data structures are organized is illustrated as follows.

* Graph
  * Vertex 1
    * Edge 1_1
    * Edge 1_2
    * ...
  * Vertex 2
    * Edge 2_1
    * Edge 2_2
    * ...
  * ...
  * Vertex n
    * Edge n_1
    * Edge n_2
    * ...
    * Edge n_m

A "Graph" consists of a list of "Vertex", each of which has an unique ID and a list of "Edge". To perform search (such as A* and Dijkstra) in the graph, we also need to add a few more attributes, such as edge cost, heuristics, flags to corresponding data structures. 

In practice, we usually want to associate even more attributes to the vertex so that it can be meaningful for a specific application. For example, when we use a graph to represent a square grid (created from a map), a square cell can be modeled as a vertex, and the connectivities of a cell with its neighbour cells can be represented as edges. In this case, a square cell (Vertex) may have attributes such as the coordinates in the grid and the occupancy type (cell filled with obstacle or not). Those attributes can be very different across different applications, thus they are not modeled directly in the "Vertex" data structure. Instead, the "additional information" is grouped into a separate concept (called a **State** in this design) and we uniquely associate a state data structure with a vertex. Similarly we can associate a **Transition** data structure to an Edge. By default the **Transition** type is double.

### b. Constructing a Graph

The "Graph" template allows us to associate different types of "State" to a vertex and "Transition" to an edge. In other words, the Graph, Vertex and Edge all have a "type", which is determined by "State" and "Transition" types. Additionally, we pass in the "StateIndexer" as a template type parameter in order to generate ID for "State". With the current implementation, the State has to be defined as a class or struct. If a user-defined State class/struct has a member variable "int64_t id_", the default state indexer could be used. Otherwise, you have to provide one in the form of a function or functor.** Inside the graph, a Vertex has the same ID with the State it's associated with. 

Here is an example to use the templates.

I. We first define a State type we want to use for constructing the graph.

~~~
struct StateExample
{
	StateExample(uint64_t id):id_(id){};

	int64_t id_;
};
~~~

II. Then we can create a few objects of class StateExample

~~~
std::vector<StateExample*> nodes;

// create nodes to be bundled with the graph vertices
for(int i = 0; i < 9; i++) {
	nodes.push_back(new StateExample(i));
~~~

III. Now use those nodes to construct a graph. Note that the graph is of type "Graph<StateExample*, double, DefaultStateIndexer<StateExample*>>" in this example. Since the latter two type parameters use the default types, you only need to explicitly specify the first one.

~~~
// create a graph
Graph<StateExample*> graph;

// we only store a pointer to the bundled data structure in the graph to avoid duplicating possibly large data
graph.AddEdge(nodes[0], nodes[1], 1.0);
graph.AddEdge(nodes[0], nodes[2], 1.5);
graph.AddEdge(nodes[1], nodes[2], 2.0);
graph.AddEdge(nodes[2], nodes[3], 2.5);
~~~

IV. Now you've got a graph. You can print all edges of this graph in the following way

~~~
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

You can also use iterators to access vertices and edges

~~~
for (auto it = graph.vertex_begin(); it != graph.vertex_end(); ++it)
{
  std::cout << "edges of vertex: " << (*it).vertex_id_ << std::endl;
  
  for (auto ite = it->edge_begin(); ite != it->edge_end(); ++ite)
    std::cout << "edge " << (*ite).dst_->vertex_id_ << std::endl;
}
~~~

### c. Graph Search

You can use A* and Dijkstra algorithms to perform search in the graph.

~~~
// In order to use A* search, you need to specify how to calculate heuristic
auto path_a = AStar::Search(&graph, 0, 13, CalcHeuristicFunc_t<SimpleState *>(CalcHeuristic));
for (auto &e : path_a)
  std::cout << "id: " << e->id_ << std::endl;

// Dijkstra search
auto path_d = Dijkstra::Search(&graph, 0, 13);
for (auto &e : path_d)
  std::cout << "id: " << e->id_ << std::endl;
~~~

In cases when it's unnecessary to build the entire graph for a search ,you can use the incremental version of A* and Dijkstra. See "demo/inc_search_demo.cpp" for a working example.

### d. Memory Management

When a Graph object goes out of scope, its destructor function will recycle memory allocated for its vertices and edges. **The graph doesn't recycle memory allocated for the bundled "State" data structure if only a pointer to the State is associated with the vertex in the graph**. In the square grid example, the graph doesn't assume the square grid also becomes useless when the graph itself is destructed. Thus you still have a complete square grid data structure after the graph object goes out of scope. The **square grid** should be responsible for recycling the memory allocated for its square cells when it goes out of scope. Thus in the above simple example, we will need to do the following operation to free the memory at the end.

~~~
// delete objects of StateExample
for(auto& e : nodes)
	delete e;
~~~

It's usually preferred to only associate a pointer to a vertex if it's expensive to copy all data over to the graph or if the attributes of your State may change dynamically and you don't want to synchronize data in the graph manually. In such case, user has to make sure the State data structures don't go out of scope before the destruction of the graph. Otherwise the graph vertices are associated with "nothing" and you will get memory errors.

In other cases, you can copy data to graph vertices and you will get a second copy of your original data in the graph once the graph is created. The data copied to the graph will be managed by the graph. You only need to recycle the original data if necessary.

An detailed example of the graph and path search can be found in "demo/simple_graph_demo.cpp". You may also find the unit tests in "tests/gtests" useful for other possible operations on the graph.

### d. Notes on Graph

* When constructing a graph, you don't need to explicitly create objects of "Vertex" for a state. By calling member function **AddEdge(src_node, dst_node, cost)** of the graph, vertices can be created and associated with the according State internally. In certain cases when you want to add a vertex to the graph only, you can use **AddVertex(state)**.
