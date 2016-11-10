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

A "Graph" consists of a list of "Vertex", each of which has an unique ID and a list of "Edge". To perform A* search in the graph, we also need to add a few more attributes, such as edge cost, heuristics, flags to according data structures. This part is generic to all graph and A* related application.

In practice, we usually want to associate even more attributes to the vertex so that it can be meaningful for a specific application. For example when we use a graph to represent a square grid (created from a map), a square cell can be modeled as a vertex, and the connectivities of a cell with its neighbour cells can be represented as edges. In this case, a square cell (Vertex) may have attributes such as the coordinates in the grid and the occupancy type (cell filled with obstacle or not). Those attributes can be very different across different applications, thus they are not modeled directly in the "Vertex" data structure. Instead, the "additional information" is grouped into a separate concept (called a **Bundled Data Structure (BDS)** in this design) and we uniquely associate a bundled data structure with a vertex.

### b. Implementation

There are 3 class templates defined: **Graph**, **Vertex**, **Edge**. The use of template enables us to associate different types of "BDS" to a vertex, without modifying the code of the aforementioned 3 classes. In other words, the Graph, Vertex and Edge all have a "type", which is determined by the type of BDS we want to associate with the vertex. With the current implementation, the BDS has to be defined as a class or struct. **A user-defined BDS class/struct has to inherit from the BDSBase class to enable proper graph construction and A* search.** In the graph data structure, the vertex has the same ID with the BDS it's associated with. This is mainly for easy indexing to find one from the other.

Here is an example to use the templates.

I. We first define a BDS type we want to use for constructing the graph.

~~~
struct BDSExample: public BDSBase<BDSExample>
{
	BDSExample(uint64_t id):
		BDSBase<BDSExample>(id){};
	~BDSExample(){};

    // simplest implementation of the function
	double GetHeuristic(const BDSExample& other_struct) const {
		return 0.0;
	}
};
~~~

II. Then we can create a few objects of class BDSExample

~~~
std::vector<BDSExample*> nodes;

// create nodes to be bundled with the graph vertices
for(int i = 0; i < 9; i++) {
	nodes.push_back(new BDSExample(i));
~~~

III. Now use those nodes to construct a graph. Note that the graph is of type BDSExample in this example.

~~~
// create a graph
Graph<BDSExample*> graph;

// the reference is used to access the bundled data structure in a vertex,
//  so you need to pass in an object instead of a pointer
graph.AddEdge(nodes[0], nodes[1], 1.0);
graph.AddEdge(nodes[0], nodes[2], 1.5);
graph.AddEdge(nodes[1], nodes[2], 2.0);
graph.AddEdge(nodes[2], nodes[3], 2.5);
~~~

IV. Now you've got a graph. You can print all edges of this graph in the following way

~~~
auto all_edges = graph.GetGraphEdges();

for(auto e : all_edges)
	e.PrintEdge();
~~~

You will get the output

~~~
Edge: start - 0 , end - 1 , cost - 1
Edge: start - 0 , end - 2 , cost - 1.5
Edge: start - 1 , end - 2 , cost - 2
Edge: start - 2 , end - 3 , cost - 2.5
~~~

### c. Memory Management

When a Graph object goes out of scope, its destructor function will recycle memory allocated for its vertices and edges. **The graph doesn't recycle memory allocated for the bundled data structure if each vertex is only associated with a pointer or a reference to a BDS**. In the square grid example, the graph doesn't assume the square grid also becomes useless when the graph itself is destructed. Thus you still have a complete square grid data structure after the graph object goes out of scope. The **square grid** should be responsible for recycling the memory allocated for its square cells when it becomes of no use. Thus in the above simple example, we will need to do the following operation to free the memory at the end.

~~~
// delete objects of BDSExample
for(auto& e : nodes)
		delete e;
~~~

It's usually preferred to only associate a pointer or a (const) reference to a vertex if it's expensive to copy all data over to the graph or if the attributes of your BDSs may change dynamically and you don't want to synchronize data in the graph manually. In such case, user has to make sure the bundled data structures don't go out of scope before the destruction of the graph. Otherwise the graph vertices are associated with "nothing" and you will get memory errors.

In other cases, you can copy data to graph vertices and treat the graph and the original data as two separate entities. The data copied to a graph will be managed by the graph. You only need to recycle the original data properly.

An detailed example of the graph and path search can be found in "demo/graph_demo.cpp". The basic operations of Graph<BDSExample>, Graph<BDSExample*> and Graph<const BDSExample&> are shown in the demo.

### d. Notes on Graph

* A* performs search on Vertex objects, so the A* algorithm also has a "type". A* is implemented as a template function.
* When constructing a graph, you don't need to explicitly create objects of "Vertex". By calling member function **AddEdge(src_node, dst_node, cost)** of the graph, vertices are created and associated with the according BDS internally.
* There are two views of the graph data structure. When constructing the graph (bottom-up view), the BDSs are manipulated directly and vertices are handled implicitly. When using the graph (top-down view) for path search, vertices are the the entities you're directly interacting with and the BDSs they associate with are probably of less interest. Of course, you can access one from the other easily using their common ID.
