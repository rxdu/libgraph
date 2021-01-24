### **Graph** 

```cpp 
template<typename State, typename Transition, typename StateIndexer>
class Graph {
 public:
    /// Default Graph constructor.
    Graph() = default;
    /// Copy constructor.
    Graph(const GraphType &other);
    /// Move constructor
    Graph(GraphType &&other);
    /// Assignment operator
    GraphType &operator=(const GraphType &other);
    /// Move assignment operator
    GraphType &operator=(GraphType &&other);

    /// Default Graph destructor.
    ~Graph();

    /* Vertex Access */
    vertex_iterator vertex_begin();
    vertex_iterator vertex_end();
    const_vertex_iterator vertex_begin() const;
    const_vertex_iterator vertex_end() const;

    /* Edge Access */
    typedef typename Vertex::edge_iterator edge_iterator;
    typedef typename Vertex::const_edge_iterator const_edge_iterator;

    /* Modify vertex or edge of the graph */
    /// This function is used to create a vertex in the graph that 
    /// associates with the given node.
    vertex_iterator AddVertex(State state);

    /// Removes a vertex if exists.
    void RemoveVertex(int64_t state_id);
    void RemoveVertex(State state);

    /// Add an edge between vertices associated with the given states.   
    /// Update the transition if an edge already exists.
    void AddEdge(State sstate, State dstate, Transition trans);

    /// This function is used to remove the directed edge from 
    /// src_node to dst_node.
    bool RemoveEdge(State sstate, State dstate);

    /* Undirected Graph */
    /// Add an undirected edge connecting two states
    void AddUndirectedEdge(State sstate, State dstate, Transition trans);

    /// Remove the edge from src_node to dst_node.
    bool RemoveUndirectedEdge(State src_node, State dst_node);

    /// This functions is used to access all edges of a graph
    std::vector<edge_iterator> GetAllEdges() const;

    /// This function return the vertex iterator with specified id/state
    inline vertex_iterator FindVertex(int64_t vertex_id);
    inline vertex_iterator FindVertex(T state);

    /// Get total number of vertices in the graph
    int64_t GetGraphVertexNumber() const;

    /// Get total number of edges in the graph
    int64_t GetGraphEdgeNumber() const;

    /* Utility functions */
    /// This function is used to reset states of all vertice for a new search
    void ResetGraphVertices();

    /// This function removes all edges and vertices in the graph
    void ClearGraph();
}
```

**Vertex** 

```cpp
/// Vertex class template.
template<typename State, typename Transition, typename StateIndexer>
struct Vertex {
    // constructor/destructor
    Vertex(State s, int64_t id);
    ~Vertex() = default;

    // copy or assignment not allowed
    Vertex() = delete;
    Vertex(const State &other) = delete;
    Vertex &operator=(const State &other) = delete;
    Vertex(State &&other) = delete;
    Vertex &operator=(State &&other) = delete;

    // generic attributes
    State state;
    const int64_t vertex_id;
    StateIndexer GetStateIndex;

    // edges connecting to other vertices
    typedef std::list<Edge> EdgeListType;
    EdgeListType edges_to;

    // vertices that contain edges connecting to current vertex
    std::list<vertex_iterator> vertices_from;

    // attributes for search algorithms
    bool is_checked = false;
    bool is_in_openlist = false;
    double f_cost = std::numeric_limits<double>::max();
    double g_cost = std::numeric_limits<double>::max();
    double h_cost = std::numeric_limits<double>::max();
    vertex_iterator search_parent;

    // edge iterator for easy access
    edge_iterator edge_begin();
    edge_iterator edge_end();
    const_edge_iterator edge_begin() const;
    const_edge_iterator edge_end() const;

    /// Returns true if two vertices have the same id. 
    bool operator==(const Vertex &other);

    /// Returns the id of current vertex.
    int64_t GetVertexID() const { return vertex_id_; }

    /// Look for the edge connecting to the vertex with give id/state.
    edge_iterator FindEdge(int64_t dst_id);
    edge_iterator FindEdge(T dst_state);

    /// Check if the vertex with given id or state is a neighbour.
    template <typename T>
    bool CheckNeighbour(T dst);

    /// Get all neighbor vertices of this vertex.
    std::vector<vertex_iterator> GetNeighbours();

    /// Clear exiting search info before a new search
    void ClearVertexSearchInfo();
};
```

**Edge**

```cpp
/// Edge class template.
template<typename State, typename Transition, typename StateIndexer>
struct Edge
{
    Edge(vertex_iterator src, vertex_iterator dst, Transition c);
    ~Edge();

    Edge(const Edge &other) = default;
    Edge &operator=(const Edge &other) = default;
    Edge(Edge &&other) = default;
    Edge &operator=(Edge &&other) = default;

    vertex_iterator src;
    vertex_iterator dst;
    Transition cost;

    /// Check if current edge is identical to the other (src_, dst_, cost_).
    bool operator==(const Edge &other);

    /// Print edge information, assuming member "cost_" is printable.
    void PrintEdge();
};
```

**Graph Search Related Types**

```cpp
template <typename State>
using Path = std::vector<State>;

template <typename State, typename Transition = double>
using GetNeighbourFunc_t =
    std::function<std::vector<std::tuple<State, Transition>>(State)>;

template <typename State, typename Transition = double>
using CalcHeuristicFunc_t = std::function<Transition(State, State)>;
```

**Dijkstra**

```cpp
class Dijkstra {
 public:
  /// Search using vertex id or state
  template <typename State, typename Transition, typename StateIndexer,
            typename VertexIdentifier>
  static Path<State> Search(Graph<State, Transition, StateIndexer> *graph,
                            VertexIdentifier start, VertexIdentifier goal);

  /// Incrementally search with start state, goal state and an empty graph
  template <typename State, typename Transition, typename StateIndexer>
  static Path<State> IncSearch(
      Graph<State, Transition, StateIndexer> *graph, State sstate, State gstate,
      GetNeighbourFunc_t<State, Transition> get_neighbours);
};
```

**A\***

```cpp
class AStar {
 public:
  /// Search using vertex id or state
  template <typename State, typename Transition, typename StateIndexer,
            typename VertexIdentifier>
  static Path<State> Search(
      Graph<State, Transition, StateIndexer> *graph, VertexIdentifier start,
      VertexIdentifier goal,
      CalcHeuristicFunc_t<State, Transition> calc_heuristic);

  /// Incrementally search with start state, goal state and an empty graph
  template <typename State, typename Transition, typename StateIndexer>
  static Path<State> IncSearch(
      Graph<State, Transition, StateIndexer> *graph, State sstate, State gstate,
      CalcHeuristicFunc_t<State, Transition> calc_heuristic,
      GetNeighbourFunc_t<State, Transition> get_neighbours);
```