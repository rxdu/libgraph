/*
 * graph.hpp
 *
 * Created on: Dec 9, 2015
 * Description:
 *
 * Major Revisions:
 *  version 0.1 Dec 09, 2015
 *  version 1.0 Sep 03, 2018
 *
 * Copyright (c) 2015-2021 Ruixiang Du (rdu)
 */

/* Reference
 *
 * Iterator:
 * [1] https://stackoverflow.com/a/16527081/2200873
 * [2]
 * https://stackoverflow.com/questions/1443793/iterate-keys-in-a-c-map/35262398#35262398
 *
 * Erase–remove idiom:
 * [3] https://en.wikipedia.org/wiki/Erase%E2%80%93remove_idiom
 *
 */

#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <unordered_map>

#include <list>
#include <vector>
#include <cstdint>
#include <limits>
#include <algorithm>
#include <type_traits>

#include "graph/details/default_indexer.hpp"

namespace rdu {
/// Graph class template.
template <typename State, typename Transition = double,
          typename StateIndexer = DefaultIndexer<State>>
class Graph {
 public:
  class Edge;
  class Vertex;
  using GraphType = Graph<State, Transition, StateIndexer>;

  typedef std::unordered_map<int64_t, Vertex *> VertexMapType;
  typedef typename VertexMapType::iterator VertexMapTypeIterator;

 public:
  /*---------------------------------------------------------------------------------*/
  /*                              Vertex Iterator */
  /*---------------------------------------------------------------------------------*/
  ///@{
  /// Vertex iterator for unified access.
  /// Wraps the "value" part of VertexMapType::iterator
  class const_vertex_iterator : public VertexMapTypeIterator {
   public:
    const_vertex_iterator() : VertexMapTypeIterator(){};
    explicit const_vertex_iterator(VertexMapTypeIterator s)
        : VertexMapTypeIterator(s){};

    const Vertex *operator->() const {
      return (Vertex *const)(VertexMapTypeIterator::operator->()->second);
    }
    const Vertex &operator*() const {
      return *(VertexMapTypeIterator::operator*().second);
    }
  };

  class vertex_iterator : public const_vertex_iterator {
   public:
    vertex_iterator() : const_vertex_iterator(){};
    explicit vertex_iterator(VertexMapTypeIterator s)
        : const_vertex_iterator(s){};

    Vertex *operator->() {
      return (Vertex *const)(VertexMapTypeIterator::operator->()->second);
    }
    Vertex &operator*() { return *(VertexMapTypeIterator::operator*().second); }
  };
  ///@}

  /*---------------------------------------------------------------------------------*/
  /*                               Edge Template */
  /*---------------------------------------------------------------------------------*/
  ///@{
  /// Edge class template.
  struct Edge {
    Edge(vertex_iterator src, vertex_iterator dst, Transition c)
        : src(src), dst(dst), cost(c){};

    vertex_iterator src;
    vertex_iterator dst;
    Transition cost;

    /// Check if current edge is identical to the other (all src, dst, cost).
    bool operator==(const Edge &other);

    /// Print edge information, assuming member "cost" is printable.
    void PrintEdge();
  };
  ///@}

  /*---------------------------------------------------------------------------------*/
  /*                              Vertex Template */
  /*---------------------------------------------------------------------------------*/
  ///@{
  /// Vertex class template.
  struct Vertex {
    /** @name Big Five
     *  Edge iterators to access vertices in the graph.
     */
    ///@{
    Vertex(State s, int64_t id) : state(s), vertex_id(id) {}
    ~Vertex() = default;

    // do not allow copy or assign
    Vertex() = delete;
    Vertex(const State &other) = delete;
    Vertex &operator=(const State &other) = delete;
    Vertex(State &&other) = delete;
    Vertex &operator=(State &&other) = delete;
    ///@}

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

    /** @name Edge access.
     *  Edge iterators to access vertices in the graph.
     */
    ///@{
    // edge iterator for easy access
    typedef typename EdgeListType::iterator edge_iterator;
    typedef typename EdgeListType::const_iterator const_edge_iterator;
    edge_iterator edge_begin() { return edges_to.begin(); }
    edge_iterator edge_end() { return edges_to.end(); }
    const_edge_iterator edge_begin() const { return edges_to.cbegin(); }
    const_edge_iterator edge_end() const { return edges_to.cend(); }
    ///@}

    /** @name Edge Operations
     *  Modify or query edge information of the vertex.
     */
    ///@{
    /// Returns true if two vertices have the same id. Otherwise, return false.
    bool operator==(const Vertex &other);

    /// Returns the id of current vertex.
    int64_t GetVertexID() const { return vertex_id; }

    /// Look for the edge connecting to the vertex with give id.
    edge_iterator FindEdge(int64_t dst_id);

    /// Look for the edge connecting to the vertex with give state.
    template <
        class T = State,
        typename std::enable_if<!std::is_integral<T>::value>::type * = nullptr>
    edge_iterator FindEdge(T dst_state);

    /// Check if the vertex with given id or state is a neighbour of current
    /// vertex.
    template <typename T>
    bool CheckNeighbour(T dst);

    /// Get all neighbor vertices of this vertex.
    std::vector<vertex_iterator> GetNeighbours();

    /// Clear exiting search info before a new search
    void ClearVertexSearchInfo();
  };
  ///@}

  /*---------------------------------------------------------------------------------*/
  /*                               Graph Template */
  /*---------------------------------------------------------------------------------*/
 public:
  /** @name Big Five
   *  Constructor, copy/move constructor, copy/move assignment operator,
   * destructor.
   */
  ///@{
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
  /// Graph class is only responsible for the memory recycling of its internal
  /// objects, such as vertices and edges. If a state is associated with a
  /// vertex by its pointer, the memory allocated
  //  for the state object will not be managed by the graph and needs to be
  //  recycled separately.
  ~Graph();
  ///@}

  /** @name Vertex Access
   *  Vertex iterators to access vertices in the graph.
   */
  ///@{
  vertex_iterator vertex_begin() {
    return vertex_iterator{vertex_map_.begin()};
  }
  vertex_iterator vertex_end() { return vertex_iterator{vertex_map_.end()}; }
  const_vertex_iterator vertex_begin() const {
    return const_vertex_iterator{vertex_map_.begin()};
  }
  const_vertex_iterator vertex_end() const {
    return const_vertex_iterator{vertex_map_.end()};
  }
  ///@}

  /** @name Edge Access
   *  Edge iterators to access edges in the vertex.
   */
  ///@{
  typedef typename Vertex::edge_iterator edge_iterator;
  typedef typename Vertex::const_edge_iterator const_edge_iterator;
  ///@}

  /** @name Graph Operations
   *  Modify vertex or edge of the graph.
   */
  ///@{
  /// This function is used to create a vertex in the graph that associates with
  /// the given node.
  vertex_iterator AddVertex(State state);

  /// This function checks if a vertex exists in the graph and remove it if
  /// presents.
  void RemoveVertex(int64_t state_id);

  template <class T = State, typename std::enable_if<
                                 !std::is_integral<T>::value>::type * = nullptr>
  void RemoveVertex(T state) {
    RemoveVertex(GetStateIndex(state));
  }

  /// This function is used to add an edge between the vertices associated with
  /// the given two states. Update the transition if edge already exists.
  void AddEdge(State sstate, State dstate, Transition trans);

  /// This function is used to remove the directed edge from src_node to
  /// dst_node.
  bool RemoveEdge(State sstate, State dstate);

  /* Undirected Graph */
  /// This function is used to add an undirected edge connecting two nodes
  void AddUndirectedEdge(State sstate, State dstate, Transition trans);

  /// This function is used to remove the edge from src_node to dst_node.
  bool RemoveUndirectedEdge(State sstate, State dstate);

  /// This functions is used to access all edges of a graph
  std::vector<edge_iterator> GetAllEdges() const;

  /// This function return the vertex iterator with specified id
  inline vertex_iterator FindVertex(int64_t vertex_id) {
    return vertex_iterator{vertex_map_.find(vertex_id)};
  }

  /// This function return the vertex iterator with specified state
  template <class T = State, typename std::enable_if<
                                 !std::is_integral<T>::value>::type * = nullptr>
  inline vertex_iterator FindVertex(T state) {
    return vertex_iterator{vertex_map_.find(GetStateIndex(state))};
  }

  /// Get total number of vertices in the graph
  int64_t GetTotalVertexNumber() const { return vertex_map_.size(); }

  /// Get total number of edges in the graph
  int64_t GetTotalEdgeNumber() const { return GetAllEdges().size(); }

  /* Utility functions */
  /// This function is used to reset states of all vertice for a new search
  void ResetAllVertices();

  /// This function removes all edges and vertices in the graph
  void ClearAll();
  ///@}

 protected:
  /** @name Internal variables and functions.
   *  Internal variables and functions.
   */
  ///@{
  /// This function returns an index of the give state.
  /// The default indexer returns member variable "id_", assuming it exists.
  StateIndexer GetStateIndex;
  VertexMapType vertex_map_;

  /// Returns the iterator to the pair whose value is "state" in the vertex map.
  /// Create a new pair if one does not exit yet and return the iterator to the
  /// newly created pair.
  vertex_iterator ObtainVertexFromVertexMap(State state);
  ///@}
};

template <typename State, typename Transition = double,
          typename StateIndexer = DefaultIndexer<State>>
using Graph_t = Graph<State, Transition, StateIndexer>;
}  // namespace rdu

#include "graph/details/edge_impl.hpp"
#include "graph/details/vertex_impl.hpp"
#include "graph/details/graph_impl.hpp"

#endif /* GRAPH_HPP */
