/* 
 * graph.hpp
 * 
 * Created on: Dec 9, 2015
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

/* Reference
 *
 * Iterator:
 * [1] https://stackoverflow.com/a/16527081/2200873
 * [2] https://stackoverflow.com/questions/1443793/iterate-keys-in-a-c-map/35262398#35262398
 * 
 * 
 */

#ifndef GRAPH_HPP
#define GRAPH_HPP

#ifndef USE_UNORDERED_MAP
#include <map>
#else
#include <unordered_map>
#endif

#include <vector>
#include <cstdint>
#include <limits>
#include <type_traits>

// #include "graph/details/helper_func.hpp"
#include "graph/details/default_indexer.hpp"

namespace librav
{
/// Graph class template.
template <typename State, typename Transition = double, typename StateIndexer = DefaultIndexer<State>, typename Allocator = std::allocator<State>>
class Graph
{
public:
  class Edge;
  class Vertex;

  using GraphType = Graph<State, Transition, StateIndexer, Allocator>;

#ifndef USE_UNORDERED_MAP
  typedef std::map<int64_t, Vertex *> VertexMapType;
#else
  typedef std::unordered_map<int64_t, Vertex *> VertexMapType;
#endif
  typedef typename VertexMapType::iterator VertexMapTypeIterator;

public:
  /// Vertex iterator for unified access.
  /// Wraps the "value" part of VertexMapType::iterator
  class const_vertex_iterator : public VertexMapTypeIterator
  {
  public:
    const_vertex_iterator() : VertexMapTypeIterator(){};
    const_vertex_iterator(VertexMapTypeIterator s) : VertexMapTypeIterator(s){};

    const Vertex *operator->() const { return (Vertex *const)(VertexMapTypeIterator::operator->()->second); }
    const Vertex &operator*() const { return *(VertexMapTypeIterator::operator*().second); }
  };

  class vertex_iterator : public const_vertex_iterator
  {
  public:
    vertex_iterator() : const_vertex_iterator(){};
    vertex_iterator(VertexMapTypeIterator s) : const_vertex_iterator(s){};

    Vertex *operator->() { return (Vertex *const)(VertexMapTypeIterator::operator->()->second); }
    Vertex &operator*() { return *(VertexMapTypeIterator::operator*().second); }
  };

  vertex_iterator vertex_begin() { return vertex_iterator(vertex_map_.begin()); }
  vertex_iterator vertex_end() { return vertex_iterator(vertex_map_.end()); }
  const_vertex_iterator vertex_begin() const { return vertex_iterator(vertex_map_.begin()); }
  const_vertex_iterator vertex_end() const { return vertex_iterator(vertex_map_.end()); }

  /*---------------------------------------------------------------------------------*/

  struct Edge
  {
    Edge(vertex_iterator src, vertex_iterator dst, Transition c) : src_(src), dst_(dst), cost_(c){};
    ~Edge() = default;

    Edge(const Edge &other) = default;
    Edge &operator=(const Edge &other) = default;
    Edge(Edge &&other) = default;
    Edge &operator=(Edge &&other) = default;

    vertex_iterator src_;
    vertex_iterator dst_;
    Transition cost_;

    /// Returns true if the edge is identical to the other (all src_, dst_, cost_).
    /// Otherwise, return false.
    bool operator==(const Edge &other)
    {
      if (src_ == other.src_ && dst_ == other.dst_ && cost_ == other.cost_)
        return true;
      return false;
    }

    void PrintEdge() { std::cout << "Edge_t: src - " << src_->GetVertexID() << " , dst - " << dst_->GetVertexID() << " , cost - " << cost_ << std::endl; }
  };

  /*---------------------------------------------------------------------------------*/

  /// Vertex class template.
  struct Vertex
  {
    Vertex(State s, int64_t id) : state_(s), vertex_id_(id) {}
    ~Vertex() = default;

    // do not allow copy or assign
    Vertex() = delete;
    Vertex(const State &other) = delete;
    Vertex &operator=(const State &other) = delete;
    Vertex(State &&other) = delete;
    Vertex &operator=(State &&other) = delete;

    // generic attributes
    State state_;
    int64_t vertex_id_;
    int64_t GetVertexID() const { return vertex_id_; }

    // edges connecting to other vertices
    typedef std::vector<Edge> EdgeListType;
    EdgeListType edges_to_;

    // vertices that contain edges connecting to current vertex
    std::vector<vertex_iterator> vertices_from_;

    // attributes for search algorithms
    bool is_checked_ = false;
    bool is_in_openlist_ = false;
    double f_cost_ = std::numeric_limits<double>::max();
    double g_cost_ = std::numeric_limits<double>::max();
    double h_cost_ = std::numeric_limits<double>::max();
    vertex_iterator search_parent_;

    // edge iterator for easy access
    typedef typename EdgeListType::iterator edge_iterator;
    typedef typename EdgeListType::const_iterator const_edge_iterator;
    edge_iterator edge_begin() { return edges_to_.begin(); }
    edge_iterator edge_end() { return edges_to_.end(); }
    const_edge_iterator edge_begin() const { return edges_to_.begin(); }
    const_edge_iterator edge_end() const { return edges_to_.end(); }

    /// Returns true if two vertices have the same id. Otherwise, return false.
    bool operator==(const Vertex &other)
    {
      if (vertex_id_ == other.vertex_id_)
        return true;
      return false;
    }

    // /// Get depth from root, valid when the vertex is used in a tree-like structure
    // std::size_t GetDepth() const;

    // /// Get all neighbor vertices of this vertex.
    // std::vector<Vertex *> GetNeighbours();

    // /// Get IDs of all neighbor vertices of this vertex.
    // std::vector<int64_t> GetNeighbourIDs();

    // /// Get edge cost from current vertex to given vertex id. -1 is returned if no edge between
    // ///		the two vertices exists.
    // Transition GetEdgeCost(int64_t dst_id) const;

    // /// Get edge cost from current vertex to given vertex. -1 is returned if no edge between
    // ///		the two vertices exists.
    // Transition GetEdgeCost(const Vertex *dst_node) const;

    /// Check if a given vertex is the neighbor of current vertex.
    bool CheckNeighbour(int64_t dst_id)
    {
      for (const auto &edge : edges_to_)
      {
        if (edge->dst_->vertex_id_ == dst_id)
          return true;
      }
      return false;
    }

    /// Check if a given vertex is the neighbor of current vertex.
    bool CheckNeighbour(State *dst_node)
    {

    }

    // /// Clear exiting search info before a new search
    // void ClearVertexSearchInfo();
  };

  /*---------------------------------------------------------------------------------*/

public:
  /// Default Graph constructor.
  Graph() {}

  /// Copy constructor.
  Graph(const GraphType &other) {}

  /// Move constructor
  Graph(GraphType &&other) {}

  /// Assignment operator
  GraphType &operator=(const GraphType &other) {}

  /// Move assignment operator
  GraphType &operator=(GraphType &&other) {}

  /// Default Graph destructor.
  /// Graph class is only responsible for the memory recycling of its internal objects, such as
  /// vertices and edges. If a state is associated with a vertex by its pointer, the memory allocated
  //  for the state object will not be managed by the graph and needs to be recycled separately.
  ~Graph(){};

  void AddEdge(State sstate, State dstate, Transition trans)
  {
    auto src_vertex = ObtainVertexFromVertexMap(sstate);
    auto dst_vertex = ObtainVertexFromVertexMap(dstate);

    // if (src_vertex->CheckNeighbour(dst_vertex))
    //   return;

    // store information for deleting vertex
    dst_vertex->vertices_from_.push_back(src_vertex);
    src_vertex->edges_to_.emplace_back(src_vertex, dst_vertex, trans);
  }

  /// This function return the vertex iterator with specified id
  inline vertex_iterator find(int64_t vertex_id)
  {
    return vertex_iterator(vertex_map_.find(vertex_id));
  }

  template <class T = State, typename std::enable_if<!std::is_integral<T>::value>::type * = nullptr>
  inline vertex_iterator find(T state)
  {
    return vertex_iterator(vertex_map_.find(GetStateIndex(state)));
  }

private:
  StateIndexer GetStateIndex;
  VertexMapType vertex_map_;

  friend class Edge;
  friend class Vertex;

  /// Returns the iterator to the pair whose value is "state" in the vertex map.
  /// Create a new pair if one does not exit yet and return the iterator to the
  /// newly created pair.
  vertex_iterator ObtainVertexFromVertexMap(State state)
  {
    int64_t state_id = GetStateIndex(state);
    auto it = vertex_map_.find(state_id);

    if (it == vertex_map_.end())
    {
      auto new_vertex = new Vertex(state, state_id);
      vertex_map_.insert(std::make_pair(state_id, new_vertex));
      return vertex_iterator(vertex_map_.find(state_id));
    }

    return vertex_iterator(it);
  }
};
} // namespace librav

// #include "graph/details/graph_impl.hpp"

#endif /* GRAPH_HPP */
