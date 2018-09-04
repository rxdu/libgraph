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
#include <algorithm>
#include <type_traits>

#include "graph/details/default_indexer.hpp"

namespace librav
{
/// Graph class template.
template <typename State, typename Transition = double, typename StateIndexer = DefaultIndexer<State>>
class Graph
{
public:
  class Edge;
  class Vertex;
  using GraphType = Graph<State, Transition, StateIndexer>;

#ifndef USE_UNORDERED_MAP
  typedef std::map<int64_t, Vertex *> VertexMapType;
#else
  typedef std::unordered_map<int64_t, Vertex *> VertexMapType;
#endif
  typedef typename VertexMapType::iterator VertexMapTypeIterator;

public:
  /*---------------------------------------------------------------------------------*/
  /*                             Vertex Iterator                                     */
  /*---------------------------------------------------------------------------------*/

  ///@{
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
  ///@}

  /*---------------------------------------------------------------------------------*/
  /*                              Edge Template                                      */
  /*---------------------------------------------------------------------------------*/
  ///@{
  struct Edge
  {
    Edge(vertex_iterator src, vertex_iterator dst, Transition c) : src_(src), dst_(dst), trans_(c){};
    ~Edge() = default;

    Edge(const Edge &other) = default;
    Edge &operator=(const Edge &other) = default;
    Edge(Edge &&other) = default;
    Edge &operator=(Edge &&other) = default;

    vertex_iterator src_;
    vertex_iterator dst_;
    Transition trans_;

    /// Returns true if the edge is identical to the other (all src_, dst_, trans_).
    /// Otherwise, return false.
    bool operator==(const Edge &other)
    {
      if (src_ == other.src_ && dst_ == other.dst_ && trans_ == other.trans_)
        return true;
      return false;
    }

    void PrintEdge() { std::cout << "Edge_t: src - " << src_->GetVertexID() << " , dst - " << dst_->GetVertexID() << " , cost - " << trans_ << std::endl; }
  };
  ///@}

  /*---------------------------------------------------------------------------------*/
  /*                              Vertex Template                                    */
  /*---------------------------------------------------------------------------------*/
  ///@{
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
    const int64_t vertex_id_;

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

    int64_t GetVertexID() const { return vertex_id_; }

    edge_iterator FindEdge(int64_t dst_id)
    {
      edge_iterator it;
      for (it = edge_begin(); it != edge_end(); ++it)
      {
        if (it->dst_->vertex_id_ == dst_id)
          return it;
      }
      return it;
    }

    template <class T = State, typename std::enable_if<!std::is_integral<T>::value>::type * = nullptr>
    edge_iterator FindEdge(T dst_state)
    {
      edge_iterator it;
      for (it = edge_begin(); it != edge_end(); ++it)
      {
        if (it->dst_->state_ == dst_state)
          return it;
      }
      return it;
    }

    template <typename T>
    bool CheckNeighbour(T dst)
    {
      auto res = FindEdge(dst);
      if (res != edge_end())
        return true;
      return false;
    }

    /// Get all neighbor vertices of this vertex.
    std::vector<vertex_iterator> GetNeighbours()
    {
      std::vector<vertex_iterator> nbs;
      for (auto it = edge_begin(); it != edge_end(); ++it)
        nbs.push_back(it->dst_);
      return nbs;
    }

    /// Clear exiting search info before a new search
    void ClearVertexSearchInfo()
    {
      is_checked_ = false;
      is_in_openlist_ = false;
      search_parent_ = vertex_iterator();

      f_cost_ = 0.0;
      g_cost_ = 0.0;
      h_cost_ = 0.0;
    }
  };

  typedef typename Vertex::edge_iterator edge_iterator;
  typedef typename Vertex::const_edge_iterator const_edge_iterator;
  ///@}

  /*---------------------------------------------------------------------------------*/
  /*                               Graph Template                                    */
  /*---------------------------------------------------------------------------------*/
public:
  /** @name Big Five
   *  Constructor, copy/move constructor, copy/move assignment operator, destructor. 
   */
  ///@{
  /// Default Graph constructor.
  Graph() = default;

  /// Copy constructor.
  Graph(const GraphType &other) 
  {
    for(auto& pair:vertex_map_)
    {
      auto vertex = pair.second;
      for(auto& edge : vertex->edges_to_)
        this->AddEdge(edge.src_->state_, edge.dst_->state_, edge.trans_);
    }
  }

  /// Move constructor
  Graph(GraphType &&other) {}

  /// Assignment operator
  GraphType &operator=(const GraphType &other) 
  {
    GraphType temp = other;
    std::swap(*this, temp);
    return *this;
  }

  /// Move assignment operator
  GraphType &operator=(GraphType &&other) {}

  /// Default Graph destructor.
  /// Graph class is only responsible for the memory recycling of its internal objects, such as
  /// vertices and edges. If a state is associated with a vertex by its pointer, the memory allocated
  //  for the state object will not be managed by the graph and needs to be recycled separately.
  ~Graph()
  {
    for (auto &vertex_pair : vertex_map_)
      delete vertex_pair.second;
  };
  ///@}

  /** @name Vertex Access
   *  Vertex iterators to access vertices in the graph.
   */
  ///@{
  vertex_iterator vertex_begin() { return vertex_iterator(vertex_map_.begin()); }
  vertex_iterator vertex_end() { return vertex_iterator(vertex_map_.end()); }
  const_vertex_iterator vertex_begin() const { return vertex_iterator(vertex_map_.begin()); }
  const_vertex_iterator vertex_end() const { return vertex_iterator(vertex_map_.end()); }
  ///@}

  /** @name Graph Operations
   *  Modify vertex or edge of the graph. 
   */
  ///@{
  /// Create a vertex in the graph that associates with the given node.
  vertex_iterator AddVertex(State state)
  {
    return ObtainVertexFromVertexMap(state);
  }

  /// This function checks if a vertex exists in the graph and remove it if presents.
  void RemoveVertex(int64_t state_id)
  {
    auto it = vertex_map_.find(state_id);

    // remove if specified vertex exists
    if (it != vertex_map_.end())
    {
      // remove from other vertices that connect to the vertex to be deleted
      auto vtx = vertex_iterator(it);
      for (auto &asv : vtx->vertices_from_)
        for (auto eit = asv->edges_to_.begin(); eit != asv->edges_to_.end(); eit++)
          if ((*eit).dst_ == vtx)
          {
            asv->edges_to_.erase(eit);
            break;
          }

      // remove from vertex map
      auto vptr = it->second;
      vertex_map_.erase(it);
      delete vptr;
    }
  }

  template <class T = State, typename std::enable_if<!std::is_integral<T>::value>::type * = nullptr>
  void RemoveVertex(State state)
  {
    int64_t state_id = GetStateIndex(state);
    RemoveVertex(state_id);
  }

  /// Add an edge between the vertices associated with the given two states.
  /// Update the transition if edge already exists.
  void AddEdge(State sstate, State dstate, Transition trans)
  {
    auto src_vertex = ObtainVertexFromVertexMap(sstate);
    auto dst_vertex = ObtainVertexFromVertexMap(dstate);

    // update transition if edge already exists
    auto it = src_vertex->FindEdge(dstate);
    if (it != src_vertex->edge_end())
    {
      it->trans_ = trans;
      return;
    }

    dst_vertex->vertices_from_.push_back(src_vertex);
    src_vertex->edges_to_.emplace_back(src_vertex, dst_vertex, trans);
  }

  /// This function is used to remove the directed edge from src_node to dst_node.
  bool RemoveEdge(State sstate, State dstate)
  {
    auto src_vertex = FindVertex(sstate);
    auto dst_vertex = FindVertex(dstate);

    if ((src_vertex != vertex_end()) && (dst_vertex != vertex_end()))
    {
      for (auto it = src_vertex->edges_to_.begin(); it != src_vertex->edges_to_.end(); ++it)
      {
        if (it->dst_ == dst_vertex)
        {
          src_vertex->edges_to_.erase(it);
          dst_vertex->vertices_from_.erase(std::remove(dst_vertex->vertices_from_.begin(), dst_vertex->vertices_from_.end(), src_vertex), dst_vertex->vertices_from_.end());
          return true;
        }
      }
    }

    return false;
  }

  /* Undirected Graph */
  /// This function is used to add an undirected edge connecting two nodes
  void AddUndirectedEdge(State sstate, State dstate, Transition trans)
  {
    AddEdge(sstate, dstate, trans);
    AddEdge(dstate, sstate, trans);
  }

  /// This function is used to remove the edge from src_node to dst_node.
  bool RemoveUndirectedEdge(State src_node, State dst_node)
  {
    bool edge1 = RemoveEdge(src_node, dst_node);
    bool edge2 = RemoveEdge(dst_node, src_node);

    if (edge1 && edge2)
      return true;
    else
      return false;
  }

  /// This functions is used to access all edges of a graph
  std::vector<edge_iterator> GetAllEdges() const
  {
    std::vector<edge_iterator> edges;
    for (auto &vertex_pair : vertex_map_)
    {
      auto vertex = vertex_pair.second;
      for (auto it = vertex->edge_begin(); it != vertex->edge_end(); ++it)
        edges.push_back(it);
    }
    return edges;
  }

  /// This function return the vertex iterator with specified id
  inline vertex_iterator FindVertex(int64_t vertex_id)
  {
    return vertex_iterator(vertex_map_.find(vertex_id));
  }

  template <class T = State, typename std::enable_if<!std::is_integral<T>::value>::type * = nullptr>
  inline vertex_iterator FindVertex(T state)
  {
    return vertex_iterator(vertex_map_.find(GetStateIndex(state)));
  }

  /// Get total number of vertices in the graph
  int64_t GetGraphVertexNumber() const { return vertex_map_.size(); }

  /// Get total number of edges in the graph
  int64_t GetGraphEdgeNumber() const { return GetAllEdges().size(); }

  /* Utility functions */
  /// This function is used to reset states of all vertice for a new search
  void ResetGraphVertices()
  {
    for (auto &vertex_pair : vertex_map_)
      vertex_pair.second->ClearVertexSearchInfo();
  }

  /// This function removes all edges and vertices in the graph
  void ClearGraph()
  {
    for (auto &vertex_pair : vertex_map_)
      delete vertex_pair.second;
    vertex_map_.clear();
  }
  ///@}

private:
  /** @name Internal variables and functions.
   *  Internal variables and functions.
   */
  ///@{
  /// This function returns an index of the give state.
  /// The default indexer returns member variable "id_", assuming it exists. 
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
  ///@}
};
} // namespace librav

// #include "graph/details/graph_impl.hpp"

#endif /* GRAPH_HPP */
