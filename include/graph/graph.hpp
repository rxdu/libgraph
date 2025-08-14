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

#include <algorithm>
#include <cstdint>
#include <limits>
#include <list>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "graph/edge.hpp" // Independent Edge class
#include "graph/impl/default_indexer.hpp"
#include "graph/vertex.hpp" // Independent Vertex class

namespace xmotion {
/// Graph class template.
template <typename State, typename Transition = double,
          typename StateIndexer = DefaultIndexer<State>>
class Graph {
public:
  // Use independent Edge and Vertex classes
  using Edge = xmotion::Edge<State, Transition, StateIndexer>;
  using Vertex = xmotion::Vertex<State, Transition, StateIndexer>;
  using GraphType = Graph<State, Transition, StateIndexer>;

  using VertexMapType = std::unordered_map<int64_t, Vertex *>;
  using VertexMapTypeIterator = typename VertexMapType::iterator;
  using VertexMapTypeConstIterator = typename VertexMapType::const_iterator;

  /*---------------------------------------------------------------------------------*/
  /*                              Vertex Iterator */
  /*---------------------------------------------------------------------------------*/
  ///@{
  /// Const vertex iterator for unified access.
  /// Wraps the "value" part of VertexMapType::const_iterator
  class const_vertex_iterator {
  private:
    VertexMapTypeConstIterator iter_;
    
  public:
    // Iterator traits
    using iterator_category = std::forward_iterator_tag;
    using value_type = const Vertex;
    using difference_type = std::ptrdiff_t;
    using pointer = const Vertex*;
    using reference = const Vertex&;

    const_vertex_iterator() : iter_() {}
    explicit const_vertex_iterator(VertexMapTypeConstIterator s) : iter_(s) {}
    explicit const_vertex_iterator(VertexMapTypeIterator s) : iter_(s) {}

    const Vertex *operator->() const;
    const Vertex &operator*() const;
    
    const_vertex_iterator& operator++() { ++iter_; return *this; }
    const_vertex_iterator operator++(int) { const_vertex_iterator tmp(*this); ++iter_; return tmp; }
    
    bool operator==(const const_vertex_iterator& other) const { return iter_ == other.iter_; }
    bool operator!=(const const_vertex_iterator& other) const { return iter_ != other.iter_; }
    
    // Access to underlying iterator for compatibility
    VertexMapTypeConstIterator base() const { return iter_; }
  };

  class vertex_iterator {
  private:
    VertexMapTypeIterator iter_;
    
  public:
    // Iterator traits
    using iterator_category = std::forward_iterator_tag;
    using value_type = Vertex;
    using difference_type = std::ptrdiff_t;
    using pointer = Vertex*;
    using reference = Vertex&;

    vertex_iterator() : iter_() {}
    explicit vertex_iterator(VertexMapTypeIterator s) : iter_(s) {}

    Vertex *operator->();
    Vertex &operator*();
    const Vertex *operator->() const;
    
    vertex_iterator& operator++() { ++iter_; return *this; }
    vertex_iterator operator++(int) { vertex_iterator tmp(*this); ++iter_; return tmp; }
    
    bool operator==(const vertex_iterator& other) const { return iter_ == other.iter_; }
    bool operator!=(const vertex_iterator& other) const { return iter_ != other.iter_; }
    
    // Conversion to const_vertex_iterator
    operator const_vertex_iterator() const { return const_vertex_iterator(iter_); }
    
    // Access to underlying iterator for compatibility
    VertexMapTypeIterator base() const { return iter_; }

    // Hash support for vertex_iterator
    struct Hash {
      size_t operator()(const vertex_iterator &iter) const;
    };

    // Equality comparison for vertex_iterator (for unordered containers)
    struct Equal {
      bool operator()(const vertex_iterator &a, const vertex_iterator &b) const;
    };
  };
  ///@}

  /*---------------------------------------------------------------------------------*/
  /*                              Edge Iterator */
  /*---------------------------------------------------------------------------------*/
  /** @name Edge Access
   *  Edge iterators to access edges in the vertex.
   */
  ///@{
  using edge_iterator = typename Vertex::edge_iterator;
  using const_edge_iterator = typename Vertex::const_edge_iterator;
  ///@}

public:
  // Note: Edge and Vertex classes are now defined independently in their own
  // headers The type aliases above (using Edge = ..., using Vertex = ...) make
  // them available as if they were nested classes for backward compatibility

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
  
  /// Swap function for efficient assignment operations
  void swap(GraphType& other) noexcept;
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

  /// This function return the const vertex iterator with specified id
  inline const_vertex_iterator FindVertex(int64_t vertex_id) const {
    return const_vertex_iterator{vertex_map_.find(vertex_id)};
  }

  /// This function return the vertex iterator with specified state
  template <class T = State, typename std::enable_if<
                                 !std::is_integral<T>::value>::type * = nullptr>
  inline vertex_iterator FindVertex(T state) {
    return vertex_iterator{vertex_map_.find(GetStateIndex(state))};
  }

  /// This function return the const vertex iterator with specified state
  template <class T = State, typename std::enable_if<
                                 !std::is_integral<T>::value>::type * = nullptr>
  inline const_vertex_iterator FindVertex(T state) const {
    return const_vertex_iterator{vertex_map_.find(GetStateIndex(state))};
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
} // namespace xmotion

#include "graph/impl/edge_impl.hpp"
#include "graph/impl/graph_impl.hpp"
#include "graph/impl/vertex_impl.hpp"

#endif /* GRAPH_HPP */
