/*
 * vertex.hpp
 *
 * Created on: Dec 9, 2015
 * Description: Vertex class for graph
 *
 * Copyright (c) 2015-2021 Ruixiang Du (rdu)
 */

#ifndef GRAPH_VERTEX_HPP
#define GRAPH_VERTEX_HPP

#include "graph/edge.hpp"
#include "graph/impl/default_indexer.hpp"
#include <cstdint>
#include <iostream>
#include <limits>
#include <list>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace xmotion {

// Forward declaration
template <typename State, typename Transition, typename StateIndexer>
class Graph;

/// Vertex class template - now independent from Graph
template <typename State, typename Transition, typename StateIndexer>
struct Vertex {
  using GraphType = Graph<State, Transition, StateIndexer>;
  using EdgeType = Edge<State, Transition, StateIndexer>;
  
  // IMPORTANT: Use Graph's vertex_iterator type to ensure compatibility
  using vertex_iterator = typename GraphType::vertex_iterator;
  using EdgeListType = std::list<EdgeType>;
  using edge_iterator = typename EdgeListType::iterator;
  using const_edge_iterator = typename EdgeListType::const_iterator;

  /** @name Big Five
   *  Constructor and destructor
   */
  ///@{
  Vertex(State s, int64_t id) : state(s), vertex_id(id) {}
  ~Vertex() = default;

  // Do not allow copy or assign
  Vertex() = delete;
  Vertex(const Vertex& other) = delete;
  Vertex& operator=(const Vertex& other) = delete;
  Vertex(Vertex&& other) = delete;
  Vertex& operator=(Vertex&& other) = delete;
  ///@}

  // Generic attributes
  State state;
  const int64_t vertex_id;
  StateIndexer GetStateIndex;

  // Edges connecting to other vertices
  EdgeListType edges_to;

  // Vertices that contain edges connecting to current vertex
  std::list<vertex_iterator> vertices_from;

  // Attributes for search algorithms
  // NOTE: These fields are deprecated for thread safety. Use SearchContext instead.
  // Will be removed in a future version.
  [[deprecated("Use SearchContext for thread-safe searches")]] 
  bool is_checked = false;
  [[deprecated("Use SearchContext for thread-safe searches")]]
  bool is_in_openlist = false;
  [[deprecated("Use SearchContext for thread-safe searches")]]
  double f_cost = std::numeric_limits<double>::max();
  [[deprecated("Use SearchContext for thread-safe searches")]]
  double g_cost = std::numeric_limits<double>::max();
  [[deprecated("Use SearchContext for thread-safe searches")]]
  double h_cost = std::numeric_limits<double>::max();
  [[deprecated("Use SearchContext for thread-safe searches")]]
  vertex_iterator search_parent;

  /** @name Edge access
   *  Edge iterators to access edges in the vertex
   */
  ///@{
  edge_iterator edge_begin() noexcept { return edges_to.begin(); }
  edge_iterator edge_end() noexcept { return edges_to.end(); }
  const_edge_iterator edge_begin() const noexcept { return edges_to.cbegin(); }
  const_edge_iterator edge_end() const noexcept { return edges_to.cend(); }
  ///@}

  /** @name Edge Operations
   *  Modify or query edge information of the vertex
   */
  ///@{
  /// Returns true if two vertices have the same id
  bool operator==(const Vertex& other) const;

  /// Returns the id of current vertex
  int64_t GetVertexID() const noexcept { return vertex_id; }

  /// Check if a vertex with given state or id is a neighbor of current vertex
  template <class T = State,
            typename std::enable_if<!std::is_integral<T>::value>::type* = nullptr>
  bool CheckNeighbour(T state);
  
  template <typename T>
  bool CheckNeighbour(T vertex_id);

  /// Find the edge connecting to a vertex with given state or id
  edge_iterator FindEdge(int64_t vertex_id);
  
  template <class T = State,
            typename std::enable_if<!std::is_integral<T>::value>::type* = nullptr>
  edge_iterator FindEdge(T state);

  /// Return all neighbors of this vertex
  std::vector<vertex_iterator> GetNeighbours();

  /// Print vertex information
  void PrintVertex() const;
  
  /// Clear vertex search info for new search
  /// @deprecated Use SearchContext for thread-safe searches instead
  [[deprecated("Use SearchContext for thread-safe searches")]]
  void ClearVertexSearchInfo();
  ///@}
  
  // Friend declaration for Graph to access private members if needed
  friend class Graph<State, Transition, StateIndexer>;
};

}  // namespace xmotion

// Include implementation after all declarations
#include "graph/impl/vertex_impl.hpp"

#endif /* GRAPH_VERTEX_HPP */