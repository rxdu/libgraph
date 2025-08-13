/*
 * edge.hpp
 *
 * Created on: Dec 9, 2015
 * Description: Edge class for graph
 *
 * Copyright (c) 2015-2021 Ruixiang Du (rdu)
 */

#ifndef GRAPH_EDGE_HPP
#define GRAPH_EDGE_HPP

#include <iostream>
#include <unordered_map>

namespace xmotion {

// Forward declarations
template <typename State, typename Transition, typename StateIndexer>
class Graph;

template <typename State, typename Transition, typename StateIndexer>
class Vertex;

/// Edge class template - now independent from Graph
template <typename State, typename Transition, typename StateIndexer>
struct Edge {
  // Forward declarations
  using GraphType = Graph<State, Transition, StateIndexer>;
  using VertexType = Vertex<State, Transition, StateIndexer>;
  
  // IMPORTANT: Use Graph's vertex_iterator type to ensure compatibility
  using vertex_iterator = typename GraphType::vertex_iterator;

  Edge(vertex_iterator src, vertex_iterator dst, Transition c)
      : src(src), dst(dst), cost(c) {}

  vertex_iterator src;
  vertex_iterator dst;
  Transition cost;

  /// Check if current edge is identical to the other (all src, dst, cost)
  bool operator==(const Edge& other) const;

  /// Print edge information, assuming member "cost" is printable
  void PrintEdge() const;
  
  // Friend declaration for Graph to access private members if needed
  friend class Graph<State, Transition, StateIndexer>;
  friend class Vertex<State, Transition, StateIndexer>;
};

}  // namespace xmotion

// Include implementation after all declarations
#include "graph/details/edge_impl.hpp"

#endif /* GRAPH_EDGE_HPP */