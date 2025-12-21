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

#include <unordered_map>

namespace xmotion {

// Forward declarations
template <typename State, typename Transition, typename StateIndexer>
class Graph;

template <typename State, typename Transition, typename StateIndexer>
class Vertex;

/// Edge class template - now independent from Graph
///
/// STABILITY NOTE: Edge uses raw Vertex pointers (Vertex*) instead of iterators.
/// This is intentional for stability: std::unordered_map iterators are invalidated
/// on rehash, but pointers to values stored via unique_ptr remain stable.
/// This ensures edges remain valid even when vertices are added to the graph.
template <typename State, typename Transition, typename StateIndexer>
struct Edge {
  // Forward declarations
  using GraphType = Graph<State, Transition, StateIndexer>;
  using VertexType = Vertex<State, Transition, StateIndexer>;

  Edge(VertexType* src, VertexType* dst, Transition c)
      : src(src), dst(dst), cost(c) {}

  /// Source vertex pointer (stable across graph modifications)
  VertexType* src;
  /// Destination vertex pointer (stable across graph modifications)
  VertexType* dst;
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
#include "graph/impl/edge_impl.hpp"

#endif /* GRAPH_EDGE_HPP */