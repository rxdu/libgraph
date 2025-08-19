/*
 * tree.hpp
 *
 * Created on: Dec 30, 2018 07:28
 * Description: tree - a specialized graph
 *          main difference to Graph:
 *          1. Tree always maintains a root vertex, can be obtaind by
 * GetRootVertex()
 *          2. AddVertex() should only be used for adding root vertex
 *          3. RemoveVertex() internally invokes RemoveSubtree()
 *          4. AddUndirectedEdge()/RemoveUndirectedEdge() are defined the same
 * as AddEdge()/RemoveEdge()
 *          5. Addtional tree-specific operations
 *
 * Note: there is no loop checking when AddVertex() is called since checking by
 * default could be costly
 *
 * Major Revisions:
 *
 * Copyright (c) 2018-2021 Ruixiang Du (rdu)
 */

/* Reference
 *
 * Override nexted class function:
 * [1]
 * https://stackoverflow.com/questions/11448863/c-friend-override-nested-classs-function
 *
 * inheriting constructor
 * [1]
 * https://stackoverflow.com/questions/34006149/inheriting-templated-constructor-from-class-template
 *
 */

#ifndef GRAPH_TREE_HPP
#define GRAPH_TREE_HPP

#include <vector>
#include <cstdint>
#include <limits>
#include <algorithm>
#include <type_traits>
#include <stdexcept>

#include "graph/graph.hpp"

namespace xmotion {

/**
 * @brief Exception Safety Guarantees for Tree Operations
 * 
 * The Tree class inherits from Graph and maintains additional invariants.
 * This documentation defines the exception safety guarantees for Tree-specific operations.
 * 
 * @section tree_exception_safety_levels Exception Safety Levels
 * 
 * **1. Basic Guarantee**: No resource leaks, object remains in valid state
 * **2. Strong Guarantee**: Operation succeeds completely or has no effect
 * **3. No-throw Guarantee**: Operation never throws exceptions (marked noexcept)
 * 
 * @section tree_operation_guarantees Operation-Specific Guarantees
 * 
 * **Tree Structure Operations (Strong Guarantee)**
 * - AddRoot(): Strong guarantee - root added or tree unchanged
 * - AddEdge(): Strong guarantee - maintains tree structure invariants
 * - RemoveSubtree(): Strong guarantee - subtree fully removed or unchanged
 * - GetParentVertex(): Throws std::invalid_argument if vertex not found,
 *                       std::logic_error if tree invariant violated
 * 
 * **Tree Query Operations (No-throw Guarantee)**
 * - GetRootVertex(): No-throw - returns end() if no root
 * - GetVertexDepth(): No-throw - returns -1 if vertex not found
 * - ClearAll(): No-throw - RAII cleanup via unique_ptr
 * 
 * @section tree_invariants Tree Invariants
 * 
 * The Tree class maintains these invariants:
 * - Each vertex (except root) has exactly one parent
 * - No cycles exist in the structure
 * - All vertices are reachable from the root
 * - Root vertex has no parent vertices
 * 
 * @section thread_safety_tree Thread Safety
 * 
 * **Concurrent Operations**: 
 * - Read operations are thread-safe when no writes occur
 * - RemoveSubtree() now uses local visited tracking for thread safety
 * - Write operations require external synchronization
 * 
 * @note Tree operations maintain all Graph exception guarantees plus
 *       additional tree-specific invariants.
 */

/// Tree class template.
template <typename State, typename Transition = double,
          typename StateIndexer = DefaultIndexer<State>>
class Tree : public Graph<State, Transition, StateIndexer> {
 public:
  // derive constructor
  using Graph<State, Transition, StateIndexer>::Graph;

  using BaseType = Graph<State, Transition, StateIndexer>;
  using Edge = typename Graph<State, Transition, StateIndexer>::Edge;
  using Vertex = typename Graph<State, Transition, StateIndexer>::Vertex;
  using TreeType = Graph<State, Transition, StateIndexer>;

  using VertexMapType =
      typename Graph<State, Transition, StateIndexer>::VertexMapType;

  /*---------------------------------------------------------------------------------*/
  /*                                Tree Template */
  /*---------------------------------------------------------------------------------*/
 public:
  /** @name Vertex Access
   *  Vertex iterators to access vertices in the graph.
   */
  ///@{
  typedef typename TreeType::vertex_iterator vertex_iterator;
  typedef typename TreeType::const_vertex_iterator const_vertex_iterator;
  ///@}

  /** @name Edge Access
   *  Edge iterators to access edges in the vertex.
   */
  ///@{
  typedef typename Vertex::edge_iterator edge_iterator;
  typedef typename Vertex::const_edge_iterator const_edge_iterator;
  ///@}

  /** @name Tree Operations
   *  Modify vertex or edge of the graph.
   */
  ///@{
  /// This function is used to create a root vertex only
  vertex_iterator AddRoot(State state);

  // /// This function checks if a vertex exists in the graph and remove it if
  // presents. void RemoveVertex(int64_t state_id) { RemoveSubtree(state_id); };

  // template <class T = State, typename
  // std::enable_if<!std::is_integral<T>::value>::type * = nullptr> void
  // RemoveVertex(T state) { RemoveVertex(TreeType::GetStateIndex(state)); }

  /// This function returns the root vertex of the tree
  vertex_iterator GetRootVertex() const noexcept { return root_; }

  // / This function returns the parent vertex of the specified node
  vertex_iterator GetParentVertex(int64_t state_id);

  template <class T = State, typename std::enable_if<
                                 !std::is_integral<T>::value>::type* = nullptr>
  vertex_iterator GetParentVertex(State state) {
    return GetParentVertex(TreeType::GetStateIndex(state));
  }

  /// This function checks depth of the specified node in the three, assuming
  /// each node only has one parent
  int32_t GetVertexDepth(int64_t state_id);

  template <class T = State, typename std::enable_if<
                                 !std::is_integral<T>::value>::type* = nullptr>
  int32_t GetVertexDepth(T state) {
    return GetVertexDepth(TreeType::GetStateIndex(state));
  }

  /// This function is used to add an edge between the vertices associated with
  /// the given two states. Update the transition if edge already exists.
  void AddEdge(State sstate, State dstate, Transition trans);

  /// This function is used to remove the edge from src_node to dst_node.
  // bool RemoveEdge(State sstate, State dstate);

  /// Same with AddEdge()
  void AddUndirectedEdge(State sstate, State dstate, Transition trans) {
    AddEdge(sstate, dstate, trans);
  }

  /// Same with RemoveEdge()
  bool RemoveUndirectedEdge(State sstate, State dstate) {
    return BaseType::RemoveEdge(sstate, dstate);
  }

  /// This function removes a subtree including the specified vertex
  void RemoveSubtree(int64_t state_id);

  template <class T = State, typename std::enable_if<
                                 !std::is_integral<T>::value>::type* = nullptr>
  void RemoveSubtree(T state) {
    RemoveSubtree(TreeType::GetStateIndex(state));
  }

  /// This function removes all edges and vertices (including the root) in the
  /// graph
  void ClearAll() noexcept;
  
  /// Check if an edge exists between two states
  bool HasEdge(State from, State to) const;
  
  /// Get the weight/transition of an edge between two states
  Transition GetEdgeWeight(State from, State to) const;
  
  /// Get the total number of edges efficiently
  size_t GetEdgeCount() const noexcept;
  
  /// Safe vertex access - returns nullptr if not found
  Vertex* GetVertex(int64_t vertex_id);
  const Vertex* GetVertex(int64_t vertex_id) const;
  
  template <class T = State, typename std::enable_if<
                                 !std::is_integral<T>::value>::type * = nullptr>
  Vertex* GetVertex(T state) {
    return GetVertex(TreeType::GetStateIndex(state));
  }
  
  template <class T = State, typename std::enable_if<
                                 !std::is_integral<T>::value>::type * = nullptr>
  const Vertex* GetVertex(T state) const {
    return GetVertex(TreeType::GetStateIndex(state));
  }
  
  /** @name Tree Validation and Query Methods */
  ///@{
  /// Check if the tree structure is valid (no cycles, single parent per node)
  bool IsValidTree() const;
  
  /// Get the height of the tree (maximum depth from root)
  int32_t GetTreeHeight() const;
  
  /// Get all leaf nodes (vertices with no outgoing edges)
  std::vector<const_vertex_iterator> GetLeafNodes() const;
  
  /// Get direct children of a vertex
  std::vector<const_vertex_iterator> GetChildren(int64_t vertex_id) const;
  
  template <class T = State, typename std::enable_if<
                                 !std::is_integral<T>::value>::type * = nullptr>
  std::vector<const_vertex_iterator> GetChildren(T state) const {
    return GetChildren(TreeType::GetStateIndex(state));
  }
  
  /// Get the size of a subtree rooted at the given vertex
  size_t GetSubtreeSize(int64_t vertex_id) const;
  
  template <class T = State, typename std::enable_if<
                                 !std::is_integral<T>::value>::type * = nullptr>
  size_t GetSubtreeSize(T state) const {
    return GetSubtreeSize(TreeType::GetStateIndex(state));
  }
  
  /// Check if all vertices are reachable from root
  bool IsConnected() const;
  ///@}

 protected:
  vertex_iterator root_{vertex_iterator(TreeType::vertex_map_.end())};

 private:
  vertex_iterator AddVertex(State state);
};

template <typename State, typename Transition = double,
          typename StateIndexer = DefaultIndexer<State>>
using Tree_t = Tree<State, Transition, StateIndexer>;
}  // namespace xmotion

#include "graph/impl/tree_impl.hpp"

#endif /* GRAPH_TREE_HPP */
