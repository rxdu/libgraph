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

#ifndef TREE_HPP
#define TREE_HPP

#include <vector>
#include <cstdint>
#include <limits>
#include <algorithm>
#include <type_traits>

#include "graph/graph.hpp"

namespace rdu {
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
  vertex_iterator AddVertex(State state);

  // /// This function checks if a vertex exists in the graph and remove it if
  // presents. void RemoveVertex(int64_t state_id) { RemoveSubtree(state_id); };

  // template <class T = State, typename
  // std::enable_if<!std::is_integral<T>::value>::type * = nullptr> void
  // RemoveVertex(T state) { RemoveVertex(TreeType::GetStateIndex(state)); }

  /// This function returns the root vertex of the tree
  vertex_iterator GetRootVertex() const { return root_; }

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
  void ClearAll();
  ///@}

 protected:
  vertex_iterator root_{vertex_iterator(TreeType::vertex_map_.end())};
};

template <typename State, typename Transition = double,
          typename StateIndexer = DefaultIndexer<State>>
using Tree_t = Tree<State, Transition, StateIndexer>;
}  // namespace rdu

#include "graph/details/tree_impl.hpp"

#endif /* TREE_HPP */
