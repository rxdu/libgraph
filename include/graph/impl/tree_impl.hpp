/*
 * tree_impl.hpp
 *
 * Created on: Dec 30, 2018 07:36
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TREE_IMPL_HPP
#define TREE_IMPL_HPP

#include <type_traits>
#include <queue>

namespace xmotion {
template <typename State, typename Transition, typename StateIndexer>
typename Tree<State, Transition, StateIndexer>::vertex_iterator
Tree<State, Transition, StateIndexer>::AddRoot(State state) {
  // only add root vertex if tree is empty
  if (!TreeType::vertex_map_.empty()) return TreeType::vertex_end();
  root_ = TreeType::ObtainVertexFromVertexMap(state);
  return root_;
}

template <typename State, typename Transition, typename StateIndexer>
int32_t Tree<State, Transition, StateIndexer>::GetVertexDepth(
    int64_t state_id) {
  auto vtx = TreeType::FindVertex(state_id);

  if (vtx != TreeType::vertex_end()) {
    int32_t depth = 0;
    auto parent = vtx->vertices_from;
    while (!parent.empty()) {
      ++depth;
      parent = parent.front()->vertices_from;
    }
    return depth;
  }

  return -1;
}

template <typename State, typename Transition, typename StateIndexer>
typename Tree<State, Transition, StateIndexer>::vertex_iterator
Tree<State, Transition, StateIndexer>::GetParentVertex(int64_t state_id) {
  auto vtx = TreeType::FindVertex(state_id);

  if (vtx == TreeType::vertex_end()) {
    throw std::invalid_argument("GetParentVertex: Vertex with state_id " + 
                                std::to_string(state_id) + " does not exist in tree");
  }
  if (vtx->vertices_from.size() > 1) {
    throw std::logic_error("Tree invariant violated: Vertex with state_id " + 
                          std::to_string(state_id) + " has " + 
                          std::to_string(vtx->vertices_from.size()) + " parents (expected at most 1)");
  }

  if (vtx == root_)
    return TreeType::vertex_end();
  else
    return vtx->vertices_from.front();
}

template <typename State, typename Transition, typename StateIndexer>
void Tree<State, Transition, StateIndexer>::RemoveSubtree(int64_t state_id) {
  auto vtx = TreeType::FindVertex(state_id);

  // remove if specified vertex exists
  if (vtx != TreeType::vertex_end()) {
    // remove from other vertices that connect to the vertex to be deleted
    for (auto &asv : vtx->vertices_from) {
      asv->edges_to.erase(
          std::remove_if(asv->edges_to.begin(), asv->edges_to.end(),
                         [&vtx](Edge edge) { return ((edge.dst) == vtx); }),
          asv->edges_to.end());
    }

    // remove all subsequent vertices
    // iterate through all vertices of the subtree
    std::vector<vertex_iterator> child_vertices;
    std::queue<vertex_iterator> queue;
    queue.push(vtx);
    while (!queue.empty()) {
      auto node = queue.front();
      child_vertices.push_back(node);
      for (auto it = node->edges_to.begin(); it != node->edges_to.end(); ++it) {
        if (!it->dst->is_checked) {
          queue.push(it->dst);
        }
      }
      node->is_checked = true;
      queue.pop();
    }

    for (auto &vtx : child_vertices) {
      // remove from vertex map - unique_ptr handles cleanup automatically
      TreeType::vertex_map_.erase(vtx.base());
    }
  }
}

template <typename State, typename Transition, typename StateIndexer>
void Tree<State, Transition, StateIndexer>::AddEdge(State sstate, State dstate,
                                                    Transition trans) {
  bool tree_empty = TreeType::vertex_map_.empty();

  auto src_vertex = TreeType::ObtainVertexFromVertexMap(sstate);
  auto dst_vertex = TreeType::ObtainVertexFromVertexMap(dstate);

  // set root if tree is empty or a parent vertex is connected to root_
  if (tree_empty || (dst_vertex == root_)) root_ = src_vertex;

  // update transition if edge already exists
  auto it = src_vertex->FindEdge(dstate);
  if (it != src_vertex->edge_end()) {
    it->cost = trans;
    return;
  }

  dst_vertex->vertices_from.push_back(src_vertex);
  src_vertex->edges_to.emplace_back(src_vertex, dst_vertex, trans);
}

template <typename State, typename Transition, typename StateIndexer>
void Tree<State, Transition, StateIndexer>::ClearAll() {
  TreeType::vertex_map_.clear();  // unique_ptr handles cleanup automatically
  root_ = TreeType::vertex_end();
}
}  // namespace xmotion

#endif /* TREE_IMPL_HPP */
