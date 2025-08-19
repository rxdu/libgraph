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
#include <unordered_set>
#include <functional>
#include <algorithm>
#include "graph/exceptions.hpp"

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
    throw ElementNotFoundError("Vertex", state_id);
  }
  if (vtx->vertices_from.size() > 1) {
    throw StructureViolationError("single-parent", 
                                  "Vertex with state_id " + std::to_string(state_id) + 
                                  " has " + std::to_string(vtx->vertices_from.size()) + 
                                  " parents (expected at most 1)");
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
    // iterate through all vertices of the subtree using local visited tracking
    // for thread safety (instead of using deprecated vertex is_checked field)
    std::unordered_set<int64_t> visited;
    std::vector<vertex_iterator> child_vertices;
    std::queue<vertex_iterator> queue;
    
    queue.push(vtx);
    visited.insert(vtx->vertex_id);
    
    while (!queue.empty()) {
      auto node = queue.front();
      child_vertices.push_back(node);
      
      for (auto it = node->edges_to.begin(); it != node->edges_to.end(); ++it) {
        if (visited.find(it->dst->vertex_id) == visited.end()) {
          queue.push(it->dst);
          visited.insert(it->dst->vertex_id);
        }
      }
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
void Tree<State, Transition, StateIndexer>::ClearAll() noexcept {
  TreeType::vertex_map_.clear();  // unique_ptr handles cleanup automatically
  root_ = TreeType::vertex_end();
}

template <typename State, typename Transition, typename StateIndexer>
bool Tree<State, Transition, StateIndexer>::HasEdge(State from, State to) const {
  auto src_vertex = TreeType::FindVertex(from);
  if (src_vertex == TreeType::vertex_end()) return false;
  
  auto edge = src_vertex->FindEdge(to);
  return edge != src_vertex->edge_end();
}

template <typename State, typename Transition, typename StateIndexer>
Transition Tree<State, Transition, StateIndexer>::GetEdgeWeight(State from, State to) const {
  auto src_vertex = TreeType::FindVertex(from);
  if (src_vertex == TreeType::vertex_end()) return Transition{};
  
  auto edge = src_vertex->FindEdge(to);
  if (edge != src_vertex->edge_end()) {
    return edge->cost;
  }
  return Transition{};
}

template <typename State, typename Transition, typename StateIndexer>
size_t Tree<State, Transition, StateIndexer>::GetEdgeCount() const noexcept {
  size_t count = 0;
  for (auto it = TreeType::vertex_begin(); it != TreeType::vertex_end(); ++it) {
    count += it->edges_to.size();
  }
  return count;
}

template <typename State, typename Transition, typename StateIndexer>
typename Tree<State, Transition, StateIndexer>::Vertex*
Tree<State, Transition, StateIndexer>::GetVertex(int64_t vertex_id) {
  auto iter = TreeType::vertex_map_.find(vertex_id);
  if (iter != TreeType::vertex_map_.end()) {
    return iter->second.get();
  }
  return nullptr;
}

template <typename State, typename Transition, typename StateIndexer>
const typename Tree<State, Transition, StateIndexer>::Vertex*
Tree<State, Transition, StateIndexer>::GetVertex(int64_t vertex_id) const {
  auto iter = TreeType::vertex_map_.find(vertex_id);
  if (iter != TreeType::vertex_map_.end()) {
    return iter->second.get();
  }
  return nullptr;
}

template <typename State, typename Transition, typename StateIndexer>
bool Tree<State, Transition, StateIndexer>::IsValidTree() const {
  if (TreeType::vertex_map_.empty()) return true;
  
  // Check that all vertices (except root) have exactly one parent
  for (auto it = this->vertex_begin(); it != this->vertex_end(); ++it) {
    const_vertex_iterator const_root(root_.base());
    if (it == const_root) {
      if (!it->vertices_from.empty()) return false; // Root should have no parents
    } else {
      if (it->vertices_from.size() != 1) return false; // Non-root should have exactly one parent
    }
  }
  
  // Check for cycles using DFS
  std::unordered_set<int64_t> visited;
  std::unordered_set<int64_t> rec_stack;
  
  std::function<bool(const_vertex_iterator)> has_cycle = [&](const_vertex_iterator v) -> bool {
    visited.insert(v->vertex_id);
    rec_stack.insert(v->vertex_id);
    
    for (auto& edge : v->edges_to) {
      if (rec_stack.find(edge.dst->vertex_id) != rec_stack.end()) {
        return true; // Found a cycle
      }
      if (visited.find(edge.dst->vertex_id) == visited.end()) {
        if (has_cycle(const_vertex_iterator(edge.dst.base()))) return true;
      }
    }
    
    rec_stack.erase(v->vertex_id);
    return false;
  };
  
  if (root_.base() != TreeType::vertex_map_.end()) {
    const_vertex_iterator const_root(root_.base());
    if (has_cycle(const_root)) {
      return false;
    }
  }
  
  return true;
}

template <typename State, typename Transition, typename StateIndexer>
int32_t Tree<State, Transition, StateIndexer>::GetTreeHeight() const {
  if (TreeType::vertex_map_.empty() || root_.base() == TreeType::vertex_map_.end()) return 0;
  
  std::function<int32_t(const_vertex_iterator)> get_height = [&](const_vertex_iterator v) -> int32_t {
    if (v->edges_to.empty()) return 0; // Leaf node
    
    int32_t max_height = 0;
    for (auto& edge : v->edges_to) {
      max_height = std::max(max_height, get_height(const_vertex_iterator(edge.dst.base())));
    }
    return max_height + 1;
  };
  
  return get_height(const_vertex_iterator(root_.base()));
}

template <typename State, typename Transition, typename StateIndexer>
std::vector<typename Tree<State, Transition, StateIndexer>::const_vertex_iterator>
Tree<State, Transition, StateIndexer>::GetLeafNodes() const {
  std::vector<const_vertex_iterator> leaves;
  
  for (auto it = this->vertex_begin(); it != this->vertex_end(); ++it) {
    if (it->edges_to.empty()) {
      leaves.push_back(it);
    }
  }
  
  return leaves;
}

template <typename State, typename Transition, typename StateIndexer>
std::vector<typename Tree<State, Transition, StateIndexer>::const_vertex_iterator>
Tree<State, Transition, StateIndexer>::GetChildren(int64_t vertex_id) const {
  std::vector<const_vertex_iterator> children;
  
  auto vtx = this->FindVertex(vertex_id);
  if (vtx != this->vertex_end()) {
    for (auto& edge : vtx->edges_to) {
      children.push_back(const_vertex_iterator(edge.dst.base()));
    }
  }
  
  return children;
}

template <typename State, typename Transition, typename StateIndexer>
size_t Tree<State, Transition, StateIndexer>::GetSubtreeSize(int64_t vertex_id) const {
  auto vtx = this->FindVertex(vertex_id);
  if (vtx == this->vertex_end()) return 0;
  
  size_t size = 1; // Count the root of the subtree
  std::queue<const_vertex_iterator> queue;
  queue.push(vtx);
  
  std::unordered_set<int64_t> visited;
  visited.insert(vtx->vertex_id);
  
  while (!queue.empty()) {
    auto node = queue.front();
    queue.pop();
    
    for (auto& edge : node->edges_to) {
      if (visited.find(edge.dst->vertex_id) == visited.end()) {
        size++;
        queue.push(const_vertex_iterator(edge.dst.base()));
        visited.insert(edge.dst->vertex_id);
      }
    }
  }
  
  return size;
}

template <typename State, typename Transition, typename StateIndexer>
bool Tree<State, Transition, StateIndexer>::IsConnected() const {
  if (TreeType::vertex_map_.empty()) return true;
  if (root_.base() == TreeType::vertex_map_.end()) return false;
  
  // Count reachable vertices from root
  size_t reachable = GetSubtreeSize(root_->vertex_id);
  
  // Check if all vertices are reachable
  return reachable == TreeType::vertex_map_.size();
}
}  // namespace xmotion

#endif /* TREE_IMPL_HPP */
