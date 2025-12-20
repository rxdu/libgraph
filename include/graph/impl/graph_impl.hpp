/*
 * graph_impl.hpp
 *
 * Created on: Sep 04, 2018 01:56
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef GRAPH_IMPL_HPP
#define GRAPH_IMPL_HPP

#include <type_traits>
#include <memory>

namespace xmotion {

/*---------------------------------------------------------------------------------*/
/*                         Iterator Implementations                               */
/*---------------------------------------------------------------------------------*/

// const_vertex_iterator implementations
template <typename State, typename Transition, typename StateIndexer>
const typename Graph<State, Transition, StateIndexer>::Vertex*
Graph<State, Transition, StateIndexer>::const_vertex_iterator::operator->() const {
  return iter_->second.get();
}

template <typename State, typename Transition, typename StateIndexer>
const typename Graph<State, Transition, StateIndexer>::Vertex&
Graph<State, Transition, StateIndexer>::const_vertex_iterator::operator*() const {
  return *(iter_->second.get());
}

// vertex_iterator implementations
template <typename State, typename Transition, typename StateIndexer>
typename Graph<State, Transition, StateIndexer>::Vertex*
Graph<State, Transition, StateIndexer>::vertex_iterator::operator->() {
  return iter_->second.get();
}

template <typename State, typename Transition, typename StateIndexer>
typename Graph<State, Transition, StateIndexer>::Vertex&
Graph<State, Transition, StateIndexer>::vertex_iterator::operator*() {
  return *(iter_->second.get());
}

template <typename State, typename Transition, typename StateIndexer>
const typename Graph<State, Transition, StateIndexer>::Vertex*
Graph<State, Transition, StateIndexer>::vertex_iterator::operator->() const {
  return iter_->second.get();
}

template <typename State, typename Transition, typename StateIndexer>
size_t Graph<State, Transition, StateIndexer>::vertex_iterator::Hash::operator()(
    const vertex_iterator& iter) const {
  return std::hash<int64_t>()(iter->vertex_id);
}

template <typename State, typename Transition, typename StateIndexer>
bool Graph<State, Transition, StateIndexer>::vertex_iterator::Equal::operator()(
    const vertex_iterator& a, const vertex_iterator& b) const {
  return a->vertex_id == b->vertex_id;
}

/*---------------------------------------------------------------------------------*/
/*                         Graph Class Implementations                            */
/*---------------------------------------------------------------------------------*/
template <typename State, typename Transition, typename StateIndexer>
Graph<State, Transition, StateIndexer>::Graph(
    const Graph<State, Transition, StateIndexer> &other) {
  for (auto &pair : other.vertex_map_) {
    auto& vertex = pair.second;
    // First ensure the vertex exists (handles isolated vertices)
    this->AddVertex(vertex->state);
    // Then add all edges
    for (auto &edge : vertex->edges_to)
      this->AddEdge(edge.src->state, edge.dst->state, edge.cost);
  }
}

template <typename State, typename Transition, typename StateIndexer>
Graph<State, Transition, StateIndexer>::Graph(
    Graph<State, Transition, StateIndexer> &&other) noexcept {
  vertex_map_ = std::move(other.vertex_map_);
}

template <typename State, typename Transition, typename StateIndexer>
Graph<State, Transition, StateIndexer>
    &Graph<State, Transition, StateIndexer>::operator=(
        const Graph<State, Transition, StateIndexer> &other) {
  if (this != &other) {
    Graph<State, Transition, StateIndexer> temp(other);
    this->swap(temp);
  }
  return *this;
}

template <typename State, typename Transition, typename StateIndexer>
Graph<State, Transition, StateIndexer>
    &Graph<State, Transition, StateIndexer>::operator=(
        Graph<State, Transition, StateIndexer> &&other) noexcept {
  std::swap(vertex_map_, other.vertex_map_);
  return *this;
}

template <typename State, typename Transition, typename StateIndexer>
void Graph<State, Transition, StateIndexer>::swap(Graph& other) noexcept {
  vertex_map_.swap(other.vertex_map_);
}

template <typename State, typename Transition, typename StateIndexer>
Graph<State, Transition, StateIndexer>::~Graph() {
  // unique_ptr automatically handles cleanup - no manual delete needed
};

template <typename State, typename Transition, typename StateIndexer>
typename Graph<State, Transition, StateIndexer>::vertex_iterator
Graph<State, Transition, StateIndexer>::AddVertex(State state) {
  return ObtainVertexFromVertexMap(state);
}

template <typename State, typename Transition, typename StateIndexer>
void Graph<State, Transition, StateIndexer>::RemoveVertex(int64_t state_id) {
  auto it = vertex_map_.find(state_id);

  // remove if specified vertex exists
  if (it != vertex_map_.end()) {
    Vertex* vtx_ptr = it->second.get();

    // remove upstream connections
    // e.g. other vertices that connect to the vertex to be deleted
    for (auto* incoming_vertex : vtx_ptr->vertices_from) {
      // Remove edges from incoming_vertex that point to vtx_ptr
      auto vtx_id = vtx_ptr->vertex_id;
      incoming_vertex->edges_to.remove_if([vtx_id](const Edge& edge) {
        return edge.dst->vertex_id == vtx_id;
      });
    }

    // remove downstream connections
    // e.g. other vertices that are connected by the vertex to be deleted
    for (auto& edge : vtx_ptr->edges_to) {
      Vertex* target_vertex = edge.dst;
      // Remove vtx_ptr from target's incoming list
      target_vertex->vertices_from.remove(vtx_ptr);
    }

    // remove from vertex map - unique_ptr handles cleanup automatically
    vertex_map_.erase(it);
  }
}

template <typename State, typename Transition, typename StateIndexer>
void Graph<State, Transition, StateIndexer>::AddEdge(State sstate, State dstate,
                                                     Transition trans) {
  auto src_vertex_it = ObtainVertexFromVertexMap(std::move(sstate));
  Vertex* src_vertex_ptr = &(*src_vertex_it);

  // update transition if edge already exists
  auto it = src_vertex_it->FindEdge(dstate);
  if (it != src_vertex_it->edge_end()) {
    it->cost = trans;
    return;
  }

  // otherwise add new edge
  auto dst_vertex_it = ObtainVertexFromVertexMap(std::move(dstate));
  Vertex* dst_vertex_ptr = &(*dst_vertex_it);

  // Use stable Vertex* pointers instead of iterators for rehash safety
  dst_vertex_ptr->vertices_from.push_back(src_vertex_ptr);
  src_vertex_ptr->edges_to.emplace_back(src_vertex_ptr, dst_vertex_ptr, trans);
}

template <typename State, typename Transition, typename StateIndexer>
bool Graph<State, Transition, StateIndexer>::RemoveEdge(State sstate,
                                                        State dstate) {
  auto src_vertex_it = FindVertex(sstate);
  auto dst_vertex_it = FindVertex(dstate);

  if ((src_vertex_it != vertex_end()) && (dst_vertex_it != vertex_end())) {
    Vertex* src_vertex_ptr = &(*src_vertex_it);
    Vertex* dst_vertex_ptr = &(*dst_vertex_it);

    for (auto it = src_vertex_ptr->edges_to.begin();
         it != src_vertex_ptr->edges_to.end(); ++it) {
      if (it->dst == dst_vertex_ptr) {
        src_vertex_ptr->edges_to.erase(it);
        // Remove src from dst's incoming list using stable pointer comparison
        dst_vertex_ptr->vertices_from.remove(src_vertex_ptr);
        return true;
      }
    }
  }

  return false;
}

template <typename State, typename Transition, typename StateIndexer>
void Graph<State, Transition, StateIndexer>::AddUndirectedEdge(
    State sstate, State dstate, Transition trans) {
  // Add first edge
  AddEdge(sstate, dstate, trans);

  // Try to add second edge with rollback on failure
  try {
    AddEdge(dstate, sstate, trans);
  } catch (...) {
    // Rollback: remove the first edge to maintain atomicity
    RemoveEdge(sstate, dstate);
    throw;  // Re-throw the original exception
  }
}

template <typename State, typename Transition, typename StateIndexer>
bool Graph<State, Transition, StateIndexer>::RemoveUndirectedEdge(
    State sstate, State dstate) {
  bool edge1 = RemoveEdge(sstate, dstate);
  bool edge2 = RemoveEdge(dstate, sstate);

  if (edge1 && edge2)
    return true;
  else
    return false;
}

template <typename State, typename Transition, typename StateIndexer>
std::vector<typename Graph<State, Transition, StateIndexer>::edge_iterator>
Graph<State, Transition, StateIndexer>::GetAllEdges() const {
  std::vector<typename Graph<State, Transition, StateIndexer>::edge_iterator>
      edges;
  for (auto &vertex_pair : vertex_map_) {
    auto& vertex = vertex_pair.second;
    for (auto it = vertex->edge_begin(); it != vertex->edge_end(); ++it)
      edges.push_back(it);
  }
  return edges;
}

template <typename State, typename Transition, typename StateIndexer>
void Graph<State, Transition, StateIndexer>::ClearAll() {
  vertex_map_.clear();  // unique_ptr automatically handles cleanup
}

template <typename State, typename Transition, typename StateIndexer>
typename Graph<State, Transition, StateIndexer>::vertex_iterator
Graph<State, Transition, StateIndexer>::ObtainVertexFromVertexMap(State state) {
  int64_t state_id = GetStateIndex(state);
  auto it = vertex_map_.find(state_id);

  if (it == vertex_map_.end()) {
    // Exception-safe vertex creation using unique_ptr with move semantics
    std::unique_ptr<Vertex> new_vertex(new Vertex(std::move(state), state_id));
    // Note: search_parent initialization removed - field is deprecated
    auto result = vertex_map_.insert(std::make_pair(state_id, std::move(new_vertex)));
    return vertex_iterator(result.first);
  }

  return vertex_iterator(it);
}

/*---------------------------------------------------------------------------------*/
/*                     API Polish - Convenience Methods Implementation            */
/*---------------------------------------------------------------------------------*/

// Vertex Information Access Methods
template <typename State, typename Transition, typename StateIndexer>
bool Graph<State, Transition, StateIndexer>::HasVertex(int64_t vertex_id) const {
  return vertex_map_.find(vertex_id) != vertex_map_.end();
}

template <typename State, typename Transition, typename StateIndexer>
size_t Graph<State, Transition, StateIndexer>::GetVertexDegree(int64_t vertex_id) const {
  return GetInDegree(vertex_id) + GetOutDegree(vertex_id);
}

template <typename State, typename Transition, typename StateIndexer>
size_t Graph<State, Transition, StateIndexer>::GetInDegree(int64_t vertex_id) const {
  auto it = FindVertex(vertex_id);
  if (it != vertex_end()) {
    return it->vertices_from.size();
  }
  return 0;
}

template <typename State, typename Transition, typename StateIndexer>
size_t Graph<State, Transition, StateIndexer>::GetOutDegree(int64_t vertex_id) const {
  auto it = FindVertex(vertex_id);
  if (it != vertex_end()) {
    return it->edges_to.size();
  }
  return 0;
}

template <typename State, typename Transition, typename StateIndexer>
std::vector<State> Graph<State, Transition, StateIndexer>::GetNeighbors(State state) const {
  return GetNeighbors(GetStateIndex(state));
}

template <typename State, typename Transition, typename StateIndexer>
std::vector<State> Graph<State, Transition, StateIndexer>::GetNeighbors(int64_t vertex_id) const {
  std::vector<State> neighbors;
  auto it = FindVertex(vertex_id);
  if (it != vertex_end()) {
    for (const auto& edge : it->edges_to) {
      neighbors.push_back(edge.dst->state);
    }
  }
  return neighbors;
}

// Edge Query Methods
template <typename State, typename Transition, typename StateIndexer>
bool Graph<State, Transition, StateIndexer>::HasEdge(State from, State to) const {
  auto from_it = FindVertex(from);
  if (from_it == vertex_end()) {
    return false;
  }
  
  int64_t to_id = GetStateIndex(to);
  for (const auto& edge : from_it->edges_to) {
    if (edge.dst->vertex_id == to_id) {
      return true;
    }
  }
  return false;
}

template <typename State, typename Transition, typename StateIndexer>
Transition Graph<State, Transition, StateIndexer>::GetEdgeWeight(State from, State to) const {
  auto from_it = FindVertex(from);
  if (from_it == vertex_end()) {
    return Transition{};
  }
  
  int64_t to_id = GetStateIndex(to);
  for (const auto& edge : from_it->edges_to) {
    if (edge.dst->vertex_id == to_id) {
      return edge.cost;
    }
  }
  return Transition{};
}

template <typename State, typename Transition, typename StateIndexer>
size_t Graph<State, Transition, StateIndexer>::GetEdgeCount() const noexcept {
  size_t count = 0;
  for (const auto& pair : vertex_map_) {
    count += pair.second->edges_to.size();
  }
  return count;
}

// Safe Vertex Access Methods
template <typename State, typename Transition, typename StateIndexer>
typename Graph<State, Transition, StateIndexer>::Vertex* 
Graph<State, Transition, StateIndexer>::GetVertex(int64_t vertex_id) {
  auto it = vertex_map_.find(vertex_id);
  if (it != vertex_map_.end()) {
    return it->second.get();
  }
  return nullptr;
}

template <typename State, typename Transition, typename StateIndexer>
const typename Graph<State, Transition, StateIndexer>::Vertex* 
Graph<State, Transition, StateIndexer>::GetVertex(int64_t vertex_id) const {
  auto it = vertex_map_.find(vertex_id);
  if (it != vertex_map_.end()) {
    return it->second.get();
  }
  return nullptr;
}

// Batch Operations
template <typename State, typename Transition, typename StateIndexer>
void Graph<State, Transition, StateIndexer>::AddVertices(const std::vector<State>& states) {
  // Reserve space for better performance
  vertex_map_.reserve(vertex_map_.size() + states.size());
  for (const auto& state : states) {
    AddVertex(state);
  }
}

template <typename State, typename Transition, typename StateIndexer>
void Graph<State, Transition, StateIndexer>::AddEdges(
    const std::vector<std::tuple<State, State, Transition>>& edges) {
  // Reserve space for vertices that might be created
  std::size_t potential_new_vertices = edges.size() * 2;
  vertex_map_.reserve(vertex_map_.size() + potential_new_vertices);
  
  for (const auto& edge : edges) {
    AddEdge(std::get<0>(edge), std::get<1>(edge), std::get<2>(edge));
  }
}

template <typename State, typename Transition, typename StateIndexer>
void Graph<State, Transition, StateIndexer>::RemoveVertices(const std::vector<State>& states) {
  for (const auto& state : states) {
    RemoveVertex(state);
  }
}

/*---------------------------------------------------------------------------------*/
/*                 Standardized Return Types Implementation                        */
/*---------------------------------------------------------------------------------*/

// Consistent Add Operations
template <typename State, typename Transition, typename StateIndexer>
std::pair<typename Graph<State, Transition, StateIndexer>::vertex_iterator, bool>
Graph<State, Transition, StateIndexer>::AddVertexWithResult(State state) {
  int64_t state_id = GetStateIndex(state);
  auto it = vertex_map_.find(state_id);
  
  if (it == vertex_map_.end()) {
    // Vertex doesn't exist, add it
    auto vertex_it = AddVertex(state);
    return std::make_pair(vertex_it, true);
  } else {
    // Vertex already exists
    return std::make_pair(vertex_iterator(it), false);
  }
}

template <typename State, typename Transition, typename StateIndexer>
bool Graph<State, Transition, StateIndexer>::AddEdgeWithResult(State from, State to, Transition trans) {
  try {
    // Check if both vertices exist
    auto from_it = FindVertex(from);
    auto to_it = FindVertex(to);

    // If vertices don't exist, we can't add edge in "WithResult" mode
    // This is more conservative than the regular AddEdge which creates vertices
    if (from_it == vertex_end() || to_it == vertex_end()) {
      return false;
    }

    Vertex* from_ptr = &(*from_it);
    Vertex* to_ptr = &(*to_it);

    // Check if edge already exists using FindEdge (returns non-const iterator)
    auto edge_it = from_it->FindEdge(to);
    if (edge_it != from_it->edge_end()) {
      // Edge exists, update weight and return true
      edge_it->cost = trans;
      return true;
    }

    // Add new edge using stable Vertex* pointers
    to_ptr->vertices_from.push_back(from_ptr);
    from_ptr->edges_to.emplace_back(from_ptr, to_ptr, trans);
    return true;
  } catch (...) {
    return false;
  }
}

template <typename State, typename Transition, typename StateIndexer>
bool Graph<State, Transition, StateIndexer>::AddUndirectedEdgeWithResult(State from, State to, Transition trans) {
  try {
    // Check if both vertices exist
    auto from_it = FindVertex(from);
    auto to_it = FindVertex(to);
    
    if (from_it == vertex_end() || to_it == vertex_end()) {
      return false;
    }
    
    // Add both directed edges for undirected edge
    AddUndirectedEdge(from, to, trans);
    return true;
  } catch (...) {
    return false;
  }
}

// Consistent Remove Operations
template <typename State, typename Transition, typename StateIndexer>
bool Graph<State, Transition, StateIndexer>::RemoveVertexWithResult(int64_t vertex_id) {
  auto it = vertex_map_.find(vertex_id);
  
  if (it == vertex_map_.end()) {
    return false; // Vertex doesn't exist
  }
  
  // Vertex exists, remove it using existing method
  RemoveVertex(vertex_id);
  return true;
}

}  // namespace xmotion

#endif /* GRAPH_IMPL_HPP */
