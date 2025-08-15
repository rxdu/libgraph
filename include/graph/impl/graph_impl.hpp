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
    Graph<State, Transition, StateIndexer> &&other) {
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
        Graph<State, Transition, StateIndexer> &&other) {
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
    auto vtx = vertex_iterator(it);
    // remove upstream connections
    // e.g. other vertices that connect to the vertex to be deleted
    for (auto &asv : vtx->vertices_from) {
      // Use list::remove_if with value capture to avoid iterator invalidation
      asv->edges_to.remove_if([vtx](const Edge& edge) { 
        return edge.dst == vtx; 
      });
    }

    // remove downstream connections
    // e.g. other vertices that are connected by the vertex to be deleted
    for (auto &edge : vtx->edges_to) {
      auto &target_vertex = edge.dst;
      // Use list::remove for vertex_iterator (simpler and more efficient)
      target_vertex->vertices_from.remove(vtx);
    }

    // remove from vertex map - unique_ptr handles cleanup automatically
    vertex_map_.erase(it);
  }
}

template <typename State, typename Transition, typename StateIndexer>
void Graph<State, Transition, StateIndexer>::AddEdge(State sstate, State dstate,
                                                     Transition trans) {
  auto src_vertex = ObtainVertexFromVertexMap(sstate);

  // update transition if edge already exists
  auto it = src_vertex->FindEdge(dstate);
  if (it != src_vertex->edge_end()) {
    it->cost = trans;
    // std::cout << "updated cost: " << trans << std::endl;
    return;
  }

  // otherwise add new edge
  auto dst_vertex = ObtainVertexFromVertexMap(dstate);
  dst_vertex->vertices_from.push_back(src_vertex);
  src_vertex->edges_to.emplace_back(src_vertex, dst_vertex, trans);
}

template <typename State, typename Transition, typename StateIndexer>
bool Graph<State, Transition, StateIndexer>::RemoveEdge(State sstate,
                                                        State dstate) {
  auto src_vertex = FindVertex(sstate);
  auto dst_vertex = FindVertex(dstate);

  if ((src_vertex != vertex_end()) && (dst_vertex != vertex_end())) {
    for (auto it = src_vertex->edges_to.begin();
         it != src_vertex->edges_to.end(); ++it) {
      if (it->dst == dst_vertex) {
        src_vertex->edges_to.erase(it);
        // Use list::remove for consistency and efficiency
        dst_vertex->vertices_from.remove(src_vertex);
        return true;
      }
    }
  }

  return false;
}

template <typename State, typename Transition, typename StateIndexer>
void Graph<State, Transition, StateIndexer>::AddUndirectedEdge(
    State sstate, State dstate, Transition trans) {
  AddEdge(sstate, dstate, trans);
  AddEdge(dstate, sstate, trans);
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
void Graph<State, Transition, StateIndexer>::ResetAllVertices() {
  for (auto &vertex_pair : vertex_map_)
    vertex_pair.second->ClearVertexSearchInfo();
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
    // Exception-safe vertex creation using unique_ptr (C++11 compatible)
    std::unique_ptr<Vertex> new_vertex(new Vertex(state, state_id));
    new_vertex->search_parent = vertex_end();
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
size_t Graph<State, Transition, StateIndexer>::GetEdgeCount() const {
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
  for (const auto& state : states) {
    AddVertex(state);
  }
}

template <typename State, typename Transition, typename StateIndexer>
void Graph<State, Transition, StateIndexer>::AddEdges(
    const std::vector<std::tuple<State, State, Transition>>& edges) {
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

}  // namespace xmotion

#endif /* GRAPH_IMPL_HPP */
