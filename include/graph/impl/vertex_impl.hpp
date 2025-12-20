/*
 * vertex_impl.hpp
 *
 * Created on: Sep 04, 2018 01:43
 * Description: Implementation for independent Vertex class
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef VERTEX_IMPL_HPP
#define VERTEX_IMPL_HPP

#include <iostream>

namespace xmotion {

template <typename State, typename Transition, typename StateIndexer>
inline bool Vertex<State, Transition, StateIndexer>::operator==(
    const Vertex<State, Transition, StateIndexer>& other) const {
  return vertex_id == other.vertex_id;
}

template <typename State, typename Transition, typename StateIndexer>
typename Vertex<State, Transition, StateIndexer>::edge_iterator
Vertex<State, Transition, StateIndexer>::FindEdge(int64_t dst_id) {
  edge_iterator it;
  for (it = edge_begin(); it != edge_end(); ++it) {
    // Access vertex through Graph's vertex_iterator -> operator (handles dereferencing automatically)
    if (it->dst->vertex_id == dst_id) return it;
  }
  return it;
}

template <typename State, typename Transition, typename StateIndexer>
template <class T, typename std::enable_if<
                               !std::is_integral<T>::value>::type *>
typename Vertex<State, Transition, StateIndexer>::edge_iterator
Vertex<State, Transition, StateIndexer>::FindEdge(T dst_state) {
  edge_iterator it;
  for (it = edge_begin(); it != edge_end(); ++it) {
    // Access vertex through Graph's vertex_iterator -> operator (handles dereferencing automatically)
    if (this->GetStateIndex(it->dst->state) == this->GetStateIndex(dst_state))
      return it;
  }
  return it;
}

template <typename State, typename Transition, typename StateIndexer>
typename Vertex<State, Transition, StateIndexer>::const_edge_iterator 
Vertex<State, Transition, StateIndexer>::FindEdge(int64_t vertex_id) const {
  auto it = edge_begin();
  for (it = edge_begin(); it != edge_end(); ++it) {
    if (it->dst->GetVertexID() == vertex_id) return it;
  }
  return it;
}

template <typename State, typename Transition, typename StateIndexer>
template <class T, typename std::enable_if<!std::is_integral<T>::value>::type*>
typename Vertex<State, Transition, StateIndexer>::const_edge_iterator 
Vertex<State, Transition, StateIndexer>::FindEdge(T dst_state) const {
  auto it = edge_begin();
  for (it = edge_begin(); it != edge_end(); ++it) {
    if (this->GetStateIndex(it->dst->state) == this->GetStateIndex(dst_state))
      return it;
  }
  return it;
}

template <typename State, typename Transition, typename StateIndexer>
template <typename T>
bool Vertex<State, Transition, StateIndexer>::CheckNeighbour(T dst) {
  auto res = FindEdge(dst);
  if (res != edge_end()) return true;
  return false;
}

template <typename State, typename Transition, typename StateIndexer>
std::vector<typename Vertex<State, Transition, StateIndexer>::VertexType*>
Vertex<State, Transition, StateIndexer>::GetNeighbours() {
  std::vector<VertexType*> nbs;
  for (auto it = edge_begin(); it != edge_end(); ++it)
    nbs.push_back(it->dst);
  return nbs;
}

template <typename State, typename Transition, typename StateIndexer>
void Vertex<State, Transition, StateIndexer>::PrintVertex() const {
  std::cout << "Vertex: id - " << vertex_id << std::endl;
}

}  // namespace xmotion

#endif /* VERTEX_IMPL_HPP */
