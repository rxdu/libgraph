/*
 * edge_impl.hpp
 *
 * Created on: Sep 04, 2018 01:37
 * Description: Implementation for independent Edge class
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef EDGE_IMPL_HPP
#define EDGE_IMPL_HPP

namespace xmotion {

template <typename State, typename Transition, typename StateIndexer>
bool Edge<State, Transition, StateIndexer>::operator==(
    const Edge<State, Transition, StateIndexer>& other) const {
  if (src == other.src && dst == other.dst && cost == other.cost) return true;
  return false;
}

template <typename State, typename Transition, typename StateIndexer>
void Edge<State, Transition, StateIndexer>::PrintEdge() const {
  // Access vertex through Graph's vertex_iterator -> operator (handles dereferencing automatically)
  std::cout << "Edge_t: src - " << src->GetVertexID() << " , dst - "
            << dst->GetVertexID() << " , cost - " << cost << std::endl;
}

}  // namespace xmotion

#endif /* EDGE_IMPL_HPP */
