/*
 * edge_impl.hpp
 *
 * Created on: Sep 04, 2018 01:37
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef EDGE_IMPL_HPP
#define EDGE_IMPL_HPP

namespace rdu {
template <typename State, typename Transition, typename StateIndexer>
bool Graph<State, Transition, StateIndexer>::Edge::operator==(
    const Graph<State, Transition, StateIndexer>::Edge &other) {
  if (src == other.src && dst == other.dst && cost == other.cost) return true;
  return false;
}

template <typename State, typename Transition, typename StateIndexer>
void Graph<State, Transition, StateIndexer>::Edge::PrintEdge() {
  std::cout << "Edge_t: src - " << src->GetVertexID() << " , dst - "
            << dst->GetVertexID() << " , cost - " << cost << std::endl;
}

}  // namespace rdu

#endif /* EDGE_IMPL_HPP */
