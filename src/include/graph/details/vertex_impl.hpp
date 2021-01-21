/*
 * vertex_impl.hpp
 *
 * Created on: Sep 04, 2018 01:43
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef VERTEX_IMPL_HPP
#define VERTEX_IMPL_HPP

namespace rdu {
template <typename State, typename Transition, typename StateIndexer>
bool Graph<State, Transition, StateIndexer>::Vertex::operator==(
    const Graph<State, Transition, StateIndexer>::Vertex &other) {
  if (vertex_id == other.vertex_id) return true;
  return false;
}

template <typename State, typename Transition, typename StateIndexer>
typename Graph<State, Transition, StateIndexer>::Vertex::edge_iterator
Graph<State, Transition, StateIndexer>::Vertex::FindEdge(int64_t dst_id) {
  typename Graph<State, Transition, StateIndexer>::Vertex::edge_iterator it;
  for (it = edge_begin(); it != edge_end(); ++it) {
    if (it->dst->vertex_id == dst_id) return it;
  }
  return it;
}

template <typename State, typename Transition, typename StateIndexer>
template <class T = State, typename std::enable_if<
                               !std::is_integral<T>::value>::type * = nullptr>
typename Graph<State, Transition, StateIndexer>::Vertex::edge_iterator
Graph<State, Transition, StateIndexer>::Vertex::FindEdge(T dst_state) {
  typename Graph<State, Transition, StateIndexer>::Vertex::edge_iterator it;
  for (it = edge_begin(); it != edge_end(); ++it) {
    if (this->GetStateIndex(it->dst->state) == this->GetStateIndex(dst_state))
      return it;
  }
  return it;
}

template <typename State, typename Transition, typename StateIndexer>
template <typename T>
bool Graph<State, Transition, StateIndexer>::Vertex::CheckNeighbour(T dst) {
  auto res = FindEdge(dst);
  if (res != edge_end()) return true;
  return false;
}

template <typename State, typename Transition, typename StateIndexer>
std::vector<typename Graph<State, Transition, StateIndexer>::vertex_iterator>
Graph<State, Transition, StateIndexer>::Vertex::GetNeighbours() {
  std::vector<typename Graph<State, Transition, StateIndexer>::vertex_iterator>
      nbs;
  for (auto it = edge_begin(); it != edge_end(); ++it) nbs.push_back(it->dst);
  return nbs;
}

template <typename State, typename Transition, typename StateIndexer>
void Graph<State, Transition, StateIndexer>::Vertex::ClearVertexSearchInfo() {
  is_checked = false;
  is_in_openlist = false;  // to be removed
  search_parent = vertex_iterator();

  f_cost = std::numeric_limits<double>::max();
  g_cost = std::numeric_limits<double>::max();
  h_cost = std::numeric_limits<double>::max();
}
}  // namespace rdu

#endif /* VERTEX_IMPL_HPP */
