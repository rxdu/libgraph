/*
 * common.hpp
 *
 * Created on: Jan 22, 2021 23:36
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef COMMON_HPP
#define COMMON_HPP

#include <iostream>
#include "graph/graph.hpp"

namespace rdu {
template <typename State>
using Path = std::vector<State>;

template <typename State, typename Transition = double>
using GetNeighbourFunc_t =
    std::function<std::vector<std::tuple<State, Transition>>(State)>;

template <typename State, typename Transition = double>
using CalcHeuristicFunc_t = std::function<Transition(State, State)>;

namespace utils {
template <typename VertexIterator>
static std::vector<VertexIterator> ReconstructPath(VertexIterator start_vtx,
                                                   VertexIterator goal_vtx) {
  std::vector<VertexIterator> path;
  VertexIterator waypoint = goal_vtx;
  while (waypoint != start_vtx) {
    path.push_back(waypoint);
    waypoint = waypoint->search_parent;
  }
  // add the start node
  path.push_back(waypoint);
  std::reverse(path.begin(), path.end());
#ifndef MINIMAL_PRINTOUT
  auto traj_s = path.begin();
  auto traj_e = path.end() - 1;
  std::cout << "starting vertex id: " << (*traj_s)->vertex_id_ << std::endl;
  std::cout << "finishing vertex id: " << (*traj_e)->vertex_id_ << std::endl;
  std::cout << "path length: " << path.size() << std::endl;
  std::cout << "total cost: " << path.back()->g_cost << std::endl;
#endif
  return path;
}
}  // namespace utils
}  // namespace rdu

#endif /* COMMON_HPP */
