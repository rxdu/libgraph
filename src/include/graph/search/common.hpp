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
#include <unordered_set>
#include <stdexcept>
#include "graph/graph.hpp"

namespace xmotion {
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
  // Use unordered_set with custom hash and equality functions
  std::unordered_set<VertexIterator, typename VertexIterator::Hash, 
                     typename VertexIterator::Equal> visited;
  VertexIterator waypoint = goal_vtx;
  
  // First, add the goal to visited to handle the edge case
  visited.insert(goal_vtx);
  
  while (waypoint != start_vtx) {
    path.push_back(waypoint);
    
    // Move to parent
    VertexIterator parent = waypoint->search_parent;
    
    // Check for self-loop (uninitialized parent often points to self)
    if (parent == waypoint) {
      throw std::runtime_error("Path reconstruction failed: vertex parent points to itself");
    }
    
    // Check for cycle
    if (!visited.insert(parent).second) {
      throw std::runtime_error("Path reconstruction failed: cycle detected in parent chain");
    }
    
    waypoint = parent;
  }
  
  // add the start node
  path.push_back(start_vtx);
  std::reverse(path.begin(), path.end());
#ifndef MINIMAL_PRINTOUT
  auto traj_s = path.begin();
  auto traj_e = path.end() - 1;
  std::cout << "starting vertex id: " << (*traj_s)->vertex_id << std::endl;
  std::cout << "finishing vertex id: " << (*traj_e)->vertex_id << std::endl;
  std::cout << "path length: " << path.size() << std::endl;
  std::cout << "total cost: " << path.back()->g_cost << std::endl;
#endif
  return path;
}
}  // namespace utils
}  // namespace xmotion

#endif /* COMMON_HPP */
