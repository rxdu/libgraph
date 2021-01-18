/*
 * astar.hpp
 *
 * Created on: Jan 18, 2016
 * Description: A* algorithm
 * Reference:
 *  	1. http://www.redblobgames.com/pathfinding/a-star/implementation.html
 *  	2. https://oopscenities.net/2012/02/24/c11-stdfunction-and-stdbind/
 *
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <vector>
#include <tuple>
#include <queue>
#include <functional>
#include <utility>
#include <cmath>
#include <algorithm>
#include <type_traits>
#include <functional>
#include <iostream>
#include <memory>

#include "graph/graph.hpp"
#include "graph/details/priority_queue.hpp"

namespace rdu {
template <typename State>
using Path = std::vector<State>;

template <typename State, typename Transition = double>
using GetNeighbourFunc_t =
    std::function<std::vector<std::tuple<State, Transition>>(State)>;

template <typename State, typename Transition = double>
using CalcHeuristicFunc_t = std::function<Transition(State, State)>;

/// A* search algorithm.
class AStar {
 public:
  /// Search using vertex ids
  template <typename State, typename Transition, typename StateIndexer,
            typename VertexIdentifier>
  static Path<State> Search(
      std::shared_ptr<Graph<State, Transition, StateIndexer>> graph,
      VertexIdentifier start, VertexIdentifier goal,
      CalcHeuristicFunc_t<State, Transition> calc_heuristic) {
    // reset last search information
    graph->ResetAllVertices();

    auto start_it = graph->FindVertex(start);
    auto goal_it = graph->FindVertex(goal);

    Path<State> empty;

    // start a new search and return result
    if (start_it != graph->vertex_end() && goal_it != graph->vertex_end())
      return PerformSearch(graph, start_it, goal_it, calc_heuristic);
    else
      return empty;
  }

  template <typename State, typename Transition, typename StateIndexer,
            typename VertexIdentifier>
  static Path<State> Search(
      Graph<State, Transition, StateIndexer> *graph, VertexIdentifier start,
      VertexIdentifier goal,
      CalcHeuristicFunc_t<State, Transition> calc_heuristic) {
    // reset last search information
    graph->ResetAllVertices();

    auto start_it = graph->FindVertex(start);
    auto goal_it = graph->FindVertex(goal);

    Path<State> empty;

    // start a new search and return result
    if (start_it != graph->vertex_end() && goal_it != graph->vertex_end())
      return PerformSearch(graph, start_it, goal_it, calc_heuristic);
    else
      return empty;
  }

  template <typename State, typename Transition, typename StateIndexer>
  static Path<State> IncSearch(
      State sstate, State gstate,
      GetNeighbourFunc_t<State, Transition> get_neighbours,
      CalcHeuristicFunc_t<State, Transition> calc_heuristic,
      StateIndexer indexer) {
    using VertexIterator =
        typename Graph<State, Transition, StateIndexer>::vertex_iterator;

    // create a new graph with only start and goal vertices
    Graph<State, Transition, StateIndexer> graph;

    VertexIterator start_vtx = graph.AddVertex(sstate);
    VertexIterator goal_vtx = graph.AddVertex(gstate);

    // open list - a list of vertices that need to be checked out
    PriorityQueue<VertexIterator> openlist;

    // begin with start vertex
    openlist.put(start_vtx, 0);
    start_vtx->is_in_openlist = true;
    start_vtx->g_cost = 0;

    // start search iterations
    bool found_path = false;
    VertexIterator current_vertex;
    while (!openlist.empty() && found_path != true) {
      current_vertex = openlist.get();
      if (current_vertex->is_checked) continue;
      if (current_vertex == goal_vtx) {
        found_path = true;
        break;
      }

      current_vertex->is_in_openlist = false;
      current_vertex->is_checked = true;

      std::vector<std::tuple<State, Transition>> neighbours =
          get_neighbours(current_vertex->state);
      for (auto &nb : neighbours)
        graph.AddEdge(current_vertex->state, std::get<0>(nb), std::get<1>(nb));

      // check all adjacent vertices (successors of current vertex)
      for (auto &edge : current_vertex->edges_to) {
        auto successor = edge.dst;

        // check if the vertex has been checked (in closed list)
        if (successor->is_checked == false) {
          auto new_cost = current_vertex->g_cost + edge.cost;

          // if the vertex is not in open list
          // or if the vertex is in open list but has a higher cost
          if (successor->is_in_openlist == false ||
              new_cost < successor->g_cost) {
            // first set the parent of the adjacent vertex to be the current
            // vertex
            successor->search_parent = current_vertex;

            // update costs
            successor->g_cost = new_cost;
            successor->h_cost =
                calc_heuristic(successor->state, goal_vtx->state);
            successor->f_cost = successor->g_cost + successor->h_cost;

            // put vertex into open list
            openlist.put(successor, successor->f_cost);
            successor->is_in_openlist = true;
          }
        }
      }
    }

    // reconstruct path from search
    Path<State> path;
    if (found_path) {
      std::cout << "path found with cost " << goal_vtx->g_cost << std::endl;
      auto path_vtx = ReconstructPath(&graph, start_vtx, goal_vtx);
      for (auto &wp : path_vtx) path.push_back(wp->state);
    } else
      std::cout << "failed to find a path" << std::endl;

    return path;
  };

 private:
  template <typename State, typename Transition, typename StateIndexer>
  static Path<State> PerformSearch(
      Graph<State, Transition, StateIndexer> *graph,
      typename Graph<State, Transition, StateIndexer>::vertex_iterator
          start_vtx,
      typename Graph<State, Transition, StateIndexer>::vertex_iterator goal_vtx,
      std::function<Transition(State, State)> CalcHeuristic) {
    using VertexIterator =
        typename Graph<State, Transition, StateIndexer>::vertex_iterator;

    // open list - a list of vertices that need to be checked out
    PriorityQueue<VertexIterator> openlist;

    // begin with start vertex
    openlist.put(start_vtx, 0);
    start_vtx->is_in_openlist = true;
    start_vtx->g_cost = 0;

    // start search iterations
    bool found_path = false;
    VertexIterator current_vertex;
    while (!openlist.empty() && found_path != true) {
      current_vertex = openlist.get();
      if (current_vertex->is_checked) continue;
      if (current_vertex == goal_vtx) {
        found_path = true;
        break;
      }

      current_vertex->is_in_openlist = false;
      current_vertex->is_checked = true;

      // check all adjacent vertices (successors of current vertex)
      for (auto &edge : current_vertex->edges_to) {
        auto successor = edge.dst;

        // check if the vertex has been checked (in closed list)
        if (successor->is_checked == false) {
          auto new_cost = current_vertex->g_cost + edge.cost;

          // if the vertex is not in open list
          // or if the vertex is in open list but has a higher cost
          if (successor->is_in_openlist == false ||
              new_cost < successor->g_cost) {
            // first set the parent of the adjacent vertex to be the current
            // vertex
            successor->search_parent = current_vertex;

            // update costs
            successor->g_cost = new_cost;
            successor->h_cost =
                CalcHeuristic(successor->state, goal_vtx->state);
            successor->f_cost = successor->g_cost + successor->h_cost;

            // put vertex into open list
            openlist.put(successor, successor->f_cost);
            successor->is_in_openlist = true;
          }
        }
      }
    }

    // reconstruct path from search
    Path<State> path;
    if (found_path) {
      std::cout << "path found with cost " << goal_vtx->g_cost << std::endl;
      auto path_vtx = ReconstructPath(graph, start_vtx, goal_vtx);
      for (auto &wp : path_vtx) path.push_back(wp->state);
    } else
      std::cout << "failed to find a path" << std::endl;

    return path;
  };

  template <typename State, typename Transition, typename StateIndexer>
  static std::vector<
      typename Graph<State, Transition, StateIndexer>::vertex_iterator>
  ReconstructPath(Graph<State, Transition, StateIndexer> *graph,
                  typename Graph<State, Transition,
                                 StateIndexer>::vertex_iterator start_vtx,
                  typename Graph<State, Transition,
                                 StateIndexer>::vertex_iterator goal_vtx) {
    using VertexIterator =
        typename Graph<State, Transition, StateIndexer>::vertex_iterator;
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
};
}  // namespace rdu

#endif /* ASTAR_HPP */
