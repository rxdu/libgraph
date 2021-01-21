/*
 * dijkstra.hpp
 *
 * Created on: Nov 30, 2017 14:22
 * Description: Dijkstra's search and traversal algorithm
 *
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

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
#include "graph/details/dynamic_priority_queue.hpp"

namespace rdu {

template <typename State>
using Path = std::vector<State>;

template <typename State, typename Transition = double>
using GetNeighbourFunc_t =
    std::function<std::vector<std::tuple<State, Transition>>(State)>;

/// Dijkstra search algorithm.
class Dijkstra {
 public:
  /// Search using vertex ids
  template <typename State, typename Transition, typename StateIndexer,
            typename VertexIdentifier>
  static Path<State> Search(
      std::shared_ptr<Graph<State, Transition, StateIndexer>> graph,
      VertexIdentifier start, VertexIdentifier goal) {
    // reset last search information
    graph->ResetAllVertices();

    auto start_it = graph->FindVertex(start);
    auto goal_it = graph->FindVertex(goal);

    Path<State> path;
    // start a new search and return result
    if (start_it != graph->vertex_end() && goal_it != graph->vertex_end()) {
      auto path_vtx = PerformSearch(graph, start_it, goal_it);
      for (auto &wp : path_vtx) path.push_back(wp->state);
    }
    return path;
  }

  template <typename State, typename Transition, typename StateIndexer,
            typename VertexIdentifier>
  static Path<State> Search(Graph<State, Transition, StateIndexer> *graph,
                            VertexIdentifier start, VertexIdentifier goal) {
    // reset last search information
    graph->ResetAllVertices();

    auto start_it = graph->FindVertex(start);
    auto goal_it = graph->FindVertex(goal);

    Path<State> path;
    // start a new search and return result
    if (start_it != graph->vertex_end() && goal_it != graph->vertex_end()) {
      auto path_vtx = PerformSearch(graph, start_it, goal_it);
      for (auto &wp : path_vtx) path.push_back(wp->state);
    }
    return path;
  }

  template <typename State, typename Transition, typename StateIndexer>
  static Path<State> IncSearch(
      State sstate, State gstate,
      GetNeighbourFunc_t<State, Transition> get_neighbours,
      StateIndexer indexer) {
    Path<State> path;
    auto path_vtx = PerformIncSearch(sstate, gstate, get_neighbours, indexer);
    for (auto &wp : path_vtx) path.push_back(wp->state);
    return path;
  }

 private:
  template <typename State, typename Transition, typename StateIndexer>
  static std::vector<
      typename Graph<State, Transition, StateIndexer>::vertex_iterator>
  PerformIncSearch(State sstate, State gstate,
                   GetNeighbourFunc_t<State, Transition> get_neighbours,
                   StateIndexer indexer) {
    // type definitions
    using PathType = std::vector<
        typename Graph<State, Transition, StateIndexer>::vertex_iterator>;
    using VertexIterator =
        typename Graph<State, Transition, StateIndexer>::vertex_iterator;

    struct VertexComparator {
      bool operator()(VertexIterator x, VertexIterator y) const {
        return (x->g_cost < y->g_cost);
      }
    };

    struct VertexIndexer {
      int64_t operator()(VertexIterator vtx) const {
        return static_cast<int64_t>(vtx->vertex_id);
      }
    };

    // open list - a list of vertices that need to be checked out
    DynamicPriorityQueue<VertexIterator, VertexComparator, VertexIndexer>
        openlist;

    // create a new graph with only start and goal vertices
    Graph<State, Transition, StateIndexer> graph;
    VertexIterator start_vtx = graph.AddVertex(sstate);
    VertexIterator goal_vtx = graph.AddVertex(gstate);

    // begin with start vertex
    start_vtx->g_cost = 0;
    openlist.Push(start_vtx);

    // start search iterations
    bool found_path = false;
    VertexIterator current_vertex;
    while (!openlist.Empty() && found_path != true) {
      current_vertex = openlist.Pop();
      current_vertex->is_checked = true;
      if (current_vertex == goal_vtx) {
        found_path = true;
        break;
      }

      std::vector<std::tuple<State, Transition>> neighbours =
          get_neighbours(current_vertex->state);
      for (auto &nb : neighbours)
        graph.AddEdge(current_vertex->state, std::get<0>(nb), std::get<1>(nb));

      // check all adjacent vertices (successors of current vertex)
      for (auto &edge : current_vertex->edges_to) {
        auto successor = edge.dst;

        // check if the vertex has been checked (in closed list)
        if (successor->is_checked == false) {
          // set the parent of the adjacent vertex to be the current vertex
          auto new_cost = current_vertex->g_cost + edge.cost;

          // relax step
          if (new_cost < successor->g_cost) {
            successor->search_parent = current_vertex;
            successor->g_cost = new_cost;
            openlist.Push(successor);
          }
        }
      }
    }

    // reconstruct path from search
    if (found_path) {
      std::cout << "path found with cost " << goal_vtx->g_cost << std::endl;
      return ReconstructPath(&graph, start_vtx, goal_vtx);
    }
    std::cout << "failed to find a path" << std::endl;
    return PathType();
  };

  template <typename State, typename Transition, typename StateIndexer>
  static std::vector<
      typename Graph<State, Transition, StateIndexer>::vertex_iterator>
  PerformSearch(Graph<State, Transition, StateIndexer> *graph,
                typename Graph<State, Transition, StateIndexer>::vertex_iterator
                    start_vtx,
                typename Graph<State, Transition, StateIndexer>::vertex_iterator
                    goal_vtx) {
    // type definitions
    using PathType = std::vector<
        typename Graph<State, Transition, StateIndexer>::vertex_iterator>;
    using VertexIterator =
        typename Graph<State, Transition, StateIndexer>::vertex_iterator;

    struct VertexComparator {
      bool operator()(VertexIterator x, VertexIterator y) const {
        return (x->g_cost < y->g_cost);
      }
    };

    struct VertexIndexer {
      int64_t operator()(VertexIterator vtx) const {
        return static_cast<int64_t>(vtx->vertex_id);
      }
    };

    // open list - a list of vertices that need to be checked out
    DynamicPriorityQueue<VertexIterator, VertexComparator, VertexIndexer>
        openlist;

    // begin with start vertex
    start_vtx->g_cost = 0;
    openlist.Push(start_vtx);

    // start search iterations
    bool found_path = false;
    VertexIterator current_vertex;
    while (!openlist.Empty() && found_path != true) {
      current_vertex = openlist.Pop();
      current_vertex->is_checked = true;
      if (current_vertex == goal_vtx) {
        found_path = true;
        break;
      }

      // check all adjacent vertices (successors of current vertex)
      for (auto &edge : current_vertex->edges_to) {
        auto successor = edge.dst;
        // check if the vertex has been checked (in closed list)
        if (successor->is_checked == false) {
          // set the parent of the adjacent vertex to be the current vertex
          auto new_cost = current_vertex->g_cost + edge.cost;

          // relax step
          if (new_cost < successor->g_cost) {
            successor->search_parent = current_vertex;
            successor->g_cost = new_cost;
            openlist.Push(successor);
          }
        }
      }
    }

    // reconstruct path from search
    if (found_path) {
      std::cout << "path found with cost " << goal_vtx->g_cost << std::endl;
      return ReconstructPath(graph, start_vtx, goal_vtx);
    }
    std::cout << "failed to find a path" << std::endl;
    return PathType();
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

#endif /* DIJKSTRA_HPP */
