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

    Path<State> empty;

    // start a new search and return result
    if (start_it != graph->vertex_end() && goal_it != graph->vertex_end())
      return PerformSearch(graph, start_it, goal_it);
    else
      return empty;
  }

  template <typename State, typename Transition, typename StateIndexer,
            typename VertexIdentifier>
  static Path<State> Search(Graph<State, Transition, StateIndexer> *graph,
                            VertexIdentifier start, VertexIdentifier goal) {
    // reset last search information
    graph->ResetAllVertices();

    auto start_it = graph->FindVertex(start);
    auto goal_it = graph->FindVertex(goal);

    Path<State> empty;

    // start a new search and return result
    if (start_it != graph->vertex_end() && goal_it != graph->vertex_end())
      return PerformSearch(graph, start_it, goal_it);
    else
      return empty;
  }

  template <typename State, typename Transition, typename StateIndexer>
  static Path<State> IncSearch(
      State sstate, State gstate,
      GetNeighbourFunc_t<State, Transition> get_neighbours,
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
    openlist.Push(start_vtx, 0);
    start_vtx->is_in_openlist = true;
    start_vtx->g_cost = 0;

    // start search iterations
    bool found_path = false;
    VertexIterator current_vertex;
    while (!openlist.Empty() && found_path != true) {
      current_vertex = openlist.Pop();
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
          // first set the parent of the adjacent vertex to be the current
          // vertex
          auto new_cost = current_vertex->g_cost + edge.cost;

          // if the vertex is not in open list
          // or if the vertex is in open list but has a higher cost
          if (successor->is_in_openlist == false ||
              new_cost < successor->g_cost) {
            successor->search_parent = current_vertex;
            successor->g_cost = new_cost;

            openlist.Push(successor, successor->g_cost);
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

 public:
  template <typename State, typename Transition, typename StateIndexer>
  static Path<State> PerformSearch(
      Graph<State, Transition, StateIndexer> *graph,
      typename Graph<State, Transition, StateIndexer>::vertex_iterator
          start_vtx,
      typename Graph<State, Transition, StateIndexer>::vertex_iterator
          goal_vtx) {
    using VertexIterator =
        typename Graph<State, Transition, StateIndexer>::vertex_iterator;

    // open list - a list of vertices that need to be checked out
    PriorityQueue<VertexIterator> openlist;

    // begin with start vertex
    openlist.Push(start_vtx, 0);
    start_vtx->is_in_openlist = true;
    start_vtx->g_cost = 0;

    // start search iterations
    bool found_path = false;
    VertexIterator current_vertex;
    while (!openlist.Empty() && found_path != true) {
      current_vertex = openlist.Pop();
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
          // first set the parent of the adjacent vertex to be the current
          // vertex
          auto new_cost = current_vertex->g_cost + edge.cost;

          // if the vertex is not in open list
          // or if the vertex is in open list but has a higher cost
          if (successor->is_in_openlist == false ||
              new_cost < successor->g_cost) {
            successor->search_parent = current_vertex;
            successor->g_cost = new_cost;

            openlist.Push(successor, successor->g_cost);
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

/************************************************************************************************/

/// Dijkstra traversal algorithm.
class DijkstraTraversal {
 public:
  /// Traverse graph from specified vertex id
  template <typename State, typename Transition, typename StateIndexer,
            typename VertexIdentifier>
  void Run(Graph<State, Transition, StateIndexer> *graph,
           VertexIdentifier start) {
    // reset last search information
    graph->ResetAllVertices();

    auto start_it = graph->FindVertex(start);

    // start traversal
    if (start_it != graph->vertex_end()) Traverse(start_it);
  }

  /// Traverse graph from specified vertex id
  template <typename State, typename Transition, typename StateIndexer>
  static Graph<State, Transition, StateIndexer> RunInc(
      State sstate, GetNeighbourFunc_t<State, Transition> get_neighbours) {
    using VertexIterator =
        typename Graph<State, Transition, StateIndexer>::vertex_iterator;

    // create a new graph with only start and goal vertices
    Graph<State, Transition, StateIndexer> graph;
    graph.AddVertex(sstate);

    VertexIterator start_vtx = graph.FindVertex(sstate);

    // open list - a list of vertices that need to be checked out
    PriorityQueue<VertexIterator> openlist;

    // begin with start vertex
    openlist.Push(start_vtx, 0);
    start_vtx->is_in_openlist = true;
    start_vtx->g_cost = 0;

    // start search iterations
    VertexIterator current_vertex;
    while (!openlist.Empty()) {
      current_vertex = openlist.Pop();
      if (current_vertex->is_checked) continue;

      current_vertex->is_in_openlist = false;
      current_vertex->is_checked = true;

      std::vector<std::tuple<State, Transition>> neighbours =
          get_neighbours(current_vertex->state);
      for (auto &nb : neighbours)
        graph.AddEdge(current_vertex->state, std::get<0>(nb), std::get<1>(nb));

      // check all adjacent vertices (successors of current vertex)
      for (auto edge = current_vertex->edge_begin();
           edge != current_vertex->edge_end(); ++edge) {
        auto successor = edge->dst;

        // check if the vertex has been checked (in closed list)
        if (successor->is_checked == false) {
          // first set the parent of the adjacent vertex to be the current
          // vertex
          auto new_cost = current_vertex->g_cost + edge->cost;

          // if the vertex is not in open list
          // or if the vertex is in open list but has a higher cost
          if (successor->is_in_openlist == false ||
              new_cost < successor->g_cost) {
            successor->search_parent = current_vertex;
            successor->g_cost = new_cost;

            openlist.Push(successor, successor->g_cost);
            successor->is_in_openlist = true;
          }
        }
      }
    }

    return graph;
  };

 private:
  template <typename State, typename Transition, typename StateIndexer>
  static void Traverse(
      Graph<State, Transition, StateIndexer> *graph,
      typename Graph<State, Transition, StateIndexer>::vertex_iterator
          start_vtx) {
    using VertexIterator =
        typename Graph<State, Transition, StateIndexer>::vertex_iterator;

    // open list - a list of vertices that need to be checked out
    PriorityQueue<VertexIterator> openlist;

    // begin with start vertex
    openlist.Push(start_vtx, 0);
    start_vtx->is_in_openlist = true;
    start_vtx->g_cost = 0;

    // start search iterations
    VertexIterator current_vertex;
    while (!openlist.Empty()) {
      current_vertex = openlist.Pop();
      if (current_vertex->is_checked) continue;

      current_vertex->is_in_openlist = false;
      current_vertex->is_checked = true;

      // check all adjacent vertices (successors of current vertex)
      for (auto edge = current_vertex->edge_begin();
           edge != current_vertex->edge_end(); ++edge) {
        auto successor = edge->dst;

        // check if the vertex has been checked (in closed list)
        if (successor->is_checked == false) {
          // first set the parent of the adjacent vertex to be the current
          // vertex
          auto new_cost = current_vertex->g_cost + edge->cost;

          // if the vertex is not in open list
          // or if the vertex is in open list but has a higher cost
          if (successor->is_in_openlist == false ||
              new_cost < successor->g_cost) {
            successor->search_parent = current_vertex;
            successor->g_cost = new_cost;

            openlist.Push(successor, successor->g_cost);
            successor->is_in_openlist = true;
          }
        }
      }
    }
  };
};

}  // namespace rdu

#endif /* DIJKSTRA_HPP */
