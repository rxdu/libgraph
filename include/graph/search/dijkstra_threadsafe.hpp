/*
 * dijkstra_threadsafe.hpp
 *
 * Created on: 2025
 * Description: Thread-safe Dijkstra's search algorithm using external search context
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef DIJKSTRA_THREADSAFE_HPP
#define DIJKSTRA_THREADSAFE_HPP

#include <vector>
#include <queue>
#include <functional>
#include <utility>
#include <cmath>
#include <algorithm>
#include <iostream>

#include "graph/graph.hpp"
#include "graph/search/search_context.hpp"
#include "graph/search/common.hpp"

namespace xmotion {

/// Thread-safe Dijkstra search algorithm using external search context
class DijkstraThreadSafe {
public:
  /**
   * @brief Thread-safe Dijkstra search with external context
   * 
   * This version of Dijkstra uses an external SearchContext to store
   * search state, allowing multiple concurrent searches on the same graph.
   * 
   * @tparam State The state type
   * @tparam Transition The transition/cost type  
   * @tparam StateIndexer The state indexer type
   * @tparam VertexIdentifier Type that can identify a vertex (State or int64_t)
   * 
   * @param graph Const pointer to the graph (read-only access)
   * @param context Reference to search context for this search
   * @param start Starting vertex identifier
   * @param goal Goal vertex identifier
   * @return Vector of states representing the path, empty if no path found
   */
  template <typename State, typename Transition, typename StateIndexer,
            typename VertexIdentifier>
  static Path<State> Search(
      const Graph<State, Transition, StateIndexer>* graph,
      SearchContext<State, Transition, StateIndexer>& context,
      VertexIdentifier start, VertexIdentifier goal) {
    
    // Clear any previous search state
    context.Clear();
    
    // Find start and goal vertices
    auto start_vertex = FindVertexHelper(graph, start);
    auto goal_vertex = FindVertexHelper(graph, goal);
    
    if (start_vertex == graph->vertex_end() || goal_vertex == graph->vertex_end()) {
      return Path<State>(); // Empty path if start or goal not found
    }

    // Priority queue for vertices to explore
    // Pair: (cost, vertex_id)
    using QueueElement = std::pair<double, int64_t>;
    std::priority_queue<QueueElement, std::vector<QueueElement>, 
                       std::greater<QueueElement>> open_list;

    // Initialize start vertex
    auto& start_info = context.GetSearchInfo(start_vertex->vertex_id);
    start_info.g_cost = 0.0;
    start_info.f_cost = 0.0;
    start_info.parent_id = -1;
    start_info.is_in_openlist = true;
    
    open_list.push({0.0, start_vertex->vertex_id});

    // Main search loop
    while (!open_list.empty()) {
      // Get vertex with minimum cost
      auto current_element = open_list.top();
      open_list.pop();
      
      double current_cost = current_element.first;
      int64_t current_id = current_element.second;
      
      auto& current_info = context.GetSearchInfo(current_id);
      
      // Skip if already processed with better cost
      if (current_info.is_checked) {
        continue;
      }
      
      // Skip if we found a better path while this was in queue
      if (current_cost > current_info.g_cost) {
        continue;
      }
      
      // Mark as processed
      current_info.is_checked = true;
      current_info.is_in_openlist = false;
      
      // Check if we reached the goal
      if (current_id == goal_vertex->vertex_id) {
        return context.ReconstructPath(graph, goal_vertex->vertex_id);
      }
      
      // Find current vertex iterator for edge traversal
      auto current_vertex = FindVertexHelper(graph, current_id);
      if (current_vertex == graph->vertex_end()) {
        continue; // Vertex disappeared (shouldn't happen with const graph)
      }
      
      // Explore neighbors
      for (auto edge_it = current_vertex->edge_begin(); 
           edge_it != current_vertex->edge_end(); ++edge_it) {
        
        int64_t neighbor_id = edge_it->dst->vertex_id;
        auto& neighbor_info = context.GetSearchInfo(neighbor_id);
        
        // Skip if already processed
        if (neighbor_info.is_checked) {
          continue;
        }
        
        // Calculate new cost
        double new_cost = current_info.g_cost + edge_it->cost;
        
        // Update if we found a better path
        if (new_cost < neighbor_info.g_cost) {
          neighbor_info.g_cost = new_cost;
          neighbor_info.f_cost = new_cost;
          neighbor_info.parent_id = current_id;
          
          if (!neighbor_info.is_in_openlist) {
            neighbor_info.is_in_openlist = true;
            open_list.push({new_cost, neighbor_id});
          }
        }
      }
    }
    
    return Path<State>(); // No path found
  }

  /**
   * @brief Convenience function that creates its own context
   * 
   * This function is thread-safe as each call gets its own context.
   * For better performance with repeated searches, reuse a context.
   */
  template <typename State, typename Transition, typename StateIndexer,
            typename VertexIdentifier>
  static Path<State> Search(
      const Graph<State, Transition, StateIndexer>* graph,
      VertexIdentifier start, VertexIdentifier goal) {
    
    SearchContext<State, Transition, StateIndexer> context;
    return Search(graph, context, start, goal);
  }

private:
  /// Helper to find vertex from different identifier types
  template <typename State, typename Transition, typename StateIndexer,
            typename VertexIdentifier>
  static typename Graph<State, Transition, StateIndexer>::const_vertex_iterator FindVertexHelper(
      const Graph<State, Transition, StateIndexer>* graph,
      VertexIdentifier identifier) {
    return graph->FindVertex(identifier);
  }
};

} // namespace xmotion

#endif /* DIJKSTRA_THREADSAFE_HPP */