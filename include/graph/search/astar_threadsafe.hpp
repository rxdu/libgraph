/*
 * astar_threadsafe.hpp
 *
 * Created on: 2025
 * Description: Thread-safe A* search algorithm using external search context
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef ASTAR_THREADSAFE_HPP
#define ASTAR_THREADSAFE_HPP

#include <vector>
#include <queue>
#include <functional>
#include <utility>
#include <cmath>
#include <algorithm>

#include "graph/graph.hpp"
#include "graph/search/search_context.hpp"
#include "graph/search/common.hpp"

namespace xmotion {

/// Thread-safe A* search algorithm using external search context
class AStarThreadSafe final {
public:
  /**
   * @brief Thread-safe A* search with external context
   * 
   * This version of A* uses an external SearchContext to store
   * search state, allowing multiple concurrent searches on the same graph.
   * 
   * @tparam State The state type
   * @tparam Transition The transition/cost type  
   * @tparam StateIndexer The state indexer type
   * @tparam VertexIdentifier Type that can identify a vertex (State or int64_t)
   * @tparam HeuristicFunc Function type for heuristic (State, State) -> double
   * 
   * @param graph Const pointer to the graph (read-only access)
   * @param context Reference to search context for this search
   * @param start Starting vertex identifier
   * @param goal Goal vertex identifier  
   * @param heuristic Heuristic function h(current, goal) -> cost
   * @return Vector of states representing the path, empty if no path found
   */
  template <typename State, typename Transition, typename StateIndexer,
            typename VertexIdentifier, typename HeuristicFunc>
  static Path<State> Search(
      const Graph<State, Transition, StateIndexer>* graph,
      SearchContext<State, Transition, StateIndexer>& context,
      VertexIdentifier start, VertexIdentifier goal,
      HeuristicFunc heuristic) {
    
    // Clear any previous search state
    context.Clear();
    
    // Find start and goal vertices
    auto start_vertex = FindVertexHelper(graph, start);
    auto goal_vertex = FindVertexHelper(graph, goal);
    
    if (start_vertex == graph->vertex_end() || goal_vertex == graph->vertex_end()) {
      return Path<State>(); // Empty path if start or goal not found
    }

    // Priority queue for vertices to explore
    // Pair: (f_cost, vertex_id) - ordered by f_cost
    using QueueElement = std::pair<double, int64_t>;
    std::priority_queue<QueueElement, std::vector<QueueElement>, 
                       std::greater<QueueElement>> open_list;

    // Initialize start vertex
    auto& start_info = context.GetSearchInfo(start_vertex->vertex_id);
    start_info.g_cost = 0.0;
    start_info.h_cost = heuristic(start_vertex->state, goal_vertex->state);
    start_info.f_cost = start_info.g_cost + start_info.h_cost;
    start_info.parent_id = -1;
    start_info.is_in_openlist = true;
    
    open_list.push({start_info.f_cost, start_vertex->vertex_id});

    // Main search loop
    while (!open_list.empty()) {
      // Get vertex with minimum f_cost
      auto current_element = open_list.top();
      open_list.pop();
      
      double current_f_cost = current_element.first;
      int64_t current_id = current_element.second;
      
      auto& current_info = context.GetSearchInfo(current_id);
      
      // Skip if already processed
      if (current_info.is_checked) {
        continue;
      }
      
      // Skip if we found a better path while this was in queue
      if (current_f_cost > current_info.f_cost) {
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
        auto neighbor_vertex = edge_it->dst;
        auto& neighbor_info = context.GetSearchInfo(neighbor_id);
        
        // Skip if already processed
        if (neighbor_info.is_checked) {
          continue;
        }
        
        // Calculate new g_cost
        double new_g_cost = current_info.g_cost + edge_it->cost;
        
        // Update if we found a better path
        if (new_g_cost < neighbor_info.g_cost) {
          neighbor_info.g_cost = new_g_cost;
          
          // Calculate or reuse h_cost
          if (neighbor_info.h_cost == std::numeric_limits<double>::max()) {
            neighbor_info.h_cost = heuristic(neighbor_vertex->state, goal_vertex->state);
          }
          
          neighbor_info.f_cost = neighbor_info.g_cost + neighbor_info.h_cost;
          neighbor_info.parent_id = current_id;
          
          if (!neighbor_info.is_in_openlist) {
            neighbor_info.is_in_openlist = true;
            open_list.push({neighbor_info.f_cost, neighbor_id});
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
            typename VertexIdentifier, typename HeuristicFunc>
  static Path<State> Search(
      const Graph<State, Transition, StateIndexer>* graph,
      VertexIdentifier start, VertexIdentifier goal,
      HeuristicFunc heuristic) {
    
    SearchContext<State, Transition, StateIndexer> context;
    return Search(graph, context, start, goal, heuristic);
  }

  /**
   * @brief A* search with std::function heuristic (for backward compatibility)
   */
  template <typename State, typename Transition, typename StateIndexer,
            typename VertexIdentifier>
  static Path<State> Search(
      const Graph<State, Transition, StateIndexer>* graph,
      SearchContext<State, Transition, StateIndexer>& context,
      VertexIdentifier start, VertexIdentifier goal,
      std::function<double(State, State)> heuristic) {
    
    return Search(graph, context, start, goal, 
                 [&heuristic](const State& s1, const State& s2) {
                   return heuristic(s1, s2);
                 });
  }

  /**
   * @brief A* search with std::function heuristic and own context
   */
  template <typename State, typename Transition, typename StateIndexer,
            typename VertexIdentifier>
  static Path<State> Search(
      const Graph<State, Transition, StateIndexer>* graph,
      VertexIdentifier start, VertexIdentifier goal,
      std::function<double(State, State)> heuristic) {
    
    SearchContext<State, Transition, StateIndexer> context;
    return Search(graph, context, start, goal, heuristic);
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

#endif /* ASTAR_THREADSAFE_HPP */