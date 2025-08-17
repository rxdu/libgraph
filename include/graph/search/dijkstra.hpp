/*
 * dijkstra.hpp
 *
 * Created on: Nov 30, 2017 14:22
 * Description: Dijkstra's search algorithm using unified search framework
 *              Combined strategy implementation and public API
 *
 * Copyright (c) 2017-2025 Ruixiang Du (rdu)
 */

#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include <limits>
#include "graph/search/search_algorithm.hpp"
#include "graph/search/search_strategy.hpp"

namespace xmotion {

/**
 * @brief Dijkstra search strategy implementation
 * 
 * Implements Dijkstra's shortest path algorithm using only g(n) cost
 * (actual distance from start). This guarantees finding the optimal path
 * in graphs with non-negative edge weights.
 */
template<typename State, typename Transition, typename StateIndexer>
class DijkstraStrategy : public SearchStrategy<DijkstraStrategy<State, Transition, StateIndexer>,
                                              State, Transition, StateIndexer> {
public:
    using Base = SearchStrategy<DijkstraStrategy<State, Transition, StateIndexer>,
                               State, Transition, StateIndexer>;
    using GraphType = typename Base::GraphType;
    using vertex_iterator = typename Base::vertex_iterator;
    using SearchInfo = typename Base::SearchInfo;
    
    DijkstraStrategy() = default;
    
    double GetPriorityImpl(const SearchInfo& info) const noexcept {
        return info.g_cost;
    }
    
    void InitializeVertexImpl(SearchInfo& info, vertex_iterator vertex, 
                             vertex_iterator goal_vertex) const {
        info.g_cost = 0.0;
        info.h_cost = 0.0;  // Dijkstra doesn't use heuristic
        info.f_cost = 0.0;  // Same as g_cost for Dijkstra
        info.is_checked = false;
        info.is_in_openlist = false;
        info.parent_id = -1;
    }
    
    bool RelaxVertexImpl(SearchInfo& current_info, SearchInfo& successor_info,
                        vertex_iterator successor_vertex, vertex_iterator goal_vertex,
                        double edge_cost) const {
        
        double new_cost = current_info.g_cost + edge_cost;
        
        if (new_cost < successor_info.g_cost) {
            successor_info.g_cost = new_cost;
            successor_info.h_cost = 0.0;  // No heuristic in Dijkstra
            successor_info.f_cost = new_cost;  // f = g for Dijkstra
            return true;
        }
        
        return false;
    }
    
    // Use default implementations for optional methods
    using Base::ProcessVertexImpl;
    using Base::IsGoalReachedImpl;
};

/**
 * @brief Helper function to create Dijkstra strategy with automatic type deduction
 */
template<typename State, typename Transition, typename StateIndexer>
DijkstraStrategy<State, Transition, StateIndexer> MakeDijkstraStrategy() {
    return DijkstraStrategy<State, Transition, StateIndexer>();
}

/**
 * @brief Dijkstra search algorithm - unified implementation
 * 
 * This implementation uses the template-based search framework with strategy pattern,
 * eliminating code duplication and providing thread-safety through SearchContext.
 * It maintains backward compatibility with the original Dijkstra API.
 */
class Dijkstra final {
public:
    /**
     * @brief Thread-safe Dijkstra search with external search context
     */
    template<typename State, typename Transition, typename StateIndexer,
             typename VertexIdentifier>
    static Path<State> Search(
        const Graph<State, Transition, StateIndexer>* graph,
        SearchContext<State, Transition, StateIndexer>& context,
        VertexIdentifier start,
        VertexIdentifier goal) {
        
        if (!graph) return Path<State>();
        
        auto start_it = graph->FindVertex(start);
        auto goal_it = graph->FindVertex(goal);
        
        if (start_it == graph->vertex_end() || goal_it == graph->vertex_end()) {
            return Path<State>();
        }
        
        auto strategy = MakeDijkstraStrategy<State, Transition, StateIndexer>();
        return SearchAlgorithm<decltype(strategy), State, Transition, StateIndexer>
            ::Search(graph, context, start_it, goal_it, strategy);
    }
    
    /**
     * @brief Convenience overload with shared_ptr graph
     */
    template<typename State, typename Transition, typename StateIndexer,
             typename VertexIdentifier>
    static Path<State> Search(
        std::shared_ptr<Graph<State, Transition, StateIndexer>> graph,
        SearchContext<State, Transition, StateIndexer>& context,
        VertexIdentifier start,
        VertexIdentifier goal) {
        
        return Search(graph.get(), context, start, goal);
    }
    
    /**
     * @brief Legacy-compatible search that manages its own context (non-thread-safe)
     */
    template<typename State, typename Transition, typename StateIndexer,
             typename VertexIdentifier>
    static Path<State> Search(
        const Graph<State, Transition, StateIndexer>* graph,
        VertexIdentifier start,
        VertexIdentifier goal) {
        
        SearchContext<State, Transition, StateIndexer> context;
        return Search(graph, context, start, goal);
    }
    
    /**
     * @brief Legacy-compatible search with shared_ptr (non-thread-safe)
     */
    template<typename State, typename Transition, typename StateIndexer,
             typename VertexIdentifier>
    static Path<State> Search(
        std::shared_ptr<Graph<State, Transition, StateIndexer>> graph,
        VertexIdentifier start,
        VertexIdentifier goal) {
        
        SearchContext<State, Transition, StateIndexer> context;
        return Search(graph.get(), context, start, goal);
    }
    
    /**
     * @brief Single-source shortest paths from start to all reachable vertices
     */
    template<typename State, typename Transition, typename StateIndexer,
             typename VertexIdentifier>
    static bool SearchAll(
        const Graph<State, Transition, StateIndexer>* graph,
        SearchContext<State, Transition, StateIndexer>& context,
        VertexIdentifier start) {
        
        if (!graph) return false;
        
        auto start_it = graph->FindVertex(start);
        if (start_it == graph->vertex_end()) return false;
        
        auto strategy = MakeDijkstraStrategy<State, Transition, StateIndexer>();
        auto dummy_goal = graph->vertex_end();
        
        SearchAlgorithm<decltype(strategy), State, Transition, StateIndexer>
            ::Search(graph, context, start_it, dummy_goal, strategy);
        
        return true;
    }
};

// Compatibility typedefs for existing code
using DijkstraThreadSafe = Dijkstra;
using DijkstraV2 = Dijkstra;  // For code that already uses V2

} // namespace xmotion

#endif /* DIJKSTRA_HPP */