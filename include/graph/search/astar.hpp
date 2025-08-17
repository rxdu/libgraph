/*
 * astar.hpp
 *
 * Created on: Nov 20, 2017 15:25
 * Description: A* search algorithm using unified search framework
 *              Combined strategy implementation and public API
 *
 * Copyright (c) 2017-2025 Ruixiang Du (rdu)
 */

#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <functional>
#include <limits>
#include "graph/search/search_algorithm.hpp"
#include "graph/search/search_strategy.hpp"

namespace xmotion {

/**
 * @brief A* search strategy implementation
 * 
 * Implements the A* algorithm using f(n) = g(n) + h(n) where:
 * - g(n) is the actual cost from start to node n
 * - h(n) is the heuristic estimate from node n to goal
 * - f(n) is the estimated total cost through node n
 */
template<typename State, typename Transition, typename StateIndexer, typename HeuristicFunc>
class AStarStrategy : public SearchStrategy<AStarStrategy<State, Transition, StateIndexer, HeuristicFunc>,
                                           State, Transition, StateIndexer> {
private:
    HeuristicFunc heuristic_;
    
public:
    using Base = SearchStrategy<AStarStrategy<State, Transition, StateIndexer, HeuristicFunc>,
                               State, Transition, StateIndexer>;
    using GraphType = typename Base::GraphType;
    using vertex_iterator = typename Base::vertex_iterator;
    using SearchInfo = typename Base::SearchInfo;
    
    explicit AStarStrategy(HeuristicFunc heuristic) noexcept
        : heuristic_(std::move(heuristic)) {}
    
    double GetPriorityImpl(const SearchInfo& info) const noexcept {
        return info.f_cost;
    }
    
    void InitializeVertexImpl(SearchInfo& info, vertex_iterator vertex, 
                             vertex_iterator goal_vertex) const {
        info.g_cost = 0.0;
        double h_cost = heuristic_(vertex->state, goal_vertex->state);
        info.h_cost = h_cost;
        info.f_cost = info.g_cost + h_cost;
        info.is_checked = false;
        info.is_in_openlist = false;
        info.parent_id = -1;
    }
    
    bool RelaxVertexImpl(SearchInfo& current_info, SearchInfo& successor_info,
                        vertex_iterator successor_vertex, vertex_iterator goal_vertex,
                        double edge_cost) const {
        
        double new_g_cost = current_info.g_cost + edge_cost;
        
        if (new_g_cost < successor_info.g_cost) {
            successor_info.g_cost = new_g_cost;
            double h_cost = heuristic_(successor_vertex->state, goal_vertex->state);
            successor_info.h_cost = h_cost;
            successor_info.f_cost = new_g_cost + h_cost;
            return true;
        }
        
        return false;
    }
    
    // Use default implementations for optional methods
    using Base::ProcessVertexImpl;
    using Base::IsGoalReachedImpl;
};

/**
 * @brief Helper function to create A* strategy with automatic type deduction
 */
template<typename State, typename Transition, typename StateIndexer, typename HeuristicFunc>
AStarStrategy<State, Transition, StateIndexer, typename std::decay<HeuristicFunc>::type>
MakeAStarStrategy(const HeuristicFunc& heuristic) {
    return AStarStrategy<State, Transition, StateIndexer, typename std::decay<HeuristicFunc>::type>(
        heuristic);
}

/**
 * @brief A* search algorithm - unified implementation
 * 
 * This implementation uses the template-based search framework with strategy pattern,
 * eliminating code duplication and providing thread-safety through SearchContext.
 * It maintains backward compatibility with the original AStar API.
 */
class AStar final {
public:
    /**
     * @brief Thread-safe A* search with external search context
     */
    template<typename State, typename Transition, typename StateIndexer,
             typename VertexIdentifier, typename HeuristicFunc>
    static Path<State> Search(
        const Graph<State, Transition, StateIndexer>* graph,
        SearchContext<State, Transition, StateIndexer>& context,
        VertexIdentifier start,
        VertexIdentifier goal,
        HeuristicFunc heuristic) {
        
        if (!graph) return Path<State>();
        
        auto start_it = graph->FindVertex(start);
        auto goal_it = graph->FindVertex(goal);
        
        if (start_it == graph->vertex_end() || goal_it == graph->vertex_end()) {
            return Path<State>();
        }
        
        auto strategy = MakeAStarStrategy<State, Transition, StateIndexer, HeuristicFunc>(
            std::move(heuristic));
        
        return SearchAlgorithm<decltype(strategy), State, Transition, StateIndexer>
            ::Search(graph, context, start_it, goal_it, strategy);
    }
    
    /**
     * @brief Convenience overload with shared_ptr graph
     */
    template<typename State, typename Transition, typename StateIndexer,
             typename VertexIdentifier, typename HeuristicFunc>
    static Path<State> Search(
        std::shared_ptr<Graph<State, Transition, StateIndexer>> graph,
        SearchContext<State, Transition, StateIndexer>& context,
        VertexIdentifier start,
        VertexIdentifier goal,
        HeuristicFunc heuristic) {
        
        return Search(graph.get(), context, start, goal, std::move(heuristic));
    }
    
    /**
     * @brief Legacy-compatible search that manages its own context (non-thread-safe)
     */
    template<typename State, typename Transition, typename StateIndexer,
             typename VertexIdentifier, typename HeuristicFunc>
    static Path<State> Search(
        const Graph<State, Transition, StateIndexer>* graph,
        VertexIdentifier start,
        VertexIdentifier goal,
        HeuristicFunc heuristic) {
        
        SearchContext<State, Transition, StateIndexer> context;
        return Search(graph, context, start, goal, std::move(heuristic));
    }
    
    /**
     * @brief Legacy-compatible search with shared_ptr (non-thread-safe)
     */
    template<typename State, typename Transition, typename StateIndexer,
             typename VertexIdentifier, typename HeuristicFunc>
    static Path<State> Search(
        std::shared_ptr<Graph<State, Transition, StateIndexer>> graph,
        VertexIdentifier start,
        VertexIdentifier goal,
        HeuristicFunc heuristic) {
        
        SearchContext<State, Transition, StateIndexer> context;
        return Search(graph.get(), context, start, goal, std::move(heuristic));
    }
};

// Compatibility typedefs for existing code
using AStarThreadSafe = AStar;
using AStarV2 = AStar;  // For code that already uses V2

} // namespace xmotion

#endif /* ASTAR_HPP */