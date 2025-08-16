/*
 * bfs.hpp
 *
 * Created on: Aug 2025
 * Description: Breadth-First Search algorithm using unified search framework
 *              Combined strategy implementation and public API
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef BFS_HPP
#define BFS_HPP

#include <limits>
#include "graph/search/search_algorithm.hpp"
#include "graph/search/search_strategy.hpp"

namespace xmotion {

/**
 * @brief Breadth-First Search strategy implementation
 * 
 * Implements BFS using a constant priority for all vertices, which effectively
 * makes the priority queue behave like a FIFO queue. BFS guarantees finding
 * the shortest path in terms of number of edges (unweighted graphs).
 */
template<typename State, typename Transition, typename StateIndexer, typename CostType = double>
class BfsStrategy : public SearchStrategy<BfsStrategy<State, Transition, StateIndexer, CostType>,
                                         State, Transition, StateIndexer, CostType> {
public:
    using Base = SearchStrategy<BfsStrategy<State, Transition, StateIndexer, CostType>,
                               State, Transition, StateIndexer, CostType>;
    using GraphType = typename Base::GraphType;
    using vertex_iterator = typename Base::vertex_iterator;
    using SearchInfo = typename Base::SearchInfo;
    
    BfsStrategy() = default;
    
    CostType GetPriorityImpl(const SearchInfo& info) const noexcept {
        return info.g_cost; // FIFO behavior
    }
    
    void InitializeVertexImpl(SearchInfo& info, vertex_iterator vertex, 
                             vertex_iterator goal_vertex) const {
        info.g_cost = 0.0;  // Start at depth 0
        info.h_cost = 0.0;  // BFS doesn't use heuristic
        info.f_cost = 0.0;  // Same as g_cost for BFS
        info.is_checked = false;
        info.is_in_openlist = false;
        info.parent_id = -1;
    }
    
    bool RelaxVertexImpl(SearchInfo& current_info, SearchInfo& successor_info,
                        vertex_iterator successor_vertex, vertex_iterator goal_vertex,
                        CostType edge_cost) const {
        // In BFS, we only process each vertex once (first visit)
        if (successor_info.g_cost == std::numeric_limits<CostType>::max()) {
            successor_info.g_cost = current_info.g_cost + CostType{1};  // Increase depth
            successor_info.h_cost = CostType{};  // No heuristic in BFS
            successor_info.f_cost = successor_info.g_cost;  // f = g for BFS
            return true;
        }
        return false; // Already visited
    }
    
    // Use default implementations for optional methods
    using Base::ProcessVertexImpl;
    using Base::IsGoalReachedImpl;
};

/**
 * @brief Helper function to create BFS strategy with automatic type deduction
 */
template<typename State, typename Transition, typename StateIndexer, typename CostType = double>
BfsStrategy<State, Transition, StateIndexer, CostType> MakeBfsStrategy() {
    return BfsStrategy<State, Transition, StateIndexer, CostType>();
}

/**
 * @brief Breadth-First Search algorithm - unified implementation
 * 
 * This class provides the public API for BFS searches using the strategy framework.
 * BFS finds the shortest path in terms of number of edges (unweighted shortest path).
 */
class BFS final {
public:
    /**
     * @brief Thread-safe BFS search with external search context
     */
    template<typename State, typename Transition, typename StateIndexer,
             typename VertexIdentifier, typename CostType = double>
    static Path<State> Search(
        const Graph<State, Transition, StateIndexer>* graph,
        SearchContext<State, Transition, StateIndexer, CostType>& context,
        VertexIdentifier start,
        VertexIdentifier goal) {
        
        if (!graph) return Path<State>();
        
        auto start_it = graph->FindVertex(start);
        auto goal_it = graph->FindVertex(goal);
        
        if (start_it == graph->vertex_end() || goal_it == graph->vertex_end()) {
            return Path<State>();
        }
        
        auto strategy = MakeBfsStrategy<State, Transition, StateIndexer, CostType>();
        return SearchAlgorithm<decltype(strategy), State, Transition, StateIndexer, CostType>
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
};

} // namespace xmotion

#endif /* BFS_HPP */