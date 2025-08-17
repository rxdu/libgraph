/*
 * dfs.hpp
 *
 * Created on: Aug 2025
 * Description: Depth-First Search algorithm using unified search framework
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef DFS_HPP
#define DFS_HPP

#include <limits>
#include "graph/search/search_algorithm.hpp"
#include "graph/search/search_strategy.hpp"

namespace xmotion {

/**
 * @brief Depth-First Search strategy implementation
 * 
 * Implements DFS using a timestamp-based priority system to achieve LIFO behavior.
 * DFS explores as far as possible along each branch before backtracking, making it
 * useful for cycle detection, topological sorting, and connectivity analysis.
 * 
 * The strategy uses negative timestamps as priorities to make the priority queue
 * behave like a stack (most recently added vertices get processed first).
 */
template<typename State, typename Transition, typename StateIndexer>
class DfsStrategy : public SearchStrategy<DfsStrategy<State, Transition, StateIndexer>,
                                         State, Transition, StateIndexer> {
public:
    using Base = SearchStrategy<DfsStrategy<State, Transition, StateIndexer>,
                               State, Transition, StateIndexer>;
    using GraphType = typename Base::GraphType;
    using vertex_iterator = typename Base::vertex_iterator;
    using SearchInfo = typename Base::SearchInfo;
    
private:
    mutable int64_t timestamp_counter_{0};
    
public:
    DfsStrategy() = default;
    
    /**
     * @brief Get priority for DFS (implements LIFO using negative timestamps)
     * 
     * Uses negative timestamps to ensure that vertices added more recently
     * get higher priority in the min-heap, creating LIFO behavior.
     */
    double GetPriorityImpl(const SearchInfo& info) const noexcept {
        // Use negative g_cost (which stores timestamp) for LIFO behavior
        // More recent timestamps (higher values) become lower priorities (more negative)
        return -info.g_cost;
    }
    
    /**
     * @brief Initialize vertex for DFS
     * 
     * Sets the timestamp for this vertex to enable LIFO ordering.
     */
    void InitializeVertexImpl(SearchInfo& info, vertex_iterator vertex, 
                             vertex_iterator goal_vertex) const {
        // Use timestamp as g_cost to achieve LIFO behavior
        info.g_cost = static_cast<double>(++timestamp_counter_);
        info.h_cost = 0.0;  // DFS doesn't use heuristic
        info.f_cost = info.g_cost;  // f = g for DFS
        info.is_checked = false;
        info.is_in_openlist = false;
        info.parent_id = -1;
    }
    
    /**
     * @brief Relax vertex for DFS
     * 
     * In DFS, we typically visit each vertex only once (first encounter).
     * This implements the standard DFS behavior where we don't revisit vertices.
     */
    bool RelaxVertexImpl(SearchInfo& current_info, SearchInfo& successor_info,
                        vertex_iterator successor_vertex, vertex_iterator goal_vertex,
                        double edge_cost) const {
        // In DFS, we only process each vertex once (first visit)
        if (successor_info.g_cost == std::numeric_limits<double>::max()) {
            // Assign new timestamp for LIFO ordering
            successor_info.g_cost = static_cast<double>(++timestamp_counter_);
            successor_info.h_cost = 0.0;  // No heuristic in DFS
            successor_info.f_cost = successor_info.g_cost;  // f = g for DFS
            return true;
        }
        return false; // Already visited
    }
    
    // Use default implementations for optional methods
    using Base::ProcessVertexImpl;
    using Base::IsGoalReachedImpl;
};

/**
 * @brief Helper function to create DFS strategy with automatic type deduction
 */
template<typename State, typename Transition, typename StateIndexer>
DfsStrategy<State, Transition, StateIndexer> MakeDfsStrategy() {
    return DfsStrategy<State, Transition, StateIndexer>();
}

/**
 * @brief Depth-First Search algorithm - unified implementation
 * 
 * This class provides the public API for DFS searches using the strategy framework.
 * DFS is particularly useful for:
 * - Cycle detection in graphs
 * - Topological sorting of DAGs
 * - Connected components analysis
 * - Path finding (though not optimal)
 */
class DFS final {
public:
    /**
     * @brief Thread-safe DFS search with external search context
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
        
        auto strategy = MakeDfsStrategy<State, Transition, StateIndexer>();
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
     * @brief DFS traversal from start to all reachable vertices
     * 
     * Performs a complete DFS traversal starting from the given vertex.
     * Useful for connectivity analysis and cycle detection.
     */
    template<typename State, typename Transition, typename StateIndexer,
             typename VertexIdentifier>
    static bool TraverseAll(
        const Graph<State, Transition, StateIndexer>* graph,
        SearchContext<State, Transition, StateIndexer>& context,
        VertexIdentifier start) {
        
        if (!graph) return false;
        
        auto start_it = graph->FindVertex(start);
        if (start_it == graph->vertex_end()) return false;
        
        auto strategy = MakeDfsStrategy<State, Transition, StateIndexer>();
        auto dummy_goal = graph->vertex_end();
        
        SearchAlgorithm<decltype(strategy), State, Transition, StateIndexer>
            ::Search(graph, context, start_it, dummy_goal, strategy);
        
        return true;
    }
    
    /**
     * @brief Check if there's a path from start to goal using DFS
     * 
     * Returns true if goal is reachable from start, false otherwise.
     * More efficient than full path reconstruction when you only need connectivity.
     */
    template<typename State, typename Transition, typename StateIndexer,
             typename VertexIdentifier>
    static bool IsReachable(
        const Graph<State, Transition, StateIndexer>* graph,
        VertexIdentifier start,
        VertexIdentifier goal) {
        
        SearchContext<State, Transition, StateIndexer> context;
        auto path = Search(graph, context, start, goal);
        return !path.empty();
    }
};

// Compatibility typedefs for existing code
using DepthFirstSearch = DFS;

} // namespace xmotion

#endif /* DFS_HPP */