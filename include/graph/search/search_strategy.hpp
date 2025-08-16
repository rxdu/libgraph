/*
 * search_strategy.hpp
 *
 * Created on: Aug 2025
 * Description: Template-based search strategy interface for unified algorithm framework
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef SEARCH_STRATEGY_HPP
#define SEARCH_STRATEGY_HPP

#include <limits>
#include "graph/graph.hpp"
#include "graph/search/search_context.hpp"

namespace xmotion {

/**
 * @brief Abstract strategy interface for search algorithms
 * 
 * This class defines the interface that concrete search strategies must implement
 * to work with the unified SearchAlgorithm template. It uses CRTP (Curiously 
 * Recurring Template Pattern) to avoid virtual function overhead.
 * 
 * @tparam Derived The concrete strategy implementation
 * @tparam State The state type used in the graph
 * @tparam Transition The transition/cost type used in edges
 * @tparam StateIndexer The indexer functor for state types
 */
template<typename Derived, typename State, typename Transition, typename StateIndexer>
class SearchStrategy {
public:
    using GraphType = Graph<State, Transition, StateIndexer>;
    using vertex_iterator = typename GraphType::const_vertex_iterator;
    using SearchInfo = typename SearchContext<State, Transition, StateIndexer>::SearchVertexInfo;
    using CostType = double; // TODO: Make this configurable in future versions
    
    /**
     * @brief Calculate priority for vertex in open list
     * @param info Search information for the vertex
     * @return Priority value (lower values have higher priority in min-heap)
     */
    inline CostType GetPriority(const SearchInfo& info) const noexcept {
        return static_cast<const Derived*>(this)->GetPriorityImpl(info);
    }
    
    /**
     * @brief Initialize vertex when first encountered in search
     * @param info Search information to initialize
     * @param vertex The vertex being initialized
     * @param goal_vertex The goal vertex (for heuristic calculation)
     */
    inline void InitializeVertex(SearchInfo& info, vertex_iterator vertex, 
                                vertex_iterator goal_vertex) const {
        static_cast<const Derived*>(this)->InitializeVertexImpl(info, vertex, goal_vertex);
    }
    
    /**
     * @brief Process vertex when it's expanded from open list (optional hook)
     * @param info Search information for current vertex
     * @param vertex The vertex being processed
     */
    inline void ProcessVertex(SearchInfo& info, vertex_iterator vertex) const {
        static_cast<const Derived*>(this)->ProcessVertexImpl(info, vertex);
    }
    
    /**
     * @brief Check if search should terminate at this vertex
     * @param current Current vertex being examined
     * @param goal Goal vertex
     * @return True if goal is reached
     */
    inline bool IsGoalReached(vertex_iterator current, vertex_iterator goal) const noexcept {
        return static_cast<const Derived*>(this)->IsGoalReachedImpl(current, goal);
    }
    
    /**
     * @brief Update vertex costs during edge relaxation
     * @param current_info Search info for current vertex
     * @param successor_info Search info for successor vertex
     * @param successor_vertex The successor vertex iterator
     * @param goal_vertex The goal vertex (for heuristic calculation)
     * @param edge_cost Cost of the edge from current to successor
     * @return True if successor was relaxed (costs improved)
     */
    inline bool RelaxVertex(SearchInfo& current_info, SearchInfo& successor_info,
                           vertex_iterator successor_vertex, vertex_iterator goal_vertex,
                           CostType edge_cost) const {
        return static_cast<const Derived*>(this)->RelaxVertexImpl(
            current_info, successor_info, successor_vertex, goal_vertex, edge_cost);
    }

protected:
    // Default implementations for optional methods
    void ProcessVertexImpl(SearchInfo& info, vertex_iterator vertex) const {
        // Default: no special processing
    }
    
    bool IsGoalReachedImpl(vertex_iterator current, vertex_iterator goal) const noexcept {
        return current == goal;
    }
};

} // namespace xmotion

#endif /* SEARCH_STRATEGY_HPP */