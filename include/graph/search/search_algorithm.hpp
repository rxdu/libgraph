/*
 * search_algorithm.hpp
 *
 * Created on: Aug 2025
 * Description: Unified template-based search algorithm framework
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef SEARCH_ALGORITHM_HPP
#define SEARCH_ALGORITHM_HPP

#include <queue>
#include <vector>
#include <functional>
#include <stdexcept>

#include "graph/graph.hpp"
#include "graph/search/search_context.hpp"
#include "graph/search/search_strategy.hpp"

namespace xmotion {

/**
 * @brief Unified search algorithm template using strategy pattern
 * 
 * This template implements the common search algorithm structure that can be
 * specialized for different search strategies (A*, Dijkstra, BFS, etc.).
 * It eliminates code duplication while maintaining performance and type safety.
 * 
 * @tparam SearchStrategy The concrete search strategy implementation
 * @tparam State The state type used in the graph
 * @tparam Transition The transition/cost type used in edges
 * @tparam StateIndexer The indexer functor for state types
 * @tparam CostType The numeric type used for costs (defaults to double)
 */
template<typename SearchStrategy, typename State, typename Transition, typename StateIndexer, typename CostType = double>
class SearchAlgorithm final {
public:
    using GraphType = Graph<State, Transition, StateIndexer>;
    using vertex_iterator = typename GraphType::const_vertex_iterator;
    using SearchContextType = SearchContext<State, Transition, StateIndexer, CostType>;
    using SearchInfo = typename SearchContextType::SearchVertexInfo;
    
    /**
     * @brief Priority queue comparator using search strategy
     * 
     * Uses the strategy's GetPriority method to compare vertices.
     * Implements min-heap behavior (lower priority values come first).
     */
    struct VertexComparator {
        const SearchStrategy& strategy;
        const SearchContextType& context;
        
        VertexComparator(const SearchStrategy& s, const SearchContextType& c) noexcept
            : strategy(s), context(c) {}
            
        bool operator()(vertex_iterator x, vertex_iterator y) const {
            const auto& info_x = context.GetSearchInfo(x);
            const auto& info_y = context.GetSearchInfo(y);
            // Note: priority_queue is max-heap, so we reverse comparison for min-heap
            return strategy.GetPriority(info_x) > strategy.GetPriority(info_y);
        }
    };
    
    /**
     * @brief Perform search using the provided strategy
     * 
     * @param graph Const pointer to the graph (read-only access)
     * @param context Reference to search context for this search
     * @param start Starting vertex iterator
     * @param goal Goal vertex iterator
     * @param strategy Search strategy implementation
     * @return Vector of states representing the path, empty if no path found
     */
    static Path<State> Search(
        const GraphType* graph,
        SearchContextType& context,
        vertex_iterator start,
        vertex_iterator goal,
        const SearchStrategy& strategy) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        if (start == graph->vertex_end()) {
            return Path<State>();
        }
        
        // Allow goal to be vertex_end() for complete traversals like DFS::TraverseAll
        
        return PerformSearch(graph, context, start, goal, strategy);
    }
    
private:
    /**
     * @brief Main search algorithm implementation
     */
    static Path<State> PerformSearch(
        const GraphType* graph,
        SearchContextType& context, 
        vertex_iterator start,
        vertex_iterator goal,
        const SearchStrategy& strategy) {
        
        // Clear previous search data but preserve allocated memory
        context.Reset();
        
        // Priority queue with strategy-based comparison
        std::priority_queue<vertex_iterator, std::vector<vertex_iterator>, 
                          VertexComparator> openlist(
            VertexComparator(strategy, context));
        
        // Initialize start vertex
        auto& start_info = context.GetSearchInfo(start);
        strategy.InitializeVertex(start_info, start, goal);
        openlist.push(start);
        start_info.is_in_openlist = true;
        
        // Main search loop
        while (!openlist.empty()) {
            vertex_iterator current = openlist.top();
            openlist.pop();
            
            auto& current_info = context.GetSearchInfo(current);
            current_info.is_in_openlist = false;
            current_info.is_checked = true;
            
            // Process current vertex (algorithm-specific hook)
            strategy.ProcessVertex(current_info, current);
            
            // Check termination condition
            if (strategy.IsGoalReached(current, goal)) {
                return ReconstructPath(graph, context, start->vertex_id, goal->vertex_id);
            }
            
            // Expand neighbors
            ExpandNeighbors(graph, context, current, goal, strategy, openlist);
        }
        
        // No path found
        return Path<State>();
    }
    
    /**
     * @brief Expand neighbors of current vertex
     */
    static void ExpandNeighbors(
        const GraphType* graph,
        SearchContextType& context,
        vertex_iterator current,
        vertex_iterator goal,
        const SearchStrategy& strategy,
        std::priority_queue<vertex_iterator, std::vector<vertex_iterator>, 
                          VertexComparator>& openlist) {
        
        auto& current_info = context.GetSearchInfo(current);
        
        for (const auto& edge : current->edges_to) {
            vertex_iterator successor = edge.dst;
            auto& successor_info = context.GetSearchInfo(successor);
            
            // Skip if already processed
            if (successor_info.is_checked) {
                continue;
            }
            
            // Attempt to relax the vertex
            if (strategy.RelaxVertex(current_info, successor_info, 
                                   successor, goal, edge.cost)) {
                successor_info.parent_id = current->vertex_id;
                
                // Add to open list if not already present
                if (!successor_info.is_in_openlist) {
                    openlist.push(successor);
                    successor_info.is_in_openlist = true;
                }
                // Note: If already in openlist, the priority queue will naturally
                // handle the updated priority on next pop operation
            }
        }
    }
    
    /**
     * @brief Reconstruct path from search results
     */
    static Path<State> ReconstructPath(
        const GraphType* graph,
        const SearchContextType& context,
        int64_t start_id,
        int64_t goal_id) {
        
        try {
            return context.template ReconstructPath<State>(graph, goal_id);
        } catch (const std::exception& e) {
            // Path reconstruction failed - return empty path
            return Path<State>();
        }
    }
};

} // namespace xmotion

#endif /* SEARCH_ALGORITHM_HPP */