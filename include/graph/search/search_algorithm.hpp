/*
 * search_algorithm.hpp
 *
 * Created on: Aug 2025
 * Description: Unified template-based search algorithm framework
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 *
 * THREAD SAFETY:
 * - Graph must be read-only during search (const pointer accepted)
 * - Each concurrent search requires its own SearchContext
 * - Strategy objects are stateless and can be shared across threads
 * - See search_context.hpp for detailed thread safety model
 */

#ifndef SEARCH_ALGORITHM_HPP
#define SEARCH_ALGORITHM_HPP

#include <queue>
#include <vector>
#include <functional>
#include <stdexcept>

#include "graph/graph.hpp"
#include "graph/impl/dynamic_priority_queue.hpp"
#include "graph/search/search_context.hpp"
#include "graph/search/search_strategy.hpp"
#include "graph/exceptions.hpp"

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
 */
template<typename SearchStrategy, typename State, typename Transition, typename StateIndexer>
class SearchAlgorithm final {
public:
    using GraphType = Graph<State, Transition, StateIndexer>;
    using vertex_iterator = typename GraphType::const_vertex_iterator;
    using SearchContextType = SearchContext<State, Transition, StateIndexer>;
    using SearchInfo = typename SearchContextType::SearchVertexInfo;
    
    /**
     * @brief Priority queue comparator using search strategy (for std::priority_queue)
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
            // Use the strategy's cost comparator: if comp(a,b) means a < b,
            // then comp(b,a) means b < a, which gives us max-heap behavior for min-heap
            auto priority_x = strategy.GetPriority(info_x);
            auto priority_y = strategy.GetPriority(info_y);
            return strategy.GetComparator()(priority_y, priority_x);
        }
    };

    /**
     * @brief Comparator for DynamicPriorityQueue (min-heap, no reversal needed)
     *
     * DynamicPriorityQueue with std::less gives min-heap directly,
     * so we don't need to reverse the comparison.
     */
    struct DPQComparator {
        const SearchStrategy* strategy;
        const SearchContextType* context;

        DPQComparator() noexcept : strategy(nullptr), context(nullptr) {}
        DPQComparator(const SearchStrategy* s, const SearchContextType* c) noexcept
            : strategy(s), context(c) {}

        bool operator()(vertex_iterator x, vertex_iterator y) const {
            const auto& info_x = context->GetSearchInfo(x);
            const auto& info_y = context->GetSearchInfo(y);
            auto priority_x = strategy->GetPriority(info_x);
            auto priority_y = strategy->GetPriority(info_y);
            // Return true if x < y (min-heap: smaller priority = higher priority)
            return strategy->GetComparator()(priority_x, priority_y);
        }
    };

    /**
     * @brief Indexer for vertex iterators (extracts vertex_id for DynamicPriorityQueue)
     */
    struct VertexIteratorIndexer {
        int64_t operator()(vertex_iterator v) const {
            return v->vertex_id;
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
            throw InvalidArgumentError("Graph pointer cannot be null");
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

        // Track start vertex for reliable path reconstruction
        context.SetStartVertexId(start->vertex_id);

        // Priority queue with strategy-based comparison
        std::priority_queue<vertex_iterator, std::vector<vertex_iterator>,
                          VertexComparator> openlist(
            VertexComparator(strategy, context));

        // Initialize start vertex
        auto& start_info = context.GetSearchInfo(start);
        strategy.InitializeVertex(start_info, start, goal, context);
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
     * @brief Expand neighbors of current vertex (std::priority_queue version)
     *
     * Uses lazy deletion strategy: when a vertex is relaxed with a better
     * priority while already in the openlist, we push it again. Stale entries
     * are skipped when popped (checked in main loop via is_checked flag).
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
                                   successor, goal, edge.cost, context)) {
                successor_info.parent_id = current->vertex_id;

                // Always push when relaxed - use lazy deletion strategy
                // If vertex was already in openlist, the old entry becomes stale
                // and will be skipped when popped (is_checked will be true by then)
                openlist.push(successor);
                successor_info.is_in_openlist = true;
            }
        }
    }

    /**
     * @brief Expand neighbors using DynamicPriorityQueue for efficient updates
     *
     * DynamicPriorityQueue supports O(log n) priority updates, avoiding
     * duplicate entries and memory overhead of the lazy deletion approach.
     */
    using DPQType = DynamicPriorityQueue<vertex_iterator, DPQComparator, VertexIteratorIndexer>;

    static void ExpandNeighborsDPQ(
        const GraphType* graph,
        SearchContextType& context,
        vertex_iterator current,
        vertex_iterator goal,
        const SearchStrategy& strategy,
        DPQType& openlist) {

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
                                   successor, goal, edge.cost, context)) {
                successor_info.parent_id = current->vertex_id;

                // Push handles insert-or-update automatically
                openlist.Push(successor);
                successor_info.is_in_openlist = true;
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
        } catch (const ElementNotFoundError& e) {
            // Goal vertex not reached - return empty path
            return Path<State>();
        } catch (const std::exception& e) {
            // Other path reconstruction errors - return empty path
            return Path<State>();
        }
    }
};

} // namespace xmotion

#endif /* SEARCH_ALGORITHM_HPP */