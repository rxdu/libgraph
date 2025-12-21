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
#include <chrono>
#include <unordered_set>

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
     *
     * DETERMINISTIC TIE-BREAKING: When two vertices have equal priority,
     * ties are broken by vertex_id to ensure reproducible search results
     * regardless of hash map ordering or iteration order.
     */
    struct VertexComparator {
        const SearchStrategy& strategy;
        const SearchContextType& context;

        VertexComparator(const SearchStrategy& s, const SearchContextType& c) noexcept
            : strategy(s), context(c) {}

        bool operator()(vertex_iterator x, vertex_iterator y) const {
            const auto& info_x = context.GetSearchInfo(x);
            const auto& info_y = context.GetSearchInfo(y);
            auto priority_x = strategy.GetPriority(info_x);
            auto priority_y = strategy.GetPriority(info_y);

            // Note: priority_queue is max-heap, so we reverse comparison for min-heap
            // Use the strategy's cost comparator: if comp(a,b) means a < b,
            // then comp(b,a) means b < a, which gives us max-heap behavior for min-heap
            const auto& comp = strategy.GetComparator();

            if (comp(priority_y, priority_x)) {
                return true;  // x has higher priority (lower cost) than y
            }
            if (comp(priority_x, priority_y)) {
                return false;  // y has higher priority (lower cost) than x
            }
            // Tie-breaking: when priorities are equal, use vertex_id for determinism
            // Lower vertex_id gets higher priority (comes first)
            return x->vertex_id > y->vertex_id;
        }
    };

    /**
     * @brief Comparator for DynamicPriorityQueue (min-heap, no reversal needed)
     *
     * DynamicPriorityQueue with std::less gives min-heap directly,
     * so we don't need to reverse the comparison.
     *
     * DETERMINISTIC TIE-BREAKING: When two vertices have equal priority,
     * ties are broken by vertex_id to ensure reproducible search results
     * regardless of hash map ordering or iteration order.
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

            const auto& comp = strategy->GetComparator();

            // Return true if x < y (min-heap: smaller priority = higher priority)
            if (comp(priority_x, priority_y)) {
                return true;  // x has higher priority (lower cost) than y
            }
            if (comp(priority_y, priority_x)) {
                return false;  // y has higher priority (lower cost) than x
            }
            // Tie-breaking: when priorities are equal, use vertex_id for determinism
            // Lower vertex_id gets higher priority (comes first)
            return x->vertex_id < y->vertex_id;
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

    /**
     * @brief Search with rich result including cost and diagnostics
     *
     * Returns a PathResult containing the path, total cost, and number of
     * nodes expanded. Useful for comparing paths or tuning heuristics.
     *
     * @param graph Const pointer to the graph
     * @param context Reference to search context
     * @param start Starting vertex iterator
     * @param goal Goal vertex iterator
     * @param strategy Search strategy implementation
     * @return PathResult with path, cost, and diagnostics
     */
    static PathResult<State, Transition> SearchWithResult(
        const GraphType* graph,
        SearchContextType& context,
        vertex_iterator start,
        vertex_iterator goal,
        const SearchStrategy& strategy) {

        PathResult<State, Transition> result;

        if (!graph || start == graph->vertex_end()) {
            return result;
        }

        result.path = PerformSearch(graph, context, start, goal, strategy);
        result.nodes_expanded = context.GetNodesExpanded();
        result.found = !result.path.empty();

        // Extract total cost from goal vertex if path was found
        if (result.found && goal != graph->vertex_end()) {
            const auto& goal_info = context.GetSearchInfo(goal);
            result.total_cost = goal_info.template GetGCost<Transition>();
        }

        return result;
    }

    /**
     * @brief Search with termination limits for real-time systems
     *
     * Allows bounding search by maximum expansions or timeout.
     * When a limit is reached, returns the best path found so far (if any).
     *
     * @param graph Const pointer to the graph
     * @param context Reference to search context
     * @param start Starting vertex iterator
     * @param goal Goal vertex iterator
     * @param strategy Search strategy implementation
     * @param limits Termination limits (max expansions, timeout)
     * @return PathResult with path, cost, and diagnostics
     */
    static PathResult<State, Transition> SearchWithLimits(
        const GraphType* graph,
        SearchContextType& context,
        vertex_iterator start,
        vertex_iterator goal,
        const SearchStrategy& strategy,
        const SearchLimits& limits) {

        PathResult<State, Transition> result;

        if (!graph || start == graph->vertex_end()) {
            return result;
        }

        if (!limits.HasLimits()) {
            // No limits, use standard search
            return SearchWithResult(graph, context, start, goal, strategy);
        }

        result.path = PerformSearchWithLimits(graph, context, start, goal, strategy, limits);
        result.nodes_expanded = context.GetNodesExpanded();
        result.found = !result.path.empty();

        if (result.found && goal != graph->vertex_end()) {
            const auto& goal_info = context.GetSearchInfo(goal);
            result.total_cost = goal_info.template GetGCost<Transition>();
        }

        return result;
    }

    /**
     * @brief Search to the nearest of multiple goals
     *
     * Finds the shortest path from start to any of the provided goals.
     * Returns the path to the first goal reached (which is optimal in
     * terms of path cost for algorithms like Dijkstra and A*).
     *
     * @param graph Const pointer to the graph
     * @param context Reference to search context
     * @param start Starting vertex iterator
     * @param goals Vector of goal vertex iterators
     * @param strategy Search strategy implementation
     * @return MultiGoalResult with path, cost, and reached goal info
     */
    static MultiGoalResult<State, Transition> SearchMultiGoal(
        const GraphType* graph,
        SearchContextType& context,
        vertex_iterator start,
        const std::vector<vertex_iterator>& goals,
        const SearchStrategy& strategy) {

        MultiGoalResult<State, Transition> result;

        if (!graph || start == graph->vertex_end() || goals.empty()) {
            return result;
        }

        // Build goal set for O(1) lookup
        std::unordered_set<int64_t> goal_ids;
        for (size_t i = 0; i < goals.size(); ++i) {
            if (goals[i] != graph->vertex_end()) {
                goal_ids.insert(goals[i]->vertex_id);
            }
        }

        if (goal_ids.empty()) {
            return result;
        }

        // Perform search with multi-goal termination
        auto reached = PerformMultiGoalSearch(graph, context, start, goal_ids, strategy);
        result.nodes_expanded = context.GetNodesExpanded();

        if (reached != graph->vertex_end()) {
            result.found = true;
            result.goal_vertex_id = reached->vertex_id;
            result.path = ReconstructPath(graph, context, start->vertex_id, reached->vertex_id);

            const auto& goal_info = context.GetSearchInfo(reached);
            result.total_cost = goal_info.template GetGCost<Transition>();

            // Find the index in the original goals vector
            for (size_t i = 0; i < goals.size(); ++i) {
                if (goals[i] != graph->vertex_end() &&
                    goals[i]->vertex_id == reached->vertex_id) {
                    result.goal_index = i;
                    break;
                }
            }
        }

        return result;
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

        // Single-goal search mode
        context.SetMultiGoalSearch(false);

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

            // Track node expansion for diagnostics
            context.IncrementNodesExpanded();

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
     * @brief Search with termination limits
     *
     * Stops search when max_expansions or timeout is reached.
     */
    static Path<State> PerformSearchWithLimits(
        const GraphType* graph,
        SearchContextType& context,
        vertex_iterator start,
        vertex_iterator goal,
        const SearchStrategy& strategy,
        const SearchLimits& limits) {

        // Clear previous search data but preserve allocated memory
        context.Reset();

        // Track start vertex for reliable path reconstruction
        context.SetStartVertexId(start->vertex_id);

        // Set up timing if timeout is specified
        using Clock = std::chrono::steady_clock;
        auto start_time = Clock::now();
        auto timeout_duration = std::chrono::milliseconds(limits.timeout_ms);

        // Priority queue with strategy-based comparison
        std::priority_queue<vertex_iterator, std::vector<vertex_iterator>,
                          VertexComparator> openlist(
            VertexComparator(strategy, context));

        // Initialize start vertex
        auto& start_info = context.GetSearchInfo(start);
        strategy.InitializeVertex(start_info, start, goal, context);
        openlist.push(start);
        start_info.is_in_openlist = true;

        // Main search loop with limit checking
        while (!openlist.empty()) {
            // Check expansion limit
            if (limits.max_expansions > 0 &&
                context.GetNodesExpanded() >= limits.max_expansions) {
                break;  // Limit reached
            }

            // Check timeout (less frequently to reduce overhead)
            if (limits.timeout_ms > 0 &&
                (context.GetNodesExpanded() % 100 == 0)) {  // Check every 100 expansions
                auto elapsed = Clock::now() - start_time;
                if (elapsed >= timeout_duration) {
                    break;  // Timeout reached
                }
            }

            vertex_iterator current = openlist.top();
            openlist.pop();

            auto& current_info = context.GetSearchInfo(current);
            current_info.is_in_openlist = false;
            current_info.is_checked = true;

            // Track node expansion for diagnostics
            context.IncrementNodesExpanded();

            // Process current vertex (algorithm-specific hook)
            strategy.ProcessVertex(current_info, current);

            // Check termination condition
            if (strategy.IsGoalReached(current, goal)) {
                return ReconstructPath(graph, context, start->vertex_id, goal->vertex_id);
            }

            // Expand neighbors
            ExpandNeighbors(graph, context, current, goal, strategy, openlist);
        }

        // No path found (or limit reached before finding path)
        return Path<State>();
    }

    /**
     * @brief Multi-goal search implementation
     *
     * Expands from start until any goal in goal_ids is reached.
     * Returns the iterator to the reached goal, or vertex_end() if none found.
     */
    static vertex_iterator PerformMultiGoalSearch(
        const GraphType* graph,
        SearchContextType& context,
        vertex_iterator start,
        const std::unordered_set<int64_t>& goal_ids,
        const SearchStrategy& strategy) {

        // Clear previous search data but preserve allocated memory
        context.Reset();

        // Multi-goal search mode - strategies should not dereference goal_vertex
        context.SetMultiGoalSearch(true);

        // Track start vertex for reliable path reconstruction
        context.SetStartVertexId(start->vertex_id);

        // Use vertex_end() as dummy goal for initialization (no heuristic target)
        vertex_iterator dummy_goal = graph->vertex_end();

        // Priority queue with strategy-based comparison
        std::priority_queue<vertex_iterator, std::vector<vertex_iterator>,
                          VertexComparator> openlist(
            VertexComparator(strategy, context));

        // Initialize start vertex
        auto& start_info = context.GetSearchInfo(start);
        strategy.InitializeVertex(start_info, start, dummy_goal, context);
        openlist.push(start);
        start_info.is_in_openlist = true;

        // Check if start is already a goal
        if (goal_ids.count(start->vertex_id) > 0) {
            context.IncrementNodesExpanded();
            return start;
        }

        // Main search loop
        while (!openlist.empty()) {
            vertex_iterator current = openlist.top();
            openlist.pop();

            auto& current_info = context.GetSearchInfo(current);
            current_info.is_in_openlist = false;
            current_info.is_checked = true;

            // Track node expansion for diagnostics
            context.IncrementNodesExpanded();

            // Process current vertex (algorithm-specific hook)
            strategy.ProcessVertex(current_info, current);

            // Check if this is any of the goals
            if (goal_ids.count(current->vertex_id) > 0) {
                return current;
            }

            // Expand neighbors (using dummy_goal since we have multiple goals)
            ExpandNeighbors(graph, context, current, dummy_goal, strategy, openlist);
        }

        // No goal found
        return graph->vertex_end();
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
            // edge.dst is now Vertex* (stable pointer), convert to iterator
            vertex_iterator successor = graph->FindVertex(edge.dst->vertex_id);
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
            // edge.dst is now Vertex* (stable pointer), convert to iterator
            vertex_iterator successor = graph->FindVertex(edge.dst->vertex_id);
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