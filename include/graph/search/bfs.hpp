/*
 * bfs.hpp
 *
 * Created on: Aug 2025
 * Description: Breadth-First Search algorithm using unified search framework
 *              Combined strategy implementation and public API
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 *
 * THREAD SAFETY:
 * - BfsStrategy is stateless and can be shared across threads
 * - For concurrent searches, each thread must use its own SearchContext
 * - Graph must be read-only during all concurrent searches
 * - Use the overloads that accept SearchContext& for thread-safe operation
 * - Legacy overloads without SearchContext are NOT thread-safe
 */

#ifndef BFS_HPP
#define BFS_HPP

#include <limits>
#include <queue>
#include <unordered_set>
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
template<typename State, typename Transition, typename StateIndexer,
         typename TransitionComparator = std::less<Transition>>
class BfsStrategy : public SearchStrategy<BfsStrategy<State, Transition, StateIndexer, TransitionComparator>,
                                         State, Transition, StateIndexer, TransitionComparator> {
public:
    using Base = SearchStrategy<BfsStrategy<State, Transition, StateIndexer, TransitionComparator>,
                               State, Transition, StateIndexer, TransitionComparator>;
    using GraphType = typename Base::GraphType;
    using vertex_iterator = typename Base::vertex_iterator;
    using SearchInfo = typename Base::SearchInfo;
    
    BfsStrategy() = default;
    explicit BfsStrategy(const TransitionComparator& comp) : Base(comp) {}

    Transition GetPriorityImpl(const SearchInfo& info) const noexcept {
        return info.template GetGCost<Transition>(); // FIFO behavior based on depth
    }

    void InitializeVertexImpl(SearchInfo& info, vertex_iterator vertex,
                             vertex_iterator goal_vertex,
                             typename Base::SearchContextType& /*context*/) const {
        info.SetGCost(Transition{});  // Start at depth 0
        info.SetHCost(Transition{});  // BFS doesn't use heuristic
        info.SetFCost(Transition{});  // Same as g_cost for BFS
        info.SetChecked(false);
        info.SetInOpenList(false);
        info.SetParent(-1);
    }

    bool RelaxVertexImpl(SearchInfo& current_info, SearchInfo& successor_info,
                        vertex_iterator successor_vertex, vertex_iterator goal_vertex,
                        const Transition& edge_cost,
                        typename Base::SearchContextType& /*context*/) const {
        // In BFS, we only process each vertex once (first visit)
        Transition successor_g_cost = successor_info.template GetGCost<Transition>();
        Transition max_cost = CostTraits<Transition>::infinity();
        
        // Check if vertex hasn't been visited yet
        if (successor_g_cost == max_cost || this->cost_comparator_(max_cost, successor_g_cost)) {
            Transition current_g_cost = current_info.template GetGCost<Transition>();
            Transition one_step = edge_cost; // In BFS, each step has unit cost
            if (std::is_arithmetic<Transition>::value) {
                one_step = Transition{1}; // Use 1 for arithmetic types to count steps
            }
            successor_info.SetGCost(current_g_cost + one_step);  // Increase depth
            successor_info.SetHCost(Transition{});  // No heuristic in BFS
            successor_info.SetFCost(current_g_cost + one_step);  // f = g for BFS
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
template<typename State, typename Transition, typename StateIndexer,
         typename TransitionComparator = std::less<Transition>>
BfsStrategy<State, Transition, StateIndexer, TransitionComparator> 
MakeBfsStrategy(const TransitionComparator& comp = TransitionComparator{}) {
    return BfsStrategy<State, Transition, StateIndexer, TransitionComparator>(comp);
}

/**
 * @brief Breadth-First Search algorithm - unified implementation
 *
 * This class provides the public API for BFS searches.
 * BFS finds the shortest path in terms of number of edges (unweighted shortest path).
 *
 * Implementation uses std::queue for true O(1) FIFO operations, avoiding the
 * O(log n) overhead of priority_queue-based approaches.
 */
class BFS final {
private:
    /**
     * @brief Internal queue-based BFS implementation
     *
     * Uses std::queue for O(1) enqueue/dequeue operations.
     * This is more efficient than the priority_queue approach and works
     * correctly with any Transition type (not just arithmetic).
     */
    template<typename State, typename Transition, typename StateIndexer>
    static Path<State> QueueBasedSearch(
        const Graph<State, Transition, StateIndexer>* graph,
        SearchContext<State, Transition, StateIndexer>& context,
        typename Graph<State, Transition, StateIndexer>::const_vertex_iterator start_it,
        typename Graph<State, Transition, StateIndexer>::const_vertex_iterator goal_it) {

        using vertex_iterator = typename Graph<State, Transition, StateIndexer>::const_vertex_iterator;

        // Reset context for fresh search
        context.Reset();

        // Track start vertex for reliable path reconstruction
        context.SetStartVertexId(start_it->vertex_id);

        // Check if start == goal
        if (start_it == goal_it) {
            return Path<State>{start_it->state};
        }

        // BFS queue - O(1) operations
        std::queue<vertex_iterator> queue;

        // Initialize start vertex
        auto& start_info = context.GetSearchInfo(start_it);
        start_info.SetChecked(true);
        start_info.SetParent(-1);
        start_info.SetGCost(Transition{});  // Depth 0

        queue.push(start_it);

        // BFS main loop
        while (!queue.empty()) {
            vertex_iterator current = queue.front();
            queue.pop();

            // Explore neighbors
            for (const auto& edge : current->edges_to) {
                vertex_iterator neighbor = edge.dst;
                auto& neighbor_info = context.GetSearchInfo(neighbor);

                // Skip if already visited
                if (neighbor_info.GetChecked()) {
                    continue;
                }

                // Mark as visited and set parent
                neighbor_info.SetChecked(true);
                neighbor_info.SetParent(current->vertex_id);

                // Track depth (optional, for debugging/metrics)
                auto& current_info = context.GetSearchInfo(current);
                Transition current_depth = current_info.template GetGCost<Transition>();
                if (std::is_arithmetic<Transition>::value) {
                    neighbor_info.SetGCost(current_depth + Transition{1});
                }

                // Check if goal reached
                if (neighbor == goal_it) {
                    return ReconstructPath<State, Transition, StateIndexer>(
                        graph, context, start_it->vertex_id, goal_it->vertex_id);
                }

                queue.push(neighbor);
            }
        }

        // No path found
        return Path<State>();
    }

    /**
     * @brief Reconstruct path from BFS search results
     */
    template<typename State, typename Transition, typename StateIndexer>
    static Path<State> ReconstructPath(
        const Graph<State, Transition, StateIndexer>* graph,
        const SearchContext<State, Transition, StateIndexer>& context,
        int64_t start_id,
        int64_t goal_id) {

        Path<State> path;
        std::vector<int64_t> vertex_ids;

        // Trace back from goal to start
        int64_t current_id = goal_id;
        while (current_id != -1) {
            vertex_ids.push_back(current_id);
            if (current_id == start_id) break;

            if (!context.HasSearchInfo(current_id)) break;
            const auto& info = context.GetSearchInfo(current_id);
            current_id = info.GetParent();
        }

        // Build path in forward order
        path.reserve(vertex_ids.size());
        for (auto it = vertex_ids.rbegin(); it != vertex_ids.rend(); ++it) {
            auto vertex_it = graph->FindVertex(*it);
            if (vertex_it != graph->vertex_end()) {
                path.push_back(vertex_it->state);
            }
        }

        return path;
    }

public:
    /**
     * @brief Thread-safe BFS search with external search context
     *
     * Uses queue-based BFS for O(V+E) time complexity with O(1) per-operation cost.
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

        return QueueBasedSearch<State, Transition, StateIndexer>(
            graph, context, start_it, goal_it);
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
     * @brief BFS traversal of all reachable vertices from start
     * 
     * Performs breadth-first traversal starting from the given vertex,
     * visiting all vertices reachable from the start vertex.
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
        
        // Manual BFS traversal
        using VertexIteratorType = decltype(start_it);
        std::queue<VertexIteratorType> q;
        std::unordered_set<int64_t> visited;
        
        q.push(start_it);
        visited.insert(start_it->vertex_id);
        
        while (!q.empty()) {
            auto current = q.front();
            q.pop();
            
            // Mark as visited in context
            auto& current_info = context.GetSearchInfo(current);
            current_info.SetChecked(true);
            
            for (const auto& edge : current->edges_to) {
                auto neighbor = edge.dst;
                int64_t neighbor_id = neighbor->vertex_id;
                
                if (visited.find(neighbor_id) == visited.end()) {
                    q.push(neighbor);
                    visited.insert(neighbor_id);
                }
            }
        }
        
        return true;
    }
    
    /**
     * @brief Convenience overload for TraverseAll with shared_ptr
     */
    template<typename State, typename Transition, typename StateIndexer,
             typename VertexIdentifier>
    static bool TraverseAll(
        std::shared_ptr<Graph<State, Transition, StateIndexer>> graph,
        SearchContext<State, Transition, StateIndexer>& context,
        VertexIdentifier start) {
        
        return TraverseAll(graph.get(), context, start);
    }
    
    /**
     * @brief Check if target vertex is reachable from start vertex
     * 
     * Uses BFS to determine if there exists a path from start to target.
     * This is more efficient than a full search when only reachability
     * information is needed.
     */
    template<typename State, typename Transition, typename StateIndexer,
             typename VertexIdentifier>
    static bool IsReachable(
        const Graph<State, Transition, StateIndexer>* graph,
        VertexIdentifier start,
        VertexIdentifier target) {
        
        if (!graph) return false;
        
        auto start_it = graph->FindVertex(start);
        auto target_it = graph->FindVertex(target);
        
        if (start_it == graph->vertex_end() || target_it == graph->vertex_end()) {
            return false;
        }
        
        // Same vertex is always reachable
        if (start_it == target_it) {
            return true;
        }
        
        SearchContext<State, Transition, StateIndexer> context;
        auto path = Search(graph, context, start, target);
        
        return !path.empty();
    }
    
    /**
     * @brief Convenience overload for IsReachable with shared_ptr
     */
    template<typename State, typename Transition, typename StateIndexer,
             typename VertexIdentifier>
    static bool IsReachable(
        std::shared_ptr<Graph<State, Transition, StateIndexer>> graph,
        VertexIdentifier start,
        VertexIdentifier target) {
        
        return IsReachable(graph.get(), start, target);
    }
};

// Compatibility typedefs for existing code
using BreadthFirstSearch = BFS;

} // namespace xmotion

#endif /* BFS_HPP */