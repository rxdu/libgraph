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
                             vertex_iterator goal_vertex) const {
        info.SetGCost(Transition{});  // Start at depth 0
        info.SetHCost(Transition{});  // BFS doesn't use heuristic
        info.SetFCost(Transition{});  // Same as g_cost for BFS
        info.SetChecked(false);
        info.SetInOpenList(false);
        info.SetParent(-1);
    }
    
    bool RelaxVertexImpl(SearchInfo& current_info, SearchInfo& successor_info,
                        vertex_iterator successor_vertex, vertex_iterator goal_vertex,
                        const Transition& edge_cost) const {
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
 * This class provides the public API for BFS searches using the strategy framework.
 * BFS finds the shortest path in terms of number of edges (unweighted shortest path).
 */
class BFS final {
public:
    /**
     * @brief Thread-safe BFS search with external search context
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
        
        auto strategy = MakeBfsStrategy<State, Transition, StateIndexer>();
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