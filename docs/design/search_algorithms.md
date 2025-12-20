# Search Algorithms Guide

This comprehensive guide covers all search algorithms available in libgraph, their use cases, performance characteristics, and implementation details.

## Table of Contents

- [Algorithm Overview](#algorithm-overview)
- [Dijkstra's Algorithm](#dijkstras-algorithm)
- [A* Algorithm](#a-algorithm)
- [Breadth-First Search (BFS)](#breadth-first-search-bfs)
- [Depth-First Search (DFS)](#depth-first-search-dfs)
- [Algorithm Comparison](#algorithm-comparison)
- [Custom Heuristics](#custom-heuristics)
- [Performance Optimization](#performance-optimization)
- [Advanced Usage Patterns](#advanced-usage-patterns)

## Algorithm Overview

libgraph provides four core search algorithms implemented with a unified framework. All algorithms share the same interface while providing specialized optimizations for different use cases.

### Common Interface

```cpp
// Basic usage (creates temporary SearchContext)
auto path = Algorithm::Search(graph, start, goal);

// Thread-safe usage (external SearchContext)
SearchContext<State> context;
auto path = Algorithm::Search(graph, context, start, goal);

// Custom comparator
auto path = Algorithm::Search(graph, context, start, goal, comparator);
```

### Unified Framework Benefits

- **Consistent API** across all algorithms
- **Thread-safe concurrent searches** using external SearchContext
- **Interchangeable algorithms** - easy to switch based on requirements
- **Performance optimization** through shared infrastructure

## Dijkstra's Algorithm

Dijkstra's algorithm finds the shortest path between vertices in a weighted graph with non-negative edge weights.

### When to Use Dijkstra

- **Guaranteed optimal paths** in weighted graphs
- **Non-negative edge weights** (requirement)
- **Single-source shortest paths** to all reachable vertices
- **No heuristic available** for A*

### Basic Usage

```cpp
#include "graph/search/dijkstra.hpp"

Graph<Location> city_map;
// ... populate graph ...

// Find shortest path
auto path = Dijkstra::Search(city_map, home, work);

// Thread-safe version
SearchContext<Location> context;
auto path = Dijkstra::Search(city_map, context, home, work);
```

### Advanced Usage

```cpp
// Custom cost comparison (for maximum instead of minimum paths)
struct MaxComparator {
    template<typename T>
    bool operator()(const T& a, const T& b) const {
        return a > b;  // Reverse comparison for maximum paths
    }
};

SearchContext<Location> context;
auto max_path = Dijkstra::Search(city_map, context, start, goal, MaxComparator{});

// Multi-criteria optimization with custom cost type
struct TravelCost {
    double time_hours;
    double fuel_cost;
    double comfort_level;
    
    bool operator<(const TravelCost& other) const {
        // Lexicographic comparison: time > cost > comfort
        if (time_hours != other.time_hours) return time_hours < other.time_hours;
        if (fuel_cost != other.fuel_cost) return fuel_cost < other.fuel_cost;
        return comfort_level > other.comfort_level;  // Higher comfort preferred
    }
    
    TravelCost operator+(const TravelCost& other) const {
        return {time_hours + other.time_hours, 
                fuel_cost + other.fuel_cost,
                std::min(comfort_level, other.comfort_level)};
    }
};

Graph<Location, TravelCost> travel_map;
auto optimal_travel = Dijkstra::Search(travel_map, context, start, goal);
```

### Performance Characteristics

- **Time Complexity**: O((V + E) log V) using binary heap
- **Space Complexity**: O(V) for distance tracking and priority queue
- **Optimal**: Always finds the shortest path for non-negative weights
- **Preprocessing**: None required

### Implementation Details

Dijkstra uses a priority queue (min-heap) to efficiently select the next vertex with minimum distance:

```cpp
// Simplified algorithm flow
void DijkstraImpl(Graph& graph, SearchContext& context, Start start, Goal goal) {
    context.distances[start] = 0;
    context.priority_queue.Push(start, 0);
    
    while (!context.priority_queue.Empty()) {
        auto current = context.priority_queue.Pop();
        
        if (current == goal) return ReconstructPath(context, start, goal);
        
        if (context.distances[current] < context.current_distance) continue;
        
        for (const auto& edge : graph.GetVertexPtr(current)->GetEdges()) {
            auto neighbor = edge.GetDst()->GetState();
            auto new_distance = context.distances[current] + edge.GetCost();
            
            if (new_distance < context.distances[neighbor]) {
                context.distances[neighbor] = new_distance;
                context.predecessors[neighbor] = current;
                context.priority_queue.UpdatePriority(neighbor, new_distance);
            }
        }
    }
}
```

## A* Algorithm

A* is an extension of Dijkstra that uses a heuristic function to guide the search toward the goal, often finding paths more efficiently.

### When to Use A*

- **Heuristic available** that estimates distance to goal
- **Single-pair shortest path** (specific start and goal)
- **Large search spaces** where heuristic can prune exploration
- **Admissible heuristic** required for optimality guarantee

### Basic Usage

```cpp
#include "graph/search/astar.hpp"

// Define heuristic function
double EuclideanDistance(const Location& from, const Location& to) {
    double dx = from.x - to.x;
    double dy = from.y - to.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Find path using A*
auto path = AStar::Search(city_map, home, work, EuclideanDistance);

// Thread-safe version
SearchContext<Location> context;
auto path = AStar::Search(city_map, context, home, work, EuclideanDistance);
```

### Heuristic Functions

The quality of the heuristic function significantly impacts A* performance:

```cpp
// Manhattan distance (good for grid-based movements)
double ManhattanDistance(const GridPoint& from, const GridPoint& to) {
    return std::abs(from.x - to.x) + std::abs(from.y - to.y);
}

// Euclidean distance (good for continuous 2D spaces)
double EuclideanDistance(const Point2D& from, const Point2D& to) {
    double dx = from.x - to.x;
    double dy = from.y - to.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Chebyshev distance (good for 8-directional movement)
double ChebyshevDistance(const GridPoint& from, const GridPoint& to) {
    return std::max(std::abs(from.x - to.x), std::abs(from.y - to.y));
}

// Custom domain-specific heuristic
double NetworkLatency(const NetworkNode& from, const NetworkNode& to) {
    // Estimate based on geographical distance and network topology
    double geo_distance = GeographicalDistance(from, to);
    double topology_factor = GetTopologyFactor(from, to);
    return geo_distance * topology_factor;
}
```

### Heuristic Admissibility

For optimal paths, heuristics must be admissible (never overestimate true cost):

```cpp
class HeuristicValidator {
public:
    template<typename State, typename HeuristicFunc>
    static bool IsAdmissible(const Graph<State>& graph, 
                           HeuristicFunc heuristic,
                           const State& goal,
                           size_t sample_size = 1000) {
        // Sample random states and check if heuristic <= actual distance
        for (size_t i = 0; i < sample_size; ++i) {
            State random_state = SampleRandomState(graph);
            
            double heuristic_estimate = heuristic(random_state, goal);
            double actual_distance = DijkstraDistance(graph, random_state, goal);
            
            if (heuristic_estimate > actual_distance + EPSILON) {
                return false;  // Heuristic overestimates
            }
        }
        return true;
    }
};
```

### Performance Characteristics

- **Time Complexity**: O((V + E) log V)* - can be much better with good heuristics
- **Space Complexity**: O(V) for search state
- **Optimal**: With admissible heuristic, always finds optimal path
- **Efficiency**: Often explores fewer nodes than Dijkstra

### Advanced A* Techniques

#### Weighted A* (A* with inadmissible heuristic)

```cpp
template<typename State, typename HeuristicFunc>
std::vector<State> WeightedAStar(const Graph<State>& graph,
                                const State& start,
                                const State& goal,
                                HeuristicFunc heuristic,
                                double weight = 1.5) {
    
    auto weighted_heuristic = [&heuristic, weight](const State& from, const State& to) {
        return weight * heuristic(from, to);  // May overestimate for faster search
    };
    
    SearchContext<State> context;
    return AStar::Search(graph, context, start, goal, weighted_heuristic);
}
```

#### Bidirectional A*

```cpp
template<typename State, typename HeuristicFunc>
class BidirectionalAStar {
public:
    static std::vector<State> Search(const Graph<State>& graph,
                                   const State& start,
                                   const State& goal,
                                   HeuristicFunc heuristic) {
        
        SearchContext<State> forward_context, backward_context;
        
        // Search from both ends simultaneously
        while (!forward_context.priority_queue.Empty() && 
               !backward_context.priority_queue.Empty()) {
            
            // Expand forward search
            if (ExpandFrontier(graph, forward_context, goal, heuristic)) {
                return ReconstructBidirectionalPath(forward_context, backward_context);
            }
            
            // Expand backward search  
            if (ExpandFrontier(graph, backward_context, start, heuristic)) {
                return ReconstructBidirectionalPath(forward_context, backward_context);
            }
        }
        
        return {};  // No path found
    }
};
```

## Breadth-First Search (BFS)

BFS finds the shortest path by number of edges (unweighted shortest path) using systematic level-by-level exploration.

### When to Use BFS

- **Unweighted graphs** or when all edges have equal cost
- **Shortest path by edge count** is desired
- **Level-order traversal** of graph structure
- **Finding all vertices at specific distance**

### Basic Usage

```cpp
#include "graph/search/bfs.hpp"

Graph<State> unweighted_graph;
// ... populate graph ...

// Find path with minimum edge count
auto path = BFS::Search(unweighted_graph, start, goal);

// Thread-safe version
SearchContext<State> context;
auto path = BFS::Search(unweighted_graph, context, start, goal);
```

### Advanced BFS Applications

```cpp
// Find all vertices at specific distance
template<typename State>
std::vector<State> FindVerticesAtDistance(const Graph<State>& graph, 
                                        const State& center, 
                                        size_t distance) {
    SearchContext<State> context;
    std::queue<std::pair<State, size_t>> queue;
    std::unordered_set<int64_t> visited;
    std::vector<State> result;
    
    DefaultIndexer<State> indexer;
    queue.push({center, 0});
    visited.insert(indexer(center));
    
    while (!queue.empty()) {
        auto [current_state, current_distance] = queue.front();
        queue.pop();
        
        if (current_distance == distance) {
            result.push_back(current_state);
            continue;  // Don't explore further from this vertex
        }
        
        if (current_distance < distance) {
            auto* vertex = graph.GetVertexPtr(current_state);
            for (const auto& edge : vertex->GetEdges()) {
                State neighbor = edge.GetDst()->GetState();
                int64_t neighbor_id = indexer(neighbor);
                
                if (visited.find(neighbor_id) == visited.end()) {
                    visited.insert(neighbor_id);
                    queue.push({neighbor, current_distance + 1});
                }
            }
        }
    }
    
    return result;
}

// Multi-source BFS for finding nearest facility
template<typename State>
std::unordered_map<int64_t, State> FindNearestFacilities(
    const Graph<State>& graph,
    const std::vector<State>& facilities) {
    
    std::queue<std::pair<State, State>> queue;  // {current, nearest_facility}
    std::unordered_map<int64_t, State> nearest_facility;
    DefaultIndexer<State> indexer;
    
    // Initialize with all facilities
    for (const auto& facility : facilities) {
        queue.push({facility, facility});
        nearest_facility[indexer(facility)] = facility;
    }
    
    while (!queue.empty()) {
        auto [current_state, facility] = queue.front();
        queue.pop();
        
        auto* vertex = graph.GetVertexPtr(current_state);
        for (const auto& edge : vertex->GetEdges()) {
            State neighbor = edge.GetDst()->GetState();
            int64_t neighbor_id = indexer(neighbor);
            
            if (nearest_facility.find(neighbor_id) == nearest_facility.end()) {
                nearest_facility[neighbor_id] = facility;
                queue.push({neighbor, facility});
            }
        }
    }
    
    return nearest_facility;
}
```

### Performance Characteristics

- **Time Complexity**: O(V + E) - visits each vertex and edge once
- **Space Complexity**: O(V) for queue and visited set
- **Optimal**: For unweighted graphs (minimum edge count)
- **Complete**: Always finds solution if one exists

## Depth-First Search (DFS)

DFS explores as far as possible along each branch before backtracking, useful for graph traversal and structural analysis.

### When to Use DFS

- **Graph traversal** and exploration
- **Topological sorting** of DAGs
- **Cycle detection** in graphs
- **Connected component analysis**
- **Path existence** queries (not necessarily shortest)

### Basic Usage

```cpp
#include "graph/search/dfs.hpp"

Graph<State> graph;
// ... populate graph ...

// Find any path (not necessarily shortest)
auto path = DFS::Search(graph, start, goal);

// Thread-safe version
SearchContext<State> context;
auto path = DFS::Search(graph, context, start, goal);
```

### Advanced DFS Applications

```cpp
// Topological sort using DFS
template<typename State>
std::vector<State> TopologicalSort(const Graph<State>& graph) {
    std::vector<State> result;
    std::unordered_set<int64_t> visited;
    std::unordered_set<int64_t> recursion_stack;
    DefaultIndexer<State> indexer;
    
    std::function<bool(const State&)> dfs_visit = [&](const State& state) -> bool {
        int64_t state_id = indexer(state);
        
        if (recursion_stack.find(state_id) != recursion_stack.end()) {
            return false;  // Cycle detected
        }
        
        if (visited.find(state_id) != visited.end()) {
            return true;  // Already processed
        }
        
        visited.insert(state_id);
        recursion_stack.insert(state_id);
        
        auto* vertex = graph.GetVertexPtr(state);
        for (const auto& edge : vertex->GetEdges()) {
            if (!dfs_visit(edge.GetDst()->GetState())) {
                return false;  // Cycle found in subtree
            }
        }
        
        recursion_stack.erase(state_id);
        result.push_back(state);  // Add to result after visiting all neighbors
        return true;
    };
    
    // Visit all vertices
    for (const auto& vertex : graph.GetVertices()) {
        if (visited.find(indexer(vertex.GetState())) == visited.end()) {
            if (!dfs_visit(vertex.GetState())) {
                throw std::runtime_error("Graph contains cycle - topological sort impossible");
            }
        }
    }
    
    std::reverse(result.begin(), result.end());  // Reverse for correct order
    return result;
}

// Find strongly connected components using Tarjan's algorithm
template<typename State>
class StronglyConnectedComponents {
private:
    struct VertexInfo {
        size_t index = SIZE_MAX;
        size_t lowlink = SIZE_MAX;
        bool on_stack = false;
    };
    
    std::unordered_map<int64_t, VertexInfo> vertex_info_;
    std::stack<State> stack_;
    std::vector<std::vector<State>> components_;
    size_t index_counter_ = 0;
    DefaultIndexer<State> indexer_;
    
public:
    std::vector<std::vector<State>> FindSCCs(const Graph<State>& graph) {
        vertex_info_.clear();
        components_.clear();
        index_counter_ = 0;
        
        for (const auto& vertex : graph.GetVertices()) {
            State state = vertex.GetState();
            int64_t state_id = indexer_(state);
            
            if (vertex_info_[state_id].index == SIZE_MAX) {
                StrongConnect(graph, state);
            }
        }
        
        return components_;
    }
    
private:
    void StrongConnect(const Graph<State>& graph, const State& state) {
        int64_t state_id = indexer_(state);
        VertexInfo& info = vertex_info_[state_id];
        
        info.index = info.lowlink = index_counter_++;
        stack_.push(state);
        info.on_stack = true;
        
        auto* vertex = graph.GetVertexPtr(state);
        for (const auto& edge : vertex->GetEdges()) {
            State neighbor = edge.GetDst()->GetState();
            int64_t neighbor_id = indexer_(neighbor);
            VertexInfo& neighbor_info = vertex_info_[neighbor_id];
            
            if (neighbor_info.index == SIZE_MAX) {
                StrongConnect(graph, neighbor);
                info.lowlink = std::min(info.lowlink, neighbor_info.lowlink);
            } else if (neighbor_info.on_stack) {
                info.lowlink = std::min(info.lowlink, neighbor_info.index);
            }
        }
        
        // If state is root of SCC, pop the stack and create component
        if (info.lowlink == info.index) {
            std::vector<State> component;
            State current;
            do {
                current = stack_.top();
                stack_.pop();
                vertex_info_[indexer_(current)].on_stack = false;
                component.push_back(current);
            } while (!(current == state));
            
            components_.push_back(std::move(component));
        }
    }
};
```

### Performance Characteristics

- **Time Complexity**: O(V + E) - visits each vertex and edge once
- **Space Complexity**: O(V) for recursion stack (or explicit stack)
- **Not Optimal**: Does not guarantee shortest paths
- **Complete**: Finds a solution if one exists

## Algorithm Comparison

### Performance Summary

| Algorithm | Time Complexity | Space Complexity | Optimality | Best Use Case |
|-----------|----------------|------------------|------------|---------------|
| **Dijkstra** | O((V+E) log V) | O(V) | Guaranteed | Weighted shortest paths |
| **A*** | O((V+E) log V)* | O(V) | With admissible h | Heuristic-guided search |
| **BFS** | O(V + E) | O(V) | Unweighted only | Minimum edge count |
| **DFS** | O(V + E) | O(V) | No | Graph traversal |

*A* can be significantly faster than Dijkstra with a good heuristic

### Decision Matrix

Choose your algorithm based on these criteria:

```cpp
// Decision helper function
template<typename State>
class AlgorithmSelector {
public:
    enum class GraphType { WEIGHTED, UNWEIGHTED };
    enum class PathType { SHORTEST, ANY, EXPLORATION };
    enum class HeuristicAvailable { YES, NO };
    
    static std::string RecommendAlgorithm(GraphType graph_type,
                                        PathType path_type,
                                        HeuristicAvailable heuristic) {
        
        if (path_type == PathType::EXPLORATION) {
            return "DFS";
        }
        
        if (graph_type == GraphType::UNWEIGHTED && path_type == PathType::SHORTEST) {
            return "BFS";
        }
        
        if (graph_type == GraphType::WEIGHTED && path_type == PathType::SHORTEST) {
            if (heuristic == HeuristicAvailable::YES) {
                return "A*";
            } else {
                return "Dijkstra";
            }
        }
        
        if (path_type == PathType::ANY) {
            return "DFS";  // Fastest for any path
        }
        
        return "Dijkstra";  // Safe default
    }
};

// Usage example
auto recommendation = AlgorithmSelector<Location>::RecommendAlgorithm(
    AlgorithmSelector<Location>::GraphType::WEIGHTED,
    AlgorithmSelector<Location>::PathType::SHORTEST,
    AlgorithmSelector<Location>::HeuristicAvailable::YES
);
// Returns "A*"
```

## Custom Heuristics

### Designing Effective Heuristics

Good heuristics should be:

1. **Admissible**: Never overestimate the true cost
2. **Consistent**: h(n) ≤ cost(n, n') + h(n') for all neighbors n'
3. **Efficient**: Fast to compute
4. **Informative**: Provide good estimates to guide search

### Domain-Specific Examples

```cpp
// Game AI pathfinding with obstacles
struct GameHeuristic {
    const std::unordered_set<GridPoint>& obstacles;
    
    double operator()(const GridPoint& from, const GridPoint& to) const {
        // Base Manhattan distance
        double base_distance = std::abs(from.x - to.x) + std::abs(from.y - to.y);
        
        // Penalty for proximity to obstacles (inadmissible but often effective)
        double obstacle_penalty = 0.0;
        for (const auto& obstacle : obstacles) {
            double dist_to_obstacle = ManhattanDistance(from, obstacle);
            if (dist_to_obstacle < 3.0) {  // Close to obstacle
                obstacle_penalty += (3.0 - dist_to_obstacle) * 0.5;
            }
        }
        
        return base_distance + obstacle_penalty;
    }
};

// Network routing with bandwidth considerations
struct NetworkHeuristic {
    const std::unordered_map<int64_t, double>& bandwidth_map;
    
    double operator()(const NetworkNode& from, const NetworkNode& to) const {
        // Geographic distance as base
        double distance = GeographicalDistance(from, to);
        
        // Factor in available bandwidth (higher bandwidth = lower cost)
        auto it = bandwidth_map.find(from.node_id);
        if (it != bandwidth_map.end() && it->second > 0) {
            return distance / it->second;  // Inverse relationship
        }
        
        return distance;
    }
};

// Multi-level heuristic for hierarchical planning
struct HierarchicalHeuristic {
    const Graph<HighLevelNode>& abstract_graph;
    
    double operator()(const DetailedNode& from, const DetailedNode& to) const {
        // Map detailed nodes to high-level representation
        HighLevelNode from_abstract = MapToAbstract(from);
        HighLevelNode to_abstract = MapToAbstract(to);
        
        if (from_abstract == to_abstract) {
            // Same high-level region - use detailed heuristic
            return DetailedDistance(from, to);
        } else {
            // Different regions - use abstract path length
            SearchContext<HighLevelNode> context;
            auto abstract_path = Dijkstra::Search(abstract_graph, context, 
                                                from_abstract, to_abstract);
            
            if (!abstract_path.empty()) {
                return CalculateAbstractPathCost(abstract_path);
            }
        }
        
        return std::numeric_limits<double>::max();  // No path in abstract graph
    }
};
```

## Performance Optimization

### Search Context Reuse

```cpp
class OptimizedPathfinder {
private:
    SearchContext<Location> reusable_context_;
    
public:
    OptimizedPathfinder(size_t estimated_graph_size) {
        reusable_context_.PreAllocate(estimated_graph_size);
    }
    
    std::vector<Location> FindPath(const Graph<Location>& graph,
                                 const Location& start,
                                 const Location& goal) {
        reusable_context_.Reset();  // Clear previous search state
        return Dijkstra::Search(graph, reusable_context_, start, goal);
    }
    
    // Batch pathfinding with context reuse
    std::vector<std::vector<Location>> FindMultiplePaths(
        const Graph<Location>& graph,
        const std::vector<std::pair<Location, Location>>& queries) {
        
        std::vector<std::vector<Location>> results;
        results.reserve(queries.size());
        
        for (const auto& query : queries) {
            reusable_context_.Reset();
            results.push_back(Dijkstra::Search(graph, reusable_context_, 
                                             query.first, query.second));
        }
        
        return results;
    }
};
```

### Algorithm Selection Based on Graph Properties

```cpp
template<typename State, typename Transition>
class AdaptivePathfinder {
public:
    std::vector<State> FindOptimalPath(const Graph<State, Transition>& graph,
                                     const State& start,
                                     const State& goal) {
        
        // Analyze graph properties
        size_t vertex_count = graph.GetVertexNumber();
        double edge_density = CalculateEdgeDensity(graph);
        bool has_negative_weights = HasNegativeWeights(graph);
        
        SearchContext<State> context;
        
        // Select algorithm based on graph characteristics
        if (has_negative_weights) {
            throw std::invalid_argument("Negative weights not supported");
        }
        
        if (vertex_count < 100) {
            // Small graph - Dijkstra is fine
            return Dijkstra::Search(graph, context, start, goal);
        }
        
        if (edge_density < 0.1) {
            // Sparse graph - BFS might be appropriate for unweighted
            if (IsUnweighted(graph)) {
                return BFS::Search(graph, context, start, goal);
            }
        }
        
        // Try to use A* if heuristic is available
        if (HasSpatialCoordinates(start) && HasSpatialCoordinates(goal)) {
            auto heuristic = [](const State& from, const State& to) {
                return EuclideanDistance(ExtractCoordinates(from), 
                                       ExtractCoordinates(to));
            };
            return AStar::Search(graph, context, start, goal, heuristic);
        }
        
        // Default to Dijkstra
        return Dijkstra::Search(graph, context, start, goal);
    }
};
```

## Advanced Usage Patterns

### Early Termination

```cpp
template<typename State, typename Predicate>
std::vector<State> SearchWithEarlyTermination(const Graph<State>& graph,
                                            const State& start,
                                            Predicate goal_predicate) {
    SearchContext<State> context;
    DynamicPriorityQueue<State, double> queue;
    std::unordered_map<int64_t, State> predecessors;
    std::unordered_set<int64_t> visited;
    DefaultIndexer<State> indexer;
    
    queue.Push(start, 0.0);
    context.distances[indexer(start)] = 0.0;
    
    while (!queue.Empty()) {
        State current = queue.Pop();
        int64_t current_id = indexer(current);
        
        if (visited.find(current_id) != visited.end()) continue;
        visited.insert(current_id);
        
        // Early termination check
        if (goal_predicate(current)) {
            return ReconstructPath(predecessors, start, current);
        }
        
        auto* vertex = graph.GetVertexPtr(current);
        for (const auto& edge : vertex->GetEdges()) {
            State neighbor = edge.GetDst()->GetState();
            int64_t neighbor_id = indexer(neighbor);
            
            if (visited.find(neighbor_id) != visited.end()) continue;
            
            double new_distance = context.distances[current_id] + edge.GetCost();
            
            if (context.distances.find(neighbor_id) == context.distances.end() ||
                new_distance < context.distances[neighbor_id]) {
                
                context.distances[neighbor_id] = new_distance;
                predecessors[neighbor_id] = current;
                queue.UpdatePriority(neighbor, new_distance);
            }
        }
    }
    
    return {};  // No goal found
}

// Usage example
auto path = SearchWithEarlyTermination(graph, start, [](const Location& loc) {
    return loc.type == LocationType::HOSPITAL;  // Find any hospital
});
```

### Path Post-processing

```cpp
template<typename State>
class PathOptimizer {
public:
    // Smooth path by removing unnecessary waypoints
    static std::vector<State> SmoothPath(const Graph<State>& graph,
                                       const std::vector<State>& original_path) {
        if (original_path.size() <= 2) return original_path;
        
        std::vector<State> smoothed_path;
        smoothed_path.push_back(original_path.front());
        
        size_t current = 0;
        while (current < original_path.size() - 1) {
            size_t farthest = current + 1;
            
            // Find the farthest reachable waypoint
            for (size_t test = current + 2; test < original_path.size(); ++test) {
                if (HasDirectPath(graph, original_path[current], original_path[test])) {
                    farthest = test;
                } else {
                    break;  // Can't reach further
                }
            }
            
            smoothed_path.push_back(original_path[farthest]);
            current = farthest;
        }
        
        return smoothed_path;
    }
    
    // Validate path integrity
    static bool ValidatePath(const Graph<State>& graph,
                           const std::vector<State>& path) {
        if (path.empty()) return true;
        
        for (size_t i = 0; i < path.size() - 1; ++i) {
            if (!graph.HasEdge(path[i], path[i + 1])) {
                return false;  // Gap in path
            }
        }
        
        return true;
    }
    
    // Calculate total path cost
    template<typename Transition>
    static Transition CalculatePathCost(const Graph<State, Transition>& graph,
                                      const std::vector<State>& path) {
        if (path.empty()) return Transition{};
        
        Transition total_cost{};
        
        for (size_t i = 0; i < path.size() - 1; ++i) {
            auto* vertex = graph.GetVertexPtr(path[i]);
            bool found = false;
            
            for (const auto& edge : vertex->GetEdges()) {
                if (edge.GetDst()->GetState() == path[i + 1]) {
                    total_cost += edge.GetCost();
                    found = true;
                    break;
                }
            }
            
            if (!found) {
                throw std::runtime_error("Invalid path - missing edge");
            }
        }
        
        return total_cost;
    }
};
```

This comprehensive guide provides the foundation for effectively using all search algorithms in libgraph, from basic pathfinding to advanced optimization techniques.