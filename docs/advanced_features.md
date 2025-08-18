# Advanced Features Guide

This guide covers advanced features and customization options in libgraph for users who need to extend beyond basic graph operations.

## Table of Contents

- [Custom Cost Types](#custom-cost-types)
- [Thread-Safe Concurrent Operations](#thread-safe-concurrent-operations)
- [Performance Optimization](#performance-optimization)
- [Custom State Indexing](#custom-state-indexing)
- [Graph Validation and Error Handling](#graph-validation-and-error-handling)
- [Memory Management Best Practices](#memory-management-best-practices)
- [Advanced Search Patterns](#advanced-search-patterns)
- [Integration Patterns](#integration-patterns)

## Custom Cost Types

### Multi-Criteria Cost Types

For problems requiring optimization across multiple objectives:

```cpp
struct TravelCost {
    double time_hours;
    double distance_km; 
    double monetary_cost;
    double comfort_level;
    
    TravelCost(double t = 0, double d = 0, double c = 0, double comfort = 0)
        : time_hours(t), distance_km(d), monetary_cost(c), comfort_level(comfort) {}
    
    // Lexicographic comparison: time > distance > cost > comfort
    bool operator<(const TravelCost& other) const {
        if (time_hours != other.time_hours) return time_hours < other.time_hours;
        if (distance_km != other.distance_km) return distance_km < other.distance_km;
        if (monetary_cost != other.monetary_cost) return monetary_cost < other.monetary_cost;
        return comfort_level > other.comfort_level;  // Higher comfort is better
    }
    
    // Required operators
    bool operator>(const TravelCost& other) const { return other < *this; }
    bool operator<=(const TravelCost& other) const { return !(*this > other); }
    bool operator>=(const TravelCost& other) const { return !(*this < other); }
    bool operator==(const TravelCost& other) const {
        return time_hours == other.time_hours && 
               distance_km == other.distance_km && 
               monetary_cost == other.monetary_cost &&
               comfort_level == other.comfort_level;
    }
    bool operator!=(const TravelCost& other) const { return !(*this == other); }
    
    // Cost accumulation
    TravelCost operator+(const TravelCost& other) const {
        return TravelCost(
            time_hours + other.time_hours,
            distance_km + other.distance_km,
            monetary_cost + other.monetary_cost,
            std::min(comfort_level, other.comfort_level)  // Worst comfort of path
        );
    }
    
    TravelCost& operator+=(const TravelCost& other) {
        time_hours += other.time_hours;
        distance_km += other.distance_km;
        monetary_cost += other.monetary_cost;
        comfort_level = std::min(comfort_level, other.comfort_level);
        return *this;
    }
    
    // For A* heuristic compatibility (optional)
    TravelCost operator-(const TravelCost& other) const {
        return TravelCost(
            std::max(0.0, time_hours - other.time_hours),
            std::max(0.0, distance_km - other.distance_km),
            std::max(0.0, monetary_cost - other.monetary_cost),
            comfort_level  // Comfort doesn't subtract meaningfully
        );
    }
};

// Required: Cost traits specialization
namespace xmotion {
    template<>
    struct CostTraits<TravelCost> {
        static TravelCost infinity() {
            return TravelCost(
                std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max(),
                0.0  // Minimum comfort
            );
        }
    };
}
```

### Tuple-Based Automatic Comparison

For simpler multi-criteria costs, use std::tuple for automatic lexicographic comparison:

```cpp
struct NetworkCost {
    std::tuple<int, double, double> values;  // (priority, latency, bandwidth)
    
    NetworkCost(int priority = 0, double latency = 0.0, double bandwidth = 0.0)
        : values(priority, latency, bandwidth) {}
    
    // Tuple provides automatic lexicographic comparison
    bool operator<(const NetworkCost& other) const { return values < other.values; }
    bool operator>(const NetworkCost& other) const { return values > other.values; }
    bool operator<=(const NetworkCost& other) const { return values <= other.values; }
    bool operator>=(const NetworkCost& other) const { return values >= other.values; }
    bool operator==(const NetworkCost& other) const { return values == other.values; }
    bool operator!=(const NetworkCost& other) const { return values != other.values; }
    
    NetworkCost operator+(const NetworkCost& other) const {
        return NetworkCost(
            std::get<0>(values) + std::get<0>(other.values),
            std::get<1>(values) + std::get<1>(other.values),
            std::get<2>(values) + std::get<2>(other.values)
        );
    }
    
    static NetworkCost max() {
        return NetworkCost(
            std::numeric_limits<int>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max()
        );
    }
};
```

### Custom Comparators

Override default comparison behavior:

```cpp
// Reverse comparison for maximum-cost paths
struct MaxCostComparator {
    template<typename T>
    bool operator()(const T& a, const T& b) const {
        return a > b;  // Reverse normal comparison
    }
};

// Usage with Dijkstra
auto path = Dijkstra::Search(graph, context, start, goal, MaxCostComparator{});
```

## Thread-Safe Concurrent Operations

### Basic Concurrent Searches

```cpp
#include <thread>
#include <vector>
#include <future>

class ConcurrentPathfinder {
private:
    const Graph<Location, double>& graph_;
    
public:
    ConcurrentPathfinder(const Graph<Location, double>& g) : graph_(g) {}
    
    // Concurrent path finding for multiple queries
    std::vector<std::vector<Location>> FindMultiplePaths(
        const std::vector<std::pair<Location, Location>>& queries) {
        
        std::vector<std::future<std::vector<Location>>> futures;
        
        for (const auto& query : queries) {
            futures.emplace_back(std::async(std::launch::async, [this, query]() {
                SearchContext<Location> context;  // Thread-local context
                return Dijkstra::Search(graph_, context, query.first, query.second);
            }));
        }
        
        std::vector<std::vector<Location>> results;
        for (auto& future : futures) {
            results.push_back(future.get());
        }
        
        return results;
    }
};
```

### Thread-Safe Graph Modifications

```cpp
#include <shared_mutex>

template<typename State, typename Transition = double>
class ThreadSafeGraph {
private:
    Graph<State, Transition> graph_;
    mutable std::shared_mutex mutex_;
    
public:
    // Exclusive write operations
    void AddVertex(const State& state) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        graph_.AddVertex(state);
    }
    
    void AddEdge(const State& from, const State& to, const Transition& cost) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        graph_.AddEdge(from, to, cost);
    }
    
    bool RemoveVertex(const State& state) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        return graph_.RemoveVertex(state);
    }
    
    // Shared read operations (concurrent access)
    template<typename... Args>
    auto Search(Args&&... args) const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return Dijkstra::Search(graph_, std::forward<Args>(args)...);
    }
    
    size_t GetVertexNumber() const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return graph_.GetVertexNumber();
    }
    
    // Read-only access to internal graph (for complex operations)
    template<typename Func>
    auto WithReadLock(Func&& func) const -> decltype(func(graph_)) {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return func(graph_);
    }
    
    template<typename Func>
    auto WithWriteLock(Func&& func) -> decltype(func(graph_)) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        return func(graph_);
    }
};
```

### Producer-Consumer Pattern

```cpp
#include <queue>
#include <condition_variable>

class PathfindingService {
private:
    struct PathRequest {
        Location start, goal;
        std::promise<std::vector<Location>> result;
    };
    
    const Graph<Location>& graph_;
    std::queue<PathRequest> request_queue_;
    std::mutex queue_mutex_;
    std::condition_variable cv_;
    std::atomic<bool> shutdown_{false};
    std::vector<std::thread> workers_;
    
public:
    PathfindingService(const Graph<Location>& graph, int num_workers = 4) 
        : graph_(graph) {
        
        for (int i = 0; i < num_workers; ++i) {
            workers_.emplace_back([this]() { WorkerLoop(); });
        }
    }
    
    ~PathfindingService() {
        shutdown_ = true;
        cv_.notify_all();
        for (auto& worker : workers_) {
            if (worker.joinable()) worker.join();
        }
    }
    
    std::future<std::vector<Location>> FindPathAsync(const Location& start, const Location& goal) {
        PathRequest request;
        request.start = start;
        request.goal = goal;
        auto future = request.result.get_future();
        
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            request_queue_.push(std::move(request));
        }
        cv_.notify_one();
        
        return future;
    }
    
private:
    void WorkerLoop() {
        SearchContext<Location> context;  // Thread-local search context
        
        while (!shutdown_) {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            cv_.wait(lock, [this]() { return !request_queue_.empty() || shutdown_; });
            
            if (shutdown_) break;
            
            PathRequest request = std::move(request_queue_.front());
            request_queue_.pop();
            lock.unlock();
            
            try {
                auto path = Dijkstra::Search(graph_, context, request.start, request.goal);
                request.result.set_value(std::move(path));
            } catch (...) {
                request.result.set_exception(std::current_exception());
            }
            
            context.Reset();  // Prepare for next search
        }
    }
};
```

## Performance Optimization

### Memory Pre-allocation

```cpp
class OptimizedPathfinder {
private:
    Graph<GameState> game_world_;
    SearchContext<GameState> reusable_context_;
    
public:
    OptimizedPathfinder(size_t expected_world_size) {
        // Pre-allocate graph capacity
        game_world_.reserve(expected_world_size);
        
        // Pre-allocate search context memory
        reusable_context_.PreAllocate(expected_world_size);
    }
    
    std::vector<GameState> FindPath(const GameState& start, const GameState& goal) {
        // Reuse pre-allocated context
        reusable_context_.Reset();  // Clear previous search state
        return Dijkstra::Search(game_world_, reusable_context_, start, goal);
    }
    
    // Batch vertex addition for efficiency
    void AddVerticesBatch(const std::vector<GameState>& states) {
        for (const auto& state : states) {
            game_world_.AddVertex(state);
        }
    }
};
```

### Context Pool for High-Throughput Applications

```cpp
template<typename State>
class ContextPool {
private:
    std::vector<std::unique_ptr<SearchContext<State>>> contexts_;
    std::queue<SearchContext<State>*> available_;
    std::mutex mutex_;
    
public:
    ContextPool(size_t pool_size, size_t estimated_graph_size) {
        for (size_t i = 0; i < pool_size; ++i) {
            auto context = std::make_unique<SearchContext<State>>();
            context->PreAllocate(estimated_graph_size);
            
            available_.push(context.get());
            contexts_.push_back(std::move(context));
        }
    }
    
    class ContextGuard {
        SearchContext<State>* context_;
        ContextPool* pool_;
        
    public:
        ContextGuard(SearchContext<State>* ctx, ContextPool* p) : context_(ctx), pool_(p) {
            context_->Reset();
        }
        
        ~ContextGuard() {
            pool_->ReturnContext(context_);
        }
        
        SearchContext<State>& operator*() { return *context_; }
        SearchContext<State>* operator->() { return context_; }
    };
    
    ContextGuard AcquireContext() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (available_.empty()) {
            throw std::runtime_error("No available contexts in pool");
        }
        
        auto* context = available_.front();
        available_.pop();
        return ContextGuard(context, this);
    }
    
private:
    void ReturnContext(SearchContext<State>* context) {
        std::lock_guard<std::mutex> lock(mutex_);
        available_.push(context);
    }
};
```

## Custom State Indexing

### Complex State Indexing

```cpp
struct GameState {
    int x, y, level;
    int health, ammo;
    std::bitset<8> inventory;
    
    // Custom equality for state comparison
    bool operator==(const GameState& other) const {
        return x == other.x && y == other.y && level == other.level &&
               health == other.health && ammo == other.ammo && 
               inventory == other.inventory;
    }
};

struct GameStateIndexer {
    int64_t operator()(const GameState& state) const {
        // Combine multiple fields into unique ID
        int64_t id = 0;
        
        // Position components (assume limited ranges)
        id |= (static_cast<int64_t>(state.x & 0xFFFF)) << 48;
        id |= (static_cast<int64_t>(state.y & 0xFFFF)) << 32;
        id |= (static_cast<int64_t>(state.level & 0xFF)) << 24;
        
        // State components
        id |= (static_cast<int64_t>(state.health & 0xFF)) << 16;
        id |= (static_cast<int64_t>(state.ammo & 0xFF)) << 8;
        id |= (state.inventory.to_ulong() & 0xFF);
        
        return id;
    }
};

// Usage
Graph<GameState, double, GameStateIndexer> game_graph;
```

### Hash-Based Indexing

```cpp
struct ComplexState {
    std::string location_name;
    std::vector<int> properties;
    std::map<std::string, double> attributes;
    
    // Provide hash function for indexing
    struct Hash {
        size_t operator()(const ComplexState& state) const {
            size_t h1 = std::hash<std::string>{}(state.location_name);
            
            size_t h2 = 0;
            for (int prop : state.properties) {
                h2 ^= std::hash<int>{}(prop) + 0x9e3779b9 + (h2 << 6) + (h2 >> 2);
            }
            
            size_t h3 = 0;
            for (const auto& attr : state.attributes) {
                h3 ^= std::hash<std::string>{}(attr.first) + 0x9e3779b9 + (h3 << 6) + (h3 >> 2);
                h3 ^= std::hash<double>{}(attr.second) + 0x9e3779b9 + (h3 << 6) + (h3 >> 2);
            }
            
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };
};

struct ComplexStateIndexer {
    int64_t operator()(const ComplexState& state) const {
        // Use hash function and convert to int64_t
        ComplexState::Hash hasher;
        return static_cast<int64_t>(hasher(state));
    }
};
```

## Graph Validation and Error Handling

### Comprehensive Graph Validation

```cpp
class GraphValidator {
public:
    template<typename State, typename Transition>
    static ValidationResult ValidateGraph(const Graph<State, Transition>& graph) {
        ValidationResult result;
        
        // Check for orphaned vertices
        auto orphaned = FindOrphanedVertices(graph);
        if (!orphaned.empty()) {
            result.warnings.push_back("Found " + std::to_string(orphaned.size()) + " orphaned vertices");
        }
        
        // Check for self-loops
        auto self_loops = FindSelfLoops(graph);
        if (!self_loops.empty()) {
            result.warnings.push_back("Found " + std::to_string(self_loops.size()) + " self-loops");
        }
        
        // Check for negative edge weights (if cost type supports comparison)
        auto negative_edges = FindNegativeEdges(graph);
        if (!negative_edges.empty()) {
            result.errors.push_back("Found " + std::to_string(negative_edges.size()) + " negative edges");
        }
        
        // Check connectivity
        if (!IsStronglyConnected(graph)) {
            result.info.push_back("Graph is not strongly connected");
        }
        
        return result;
    }
    
    struct ValidationResult {
        std::vector<std::string> errors;
        std::vector<std::string> warnings;  
        std::vector<std::string> info;
        
        bool IsValid() const { return errors.empty(); }
        
        void PrintReport() const {
            for (const auto& error : errors) {
                std::cout << "ERROR: " << error << std::endl;
            }
            for (const auto& warning : warnings) {
                std::cout << "WARNING: " << warning << std::endl;
            }
            for (const auto& info_msg : info) {
                std::cout << "INFO: " << info_msg << std::endl;
            }
        }
    };
};
```

### Custom Exception Handling

```cpp
class PathfindingException : public std::runtime_error {
public:
    PathfindingException(const std::string& msg) : std::runtime_error(msg) {}
};

class NoPathException : public PathfindingException {
public:
    template<typename State>
    NoPathException(const State& start, const State& goal) 
        : PathfindingException("No path found from " + ToString(start) + " to " + ToString(goal)) {}
};

template<typename State>
std::vector<State> SafeSearch(const Graph<State>& graph, 
                             const State& start, 
                             const State& goal) {
    try {
        // Validate inputs
        if (!graph.GetVertexPtr(start)) {
            throw PathfindingException("Start vertex not found in graph");
        }
        if (!graph.GetVertexPtr(goal)) {
            throw PathfindingException("Goal vertex not found in graph");
        }
        
        auto path = Dijkstra::Search(graph, start, goal);
        
        if (path.empty()) {
            throw NoPathException(start, goal);
        }
        
        return path;
        
    } catch (const std::exception& e) {
        std::cerr << "Search failed: " << e.what() << std::endl;
        throw;  // Re-throw for caller to handle
    }
}
```

## Memory Management Best Practices

### RAII Graph Management

```cpp
class ManagedGraph {
private:
    std::unique_ptr<Graph<Location>> graph_;
    
public:
    ManagedGraph() : graph_(std::make_unique<Graph<Location>>()) {}
    
    // Move-only semantics for efficiency
    ManagedGraph(const ManagedGraph&) = delete;
    ManagedGraph& operator=(const ManagedGraph&) = delete;
    
    ManagedGraph(ManagedGraph&&) = default;
    ManagedGraph& operator=(ManagedGraph&&) = default;
    
    // Safe access to graph
    Graph<Location>& GetGraph() { 
        if (!graph_) throw std::runtime_error("Graph not initialized");
        return *graph_; 
    }
    
    const Graph<Location>& GetGraph() const { 
        if (!graph_) throw std::runtime_error("Graph not initialized");
        return *graph_; 
    }
    
    // Bulk operations for efficiency
    void LoadFromFile(const std::string& filename) {
        auto new_graph = std::make_unique<Graph<Location>>();
        
        // Load data into new graph...
        // If loading fails, old graph remains intact
        
        graph_ = std::move(new_graph);  // Atomic swap
    }
};
```

### Memory Pool for Large Graphs

```cpp
template<typename T>
class MemoryPool {
private:
    struct Block {
        alignas(T) char data[sizeof(T)];
        bool occupied = false;
    };
    
    std::vector<Block> pool_;
    std::stack<size_t> free_indices_;
    
public:
    MemoryPool(size_t initial_size) : pool_(initial_size) {
        for (size_t i = initial_size; i > 0; --i) {
            free_indices_.push(i - 1);
        }
    }
    
    template<typename... Args>
    T* Allocate(Args&&... args) {
        if (free_indices_.empty()) {
            // Expand pool
            size_t old_size = pool_.size();
            pool_.resize(old_size * 2);
            for (size_t i = pool_.size(); i > old_size; --i) {
                free_indices_.push(i - 1);
            }
        }
        
        size_t index = free_indices_.top();
        free_indices_.pop();
        
        Block& block = pool_[index];
        block.occupied = true;
        
        return new (block.data) T(std::forward<Args>(args)...);
    }
    
    void Deallocate(T* ptr) {
        // Find block index
        size_t index = static_cast<Block*>(ptr) - pool_.data();
        
        pool_[index].occupied = false;
        ptr->~T();
        
        free_indices_.push(index);
    }
};
```

## Advanced Search Patterns

### Bi-directional Search

```cpp
template<typename State>
class BidirectionalSearch {
private:
    struct SearchFrontier {
        SearchContext<State> context;
        std::unordered_set<int64_t> visited;
        std::unordered_map<int64_t, State> came_from;
    };
    
public:
    static std::vector<State> Search(const Graph<State>& graph,
                                   const State& start,
                                   const State& goal) {
        SearchFrontier forward_search, backward_search;
        
        // Initialize searches
        forward_search.context.Reset();
        backward_search.context.Reset();
        
        // Implement bidirectional search logic...
        // Meet in the middle for improved performance
        
        return ConstructPath(meeting_point, forward_search, backward_search);
    }
};
```

### A* with Dynamic Heuristic

```cpp
template<typename State>
class AdaptiveAStar {
private:
    mutable std::unordered_map<int64_t, double> heuristic_cache_;
    
public:
    std::vector<State> Search(const Graph<State>& graph,
                            const State& start,
                            const State& goal) const {
        
        auto adaptive_heuristic = [this, &goal](const State& current) -> double {
            int64_t current_id = DefaultIndexer<State>{}(current);
            
            auto it = heuristic_cache_.find(current_id);
            if (it != heuristic_cache_.end()) {
                return it->second;
            }
            
            // Compute base heuristic
            double h = EuclideanDistance(current, goal);
            
            // Adapt based on search experience
            // (This is simplified - real implementation would use learning)
            heuristic_cache_[current_id] = h;
            
            return h;
        };
        
        SearchContext<State> context;
        return AStar::Search(graph, context, start, goal, adaptive_heuristic);
    }
};
```

## Integration Patterns

### Graph Builder Pattern

```cpp
template<typename State, typename Transition = double>
class GraphBuilder {
private:
    Graph<State, Transition> graph_;
    
public:
    GraphBuilder& AddVertex(const State& state) {
        graph_.AddVertex(state);
        return *this;
    }
    
    GraphBuilder& AddEdge(const State& from, const State& to, const Transition& cost) {
        graph_.AddEdge(from, to, cost);
        return *this;
    }
    
    GraphBuilder& AddBidirectionalEdge(const State& a, const State& b, const Transition& cost) {
        graph_.AddEdge(a, b, cost);
        graph_.AddEdge(b, a, cost);
        return *this;
    }
    
    template<typename Container>
    GraphBuilder& AddVertices(const Container& vertices) {
        for (const auto& vertex : vertices) {
            graph_.AddVertex(vertex);
        }
        return *this;
    }
    
    Graph<State, Transition> Build() && {
        return std::move(graph_);
    }
    
    const Graph<State, Transition>& Build() const& {
        return graph_;
    }
};

// Usage
auto graph = GraphBuilder<Location>{}
    .AddVertex({"A", 0, 0})
    .AddVertex({"B", 10, 0})
    .AddVertex({"C", 5, 5})
    .AddBidirectionalEdge({"A", 0, 0}, {"B", 10, 0}, 10.0)
    .AddBidirectionalEdge({"B", 10, 0}, {"C", 5, 5}, 7.07)
    .AddBidirectionalEdge({"A", 0, 0}, {"C", 5, 5}, 7.07)
    .Build();
```

### Observer Pattern for Graph Changes

```cpp
template<typename State, typename Transition>
class ObservableGraph {
public:
    class Observer {
    public:
        virtual ~Observer() = default;
        virtual void OnVertexAdded(const State& state) {}
        virtual void OnVertexRemoved(const State& state) {}
        virtual void OnEdgeAdded(const State& from, const State& to, const Transition& cost) {}
        virtual void OnEdgeRemoved(const State& from, const State& to) {}
    };
    
private:
    Graph<State, Transition> graph_;
    std::vector<std::weak_ptr<Observer>> observers_;
    
    void NotifyObservers(std::function<void(Observer*)> notification) {
        auto it = observers_.begin();
        while (it != observers_.end()) {
            if (auto observer = it->lock()) {
                notification(observer.get());
                ++it;
            } else {
                it = observers_.erase(it);  // Remove expired observers
            }
        }
    }
    
public:
    void AddObserver(std::shared_ptr<Observer> observer) {
        observers_.push_back(observer);
    }
    
    void AddVertex(const State& state) {
        graph_.AddVertex(state);
        NotifyObservers([&state](Observer* obs) { obs->OnVertexAdded(state); });
    }
    
    bool RemoveVertex(const State& state) {
        bool removed = graph_.RemoveVertex(state);
        if (removed) {
            NotifyObservers([&state](Observer* obs) { obs->OnVertexRemoved(state); });
        }
        return removed;
    }
    
    // Delegate other operations to internal graph
    const Graph<State, Transition>& GetGraph() const { return graph_; }
};
```

These advanced features provide the foundation for building sophisticated graph-based applications with optimal performance and maintainability.