/*
 * tuple_cost_demo.cpp
 * 
 * Demonstrates using std::tuple for automatic lexicographic cost comparison
 * in network routing scenarios. Shows how tuple-based costs provide built-in
 * hierarchical comparison without manual operator overloading.
 * 
 * Example use case:
 * - Network routing with priority levels, distance, and latency optimization
 */

#include <iostream>
#include <tuple>
#include "graph/graph.hpp"
#include "graph/search/dijkstra.hpp"

namespace xmotion {

/**
 * @brief Using std::tuple for automatic lexicographic comparison
 * 
 * std::tuple provides built-in lexicographic comparison operators,
 * making it easier to implement multi-criteria costs without
 * manually overloading all comparison operators.
 */
struct TupleCost {
    std::tuple<int, double, double> values;  // (priority_level, distance, time)
    
    TupleCost(int priority = 0, double distance = 0, double time = 0)
        : values(priority, distance, time) {}
    
    // Create a "maximum" value for initialization
    static TupleCost max() {
        return TupleCost(
            std::numeric_limits<int>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max()
        );
    }
    
    // Tuple automatically provides lexicographic comparison
    bool operator<(const TupleCost& other) const {
        return values < other.values;
    }
    
    bool operator>(const TupleCost& other) const {
        return values > other.values;
    }
    
    bool operator<=(const TupleCost& other) const {
        return values <= other.values;
    }
    
    bool operator>=(const TupleCost& other) const {
        return values >= other.values;
    }
    
    bool operator==(const TupleCost& other) const {
        return values == other.values;
    }
    
    bool operator!=(const TupleCost& other) const {
        return values != other.values;
    }
    
    TupleCost operator+(const TupleCost& other) const {
        return TupleCost(
            std::get<0>(values) + std::get<0>(other.values),
            std::get<1>(values) + std::get<1>(other.values),
            std::get<2>(values) + std::get<2>(other.values)
        );
    }
    
    TupleCost& operator+=(const TupleCost& other) {
        std::get<0>(values) += std::get<0>(other.values);
        std::get<1>(values) += std::get<1>(other.values);
        std::get<2>(values) += std::get<2>(other.values);
        return *this;
    }
    
    friend std::ostream& operator<<(std::ostream& os, const TupleCost& cost) {
        os << "(" << std::get<0>(cost.values) << ", " 
           << std::get<1>(cost.values) << ", " 
           << std::get<2>(cost.values) << ")";
        return os;
    }
};

// Custom indexer for std::string 
struct StringIndexer {
    int64_t operator()(const std::string& str) const {
        return std::hash<std::string>()(str);
    }
};

} // namespace xmotion

// Specialize CostTraits for TupleCost
namespace xmotion {

template<>
struct CostTraits<TupleCost> {
    static TupleCost infinity() {
        return TupleCost::max();
    }
};

} // namespace xmotion

namespace xmotion {

void DemoTupleCost() {
    std::cout << "=== Network Routing with Tuple-based Cost ===\n\n";
    std::cout << "Cost priority: (1) priority level, (2) distance, (3) latency\n\n";
    
    // Create a network graph with tuple costs
    Graph<std::string, TupleCost, StringIndexer> network;
    
    // Add nodes
    network.AddVertex("Router_A");
    network.AddVertex("Router_B");
    network.AddVertex("Router_C");
    network.AddVertex("Router_D");
    network.AddVertex("Server");
    
    // Add connections: (priority_level, distance_km, latency_ms)
    // Lower priority number = higher priority path
    
    // Premium path: A -> Server (high priority, long distance, low latency)
    network.AddEdge("Router_A", "Server", TupleCost(1, 100, 5));
    
    // Standard path: A -> B -> Server (medium priority, medium distance, medium latency)
    network.AddEdge("Router_A", "Router_B", TupleCost(2, 30, 10));
    network.AddEdge("Router_B", "Server", TupleCost(2, 40, 12));
    
    // Budget path: A -> C -> D -> Server (low priority, short distance, high latency)
    network.AddEdge("Router_A", "Router_C", TupleCost(3, 20, 15));
    network.AddEdge("Router_C", "Router_D", TupleCost(3, 15, 20));
    network.AddEdge("Router_D", "Server", TupleCost(3, 10, 25));
    
    // Alternative standard path with better latency
    network.AddEdge("Router_B", "Router_D", TupleCost(2, 25, 8));
    
    std::cout << "Network paths:\n";
    std::cout << "- Premium: A -> Server (priority=1, distance=100km, latency=5ms)\n";
    std::cout << "- Standard: A -> B -> Server (priority=2, distance=70km, latency=22ms)\n";
    std::cout << "- Enhanced: A -> B -> D -> Server (priority=2, distance=65km, latency=43ms)\n";
    std::cout << "- Budget: A -> C -> D -> Server (priority=3, distance=45km, latency=60ms)\n\n";
    
    // Demonstrate step-by-step path finding
    std::cout << "=== Path Analysis ===\n";
    
    // Find optimal path
    auto result = Dijkstra::Search(&network, std::string("Router_A"), std::string("Server"));
    
    if (!result.empty()) {
        auto path = result;
        std::cout << "Optimal path found:\n";
        for (size_t i = 0; i < path.size(); ++i) {
            std::cout << path[i];
            if (i < path.size() - 1) std::cout << " -> ";
        }
        std::cout << "\n\n";
        
        // Calculate total cost for the path
        TupleCost total_cost(0, 0, 0);
        for (size_t i = 0; i < path.size() - 1; ++i) {
            // Note: In a real implementation, you'd get the actual edge cost
            // This is simplified for demonstration
        }
        
        std::cout << "Why this path was chosen:\n";
        std::cout << "1. Priority level is considered first (lower number = higher priority)\n";
        std::cout << "2. Among same priority paths, distance is considered second\n";
        std::cout << "3. Finally, latency is used as tie-breaker\n\n";
        
        std::cout << "This demonstrates lexicographic ordering:\n";
        std::cout << "- Premium path (priority=1) beats all others regardless of distance/latency\n";
        std::cout << "- If no premium path existed, standard paths (priority=2) would compete\n";
        std::cout << "- Budget paths (priority=3) only chosen if no better priority available\n";
        
    } else {
        std::cout << "No path found. This might indicate an issue with cost initialization.\n";
    }
}

void DemoTupleComparison() {
    std::cout << "\n\n=== Tuple Comparison Demonstration ===\n\n";
    
    // Create different cost combinations to show lexicographic ordering
    TupleCost cost1(1, 100, 50);  // High priority, high distance, medium latency
    TupleCost cost2(2, 10, 5);    // Low priority, low distance, low latency
    TupleCost cost3(1, 200, 100); // High priority, very high distance, high latency
    TupleCost cost4(1, 100, 25);  // High priority, high distance, low latency
    
    std::cout << "Comparing costs:\n";
    std::cout << "Cost1: " << cost1 << " (priority=1, distance=100, latency=50)\n";
    std::cout << "Cost2: " << cost2 << " (priority=2, distance=10, latency=5)\n";
    std::cout << "Cost3: " << cost3 << " (priority=1, distance=200, latency=100)\n";
    std::cout << "Cost4: " << cost4 << " (priority=1, distance=100, latency=25)\n\n";
    
    std::cout << "Lexicographic comparison results:\n";
    std::cout << "Cost1 < Cost2: " << (cost1 < cost2) << " (priority 1 vs 2 - true)\n";
    std::cout << "Cost1 < Cost3: " << (cost1 < cost3) << " (same priority, distance 100 vs 200 - true)\n";
    std::cout << "Cost1 < Cost4: " << (cost1 < cost4) << " (same priority & distance, latency 50 vs 25 - false)\n";
    std::cout << "Cost4 < Cost1: " << (cost4 < cost1) << " (same priority & distance, latency 25 vs 50 - true)\n\n";
    
    std::cout << "Key insight: Each criterion is only considered if all higher-priority criteria are equal.\n";
}

} // namespace xmotion

int main() {
    xmotion::DemoTupleCost();
    xmotion::DemoTupleComparison();
    
    std::cout << "\n=== Tuple-based Cost Benefits ===\n";
    std::cout << "1. std::tuple provides automatic lexicographic comparison\n";
    std::cout << "2. No need to manually implement all comparison operators\n";
    std::cout << "3. Easy to extend with additional criteria (just add to tuple)\n";
    std::cout << "4. Type-safe with compile-time checking\n";
    std::cout << "5. Clear semantic meaning through tuple element positions\n";
    std::cout << "6. Works seamlessly with STL algorithms and containers\n";
    
    return 0;
}