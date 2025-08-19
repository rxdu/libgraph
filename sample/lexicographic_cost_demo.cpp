/*
 * lexicographic_cost_demo.cpp
 * 
 * Demonstrates using lexicographic (multi-criteria) costs for graph edges.
 * Costs are compared hierarchically: first by primary criterion, then secondary, etc.
 * 
 * Example use cases:
 * - Multi-objective path planning (minimize distance, then time, then fuel)
 * - Network routing (minimize hops, then latency, then bandwidth usage)
 * - Transportation planning (minimize transfers, then time, then cost)
 */

#include <iostream>
#include <tuple>
#include "graph/graph.hpp"
#include "graph/search/dijkstra.hpp"
#include "graph/search/astar.hpp"

namespace xmotion {

/**
 * @brief Lexicographic cost with multiple criteria compared hierarchically
 * 
 * Comparison order:
 * 1. Primary cost (e.g., number of transfers in transit)
 * 2. Secondary cost (e.g., total travel time)
 * 3. Tertiary cost (e.g., monetary cost)
 */
struct LexicographicCost {
    double primary;    // Most important criterion
    double secondary;  // Second priority
    double tertiary;   // Third priority
    
    LexicographicCost(double p = 0, double s = 0, double t = 0) 
        : primary(p), secondary(s), tertiary(t) {}
    
    // Create a "maximum" value for initialization
    static LexicographicCost max() {
        return LexicographicCost(
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(), 
            std::numeric_limits<double>::max()
        );
    }
    
    // Lexicographic comparison: compare primary first, then secondary, then tertiary
    bool operator<(const LexicographicCost& other) const {
        if (primary != other.primary) return primary < other.primary;
        if (secondary != other.secondary) return secondary < other.secondary;
        return tertiary < other.tertiary;
    }
    
    bool operator>(const LexicographicCost& other) const {
        return other < *this;
    }
    
    bool operator<=(const LexicographicCost& other) const {
        return !(*this > other);
    }
    
    bool operator>=(const LexicographicCost& other) const {
        return !(*this < other);
    }
    
    bool operator==(const LexicographicCost& other) const {
        return primary == other.primary && 
               secondary == other.secondary && 
               tertiary == other.tertiary;
    }
    
    bool operator!=(const LexicographicCost& other) const {
        return !(*this == other);
    }
    
    // Addition for path cost accumulation
    LexicographicCost operator+(const LexicographicCost& other) const {
        return LexicographicCost(
            primary + other.primary,
            secondary + other.secondary,
            tertiary + other.tertiary
        );
    }
    
    LexicographicCost& operator+=(const LexicographicCost& other) {
        primary += other.primary;
        secondary += other.secondary;
        tertiary += other.tertiary;
        return *this;
    }
    
    // For A* heuristic compatibility
    LexicographicCost operator-(const LexicographicCost& other) const {
        return LexicographicCost(
            primary - other.primary,
            secondary - other.secondary,
            tertiary - other.tertiary
        );
    }
    
    // Print cost for debugging
    friend std::ostream& operator<<(std::ostream& os, const LexicographicCost& cost) {
        os << "(" << cost.primary << ", " << cost.secondary << ", " << cost.tertiary << ")";
        return os;
    }
};

/**
 * @brief Alternative: Using std::tuple for automatic lexicographic comparison
 * 
 * std::tuple provides built-in lexicographic comparison operators
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

} // namespace xmotion

// Specialize CostTraits for our custom cost types
namespace xmotion {

template<>
struct CostTraits<LexicographicCost> {
    static LexicographicCost infinity() {
        return LexicographicCost::max();
    }
};

template<>
struct CostTraits<TupleCost> {
    static TupleCost infinity() {
        return TupleCost::max();
    }
};

} // namespace xmotion

namespace xmotion {

// Custom indexer for std::string 
struct StringIndexer {
    int64_t operator()(const std::string& str) const {
        return std::hash<std::string>()(str);
    }
};

// Example: Transit system with transfers, time, and monetary cost
struct TransitStation {
    std::string name;
    int zone;
    
    TransitStation(const std::string& n = "", int z = 0) : name(n), zone(z) {}
    
    bool operator==(const TransitStation& other) const {
        return name == other.name;
    }
    
    int64_t GetId() const {
        return std::hash<std::string>()(name);
    }
};

void DemoLexicographicCost() {
    std::cout << "=== Transit Network with Lexicographic Cost ===\n\n";
    std::cout << "Cost priority: (1) transfers, (2) time, (3) price\n\n";
    
    // Create a transit network graph
    Graph<TransitStation, LexicographicCost> transit_network;
    
    // Add stations
    TransitStation station_a("Station A", 1);
    TransitStation station_b("Station B", 1);
    TransitStation station_c("Station C", 2);
    TransitStation station_d("Station D", 2);
    TransitStation station_e("Station E", 3);
    
    transit_network.AddVertex(station_a);
    transit_network.AddVertex(station_b);
    transit_network.AddVertex(station_c);
    transit_network.AddVertex(station_d);
    transit_network.AddVertex(station_e);
    
    // Add connections with costs: (transfers, time_minutes, price_dollars)
    // Direct express line A->E (no transfer, longer time, higher price)
    transit_network.AddEdge(station_a, station_e, LexicographicCost(0, 45, 8.50));
    
    // Route through B and C (1 transfer, medium time, medium price)
    transit_network.AddEdge(station_a, station_b, LexicographicCost(0, 10, 2.00));
    transit_network.AddEdge(station_b, station_c, LexicographicCost(1, 15, 2.50));  // Transfer here
    transit_network.AddEdge(station_c, station_e, LexicographicCost(0, 12, 2.00));
    
    // Route through D (1 transfer, shortest time, medium price)
    transit_network.AddEdge(station_a, station_d, LexicographicCost(0, 8, 3.00));
    transit_network.AddEdge(station_d, station_e, LexicographicCost(1, 8, 3.00));  // Transfer here
    
    // Alternative from B to E (no additional transfer but longer)
    transit_network.AddEdge(station_b, station_e, LexicographicCost(0, 35, 5.00));
    
    std::cout << "Network structure:\n";
    std::cout << "- Direct express: A -> E (no transfer, 45 min, $8.50)\n";
    std::cout << "- Via B-C: A -> B -> C -> E (1 transfer, 37 min, $6.50)\n";
    std::cout << "- Via D: A -> D -> E (1 transfer, 16 min, $6.00)\n";
    std::cout << "- Alternative: A -> B -> E (no transfer, 45 min, $7.00)\n\n";
    
    // Find optimal path using Dijkstra
    auto result = Dijkstra::Search(&transit_network, station_a, station_e);
    
    if (!result.empty()) {
        auto path = result;
        std::cout << "Optimal path found:\n";
        for (size_t i = 0; i < path.size(); ++i) {
            std::cout << path[i].name;
            if (i < path.size() - 1) std::cout << " -> ";
        }
        std::cout << "\n\n";
        
        std::cout << "Path demonstrates lexicographic cost optimization:\n";
        std::cout << "Minimizes transfers first, then time, then price.\n";
    } else {
        std::cout << "No path found. This might indicate an issue with cost initialization.\n";
    }
}
} // namespace xmotion

int main() {
    xmotion::DemoLexicographicCost();

    std::cout << "\n=== Key Insights ===\n";
    std::cout << "1. Lexicographic costs enable multi-criteria optimization\n";
    std::cout << "2. Priority order matters: primary criterion dominates decisions\n";
    std::cout << "3. Works seamlessly with Dijkstra/A* due to proper operator overloading\n";
    std::cout << "4. std::tuple provides automatic lexicographic comparison\n";
    std::cout << "5. Useful for real-world problems with multiple competing objectives\n";
    
    return 0;
}