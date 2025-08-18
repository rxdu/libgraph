/*
 * generic_cost_framework_test.cpp
 *
 * Created on: Aug 2025
 * Description: Comprehensive tests for the generic cost type framework
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <tuple>
#include <string>
#include <limits>

#include "graph/graph.hpp"
#include "graph/search/search_context.hpp"
#include "graph/search/dijkstra.hpp"
#include "graph/search/astar.hpp"
#include "graph/search/bfs.hpp"
#include "graph/search/dfs.hpp"

using namespace xmotion;

// Test custom cost types
struct Priority {
    int level;
    double weight;
    
    Priority(int l = 0, double w = 0.0) : level(l), weight(w) {}
    
    bool operator<(const Priority& other) const {
        if (level != other.level) return level < other.level;
        return weight < other.weight;
    }
    
    bool operator>(const Priority& other) const { return other < *this; }
    bool operator==(const Priority& other) const {
        return level == other.level && weight == other.weight;
    }
    
    Priority operator+(const Priority& other) const {
        return Priority(level + other.level, weight + other.weight);
    }
    
    static Priority max() {
        return Priority(std::numeric_limits<int>::max(), 
                       std::numeric_limits<double>::max());
    }
};

struct TupleCost {
    std::tuple<int, double> values;
    
    TupleCost(int priority = 0, double cost = 0.0) : values(priority, cost) {}
    
    bool operator<(const TupleCost& other) const { return values < other.values; }
    bool operator>(const TupleCost& other) const { return values > other.values; }
    bool operator==(const TupleCost& other) const { return values == other.values; }
    
    TupleCost operator+(const TupleCost& other) const {
        return TupleCost(std::get<0>(values) + std::get<0>(other.values),
                        std::get<1>(values) + std::get<1>(other.values));
    }
    
    static TupleCost max() {
        return TupleCost(std::numeric_limits<int>::max(),
                        std::numeric_limits<double>::max());
    }
};

// CostTraits specializations
namespace xmotion {

template<>
struct CostTraits<Priority> {
    static Priority infinity() { return Priority::max(); }
};

template<>
struct CostTraits<TupleCost> {
    static TupleCost infinity() { return TupleCost::max(); }
};

} // namespace xmotion

// Test state
struct SimpleNode {
    int id;
    SimpleNode(int i = 0) : id(i) {}
    int64_t GetId() const { return id; }
    bool operator==(const SimpleNode& other) const { return id == other.id; }
};

class GenericCostFrameworkTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test graph: 0 -> 1 -> 2 -> 3
        //                     \       /
        //                      \     /
        //                       \   /
        //                        > 4
        for (int i = 0; i <= 4; ++i) {
            nodes.emplace_back(i);
        }
    }
    
    std::vector<SimpleNode> nodes;
};

TEST_F(GenericCostFrameworkTest, CostTraitsSpecialization) {
    // Test that CostTraits works for custom types
    auto priority_inf = CostTraits<Priority>::infinity();
    EXPECT_EQ(priority_inf.level, std::numeric_limits<int>::max());
    EXPECT_EQ(priority_inf.weight, std::numeric_limits<double>::max());
    
    auto tuple_inf = CostTraits<TupleCost>::infinity();
    EXPECT_EQ(std::get<0>(tuple_inf.values), std::numeric_limits<int>::max());
    EXPECT_EQ(std::get<1>(tuple_inf.values), std::numeric_limits<double>::max());
    
    // Test that it still works for built-in types
    auto double_inf = CostTraits<double>::infinity();
    EXPECT_EQ(double_inf, std::numeric_limits<double>::max());
}

TEST_F(GenericCostFrameworkTest, SearchContextWithCustomCosts) {
    SearchContext<SimpleNode, Priority, DefaultIndexer<SimpleNode>> context;
    
    auto& info = context.GetSearchInfo(1);
    
    // Test generic cost methods
    Priority cost(2, 5.5);
    info.SetGCost(cost);
    
    auto retrieved = info.GetGCost<Priority>();
    EXPECT_EQ(retrieved.level, 2);
    EXPECT_EQ(retrieved.weight, 5.5);
    
    // Test initialization with CostTraits
    auto& info2 = context.GetSearchInfo(2);
    auto initial_cost = info2.GetGCost<Priority>();
    EXPECT_EQ(initial_cost.level, std::numeric_limits<int>::max());
    EXPECT_EQ(initial_cost.weight, std::numeric_limits<double>::max());
}

TEST_F(GenericCostFrameworkTest, DijkstraWithCustomComparator) {
    Graph<SimpleNode, Priority> graph;
    
    // Build graph
    for (const auto& node : nodes) {
        graph.AddVertex(node);
    }
    
    // Add edges with priority costs
    graph.AddEdge(nodes[0], nodes[1], Priority(1, 10.0));  // High priority, high cost
    graph.AddEdge(nodes[0], nodes[4], Priority(3, 5.0));   // Low priority, low cost
    graph.AddEdge(nodes[1], nodes[2], Priority(1, 5.0));   // High priority, low cost
    graph.AddEdge(nodes[4], nodes[2], Priority(2, 8.0));   // Medium priority, medium cost
    
    // Test with default comparator (std::less<Priority>)
    auto path = Dijkstra::Search(&graph, nodes[0], nodes[2]);
    
    EXPECT_FALSE(path.empty());
    EXPECT_EQ(path.front().id, 0);
    EXPECT_EQ(path.back().id, 2);
    
    // Should prefer high priority path even if more costly
    // Path should be 0 -> 1 -> 2 (priority 1) rather than 0 -> 4 -> 2 (priority 2+)
    std::vector<int> expected_path = {0, 1, 2};
    std::vector<int> actual_path;
    for (const auto& node : path) {
        actual_path.push_back(node.id);
    }
    
    EXPECT_EQ(actual_path, expected_path);
}

TEST_F(GenericCostFrameworkTest, AStarWithCustomCosts) {
    Graph<SimpleNode, TupleCost> graph;
    
    for (const auto& node : nodes) {
        graph.AddVertex(node);
    }
    
    // Add edges with tuple costs (priority, distance)
    graph.AddEdge(nodes[0], nodes[1], TupleCost(1, 10.0));
    graph.AddEdge(nodes[0], nodes[4], TupleCost(2, 5.0));
    graph.AddEdge(nodes[1], nodes[2], TupleCost(1, 5.0));
    graph.AddEdge(nodes[4], nodes[2], TupleCost(1, 8.0));
    
    // Simple heuristic
    auto heuristic = [](const SimpleNode& from, const SimpleNode& to) {
        return TupleCost(0, std::abs(from.id - to.id));
    };
    
    auto path = AStar::Search(&graph, nodes[0], nodes[2], heuristic);
    
    EXPECT_FALSE(path.empty());
    EXPECT_EQ(path.front().id, 0);
    EXPECT_EQ(path.back().id, 2);
}

TEST_F(GenericCostFrameworkTest, AllAlgorithmsWithSameCustomCost) {
    Graph<SimpleNode, Priority> graph;
    
    for (const auto& node : nodes) {
        graph.AddVertex(node);
    }
    
    // Simple linear path for consistency testing
    graph.AddEdge(nodes[0], nodes[1], Priority(1, 1.0));
    graph.AddEdge(nodes[1], nodes[2], Priority(1, 1.0));
    graph.AddEdge(nodes[2], nodes[3], Priority(1, 1.0));
    
    // Test that all algorithms can handle the same custom cost type
    auto dijkstra_path = Dijkstra::Search(&graph, nodes[0], nodes[3]);
    EXPECT_EQ(dijkstra_path.size(), 4);
    
    auto heuristic = [](const SimpleNode& from, const SimpleNode& to) {
        return Priority(0, std::abs(from.id - to.id));
    };
    auto astar_path = AStar::Search(&graph, nodes[0], nodes[3], heuristic);
    EXPECT_EQ(astar_path.size(), 4);
    
    auto bfs_path = BFS::Search(&graph, nodes[0], nodes[3]);
    EXPECT_EQ(bfs_path.size(), 4);
    
    auto dfs_path = DFS::Search(&graph, nodes[0], nodes[3]);
    EXPECT_FALSE(dfs_path.empty());  // DFS may find different path
    
    // All should find a path from 0 to 3
    EXPECT_EQ(dijkstra_path.front().id, 0);
    EXPECT_EQ(dijkstra_path.back().id, 3);
    EXPECT_EQ(astar_path.front().id, 0);
    EXPECT_EQ(astar_path.back().id, 3);
    EXPECT_EQ(bfs_path.front().id, 0);
    EXPECT_EQ(bfs_path.back().id, 3);
    EXPECT_EQ(dfs_path.front().id, 0);
    EXPECT_EQ(dfs_path.back().id, 3);
}

TEST_F(GenericCostFrameworkTest, ThreadSafetyWithCustomCosts) {
    Graph<SimpleNode, Priority> graph;
    
    for (const auto& node : nodes) {
        graph.AddVertex(node);
    }
    
    graph.AddEdge(nodes[0], nodes[1], Priority(1, 1.0));
    graph.AddEdge(nodes[1], nodes[2], Priority(1, 1.0));
    
    // Test concurrent searches with custom costs
    SearchContext<SimpleNode, Priority, DefaultIndexer<SimpleNode>> context1, context2;
    
    auto path1 = Dijkstra::Search(&graph, context1, nodes[0], nodes[2]);
    auto path2 = Dijkstra::Search(&graph, context2, nodes[0], nodes[2]);
    
    EXPECT_EQ(path1.size(), path2.size());
    EXPECT_EQ(path1.size(), 3);
    
    // Verify contexts remain independent
    EXPECT_TRUE(context1.HasSearchInfo(0));
    EXPECT_TRUE(context2.HasSearchInfo(0));
    
    // Context data should be independent
    auto& info1 = context1.GetSearchInfo(0);
    auto& info2 = context2.GetSearchInfo(0);
    
    info1.SetAttribute("test_marker", std::string("context1"));
    info2.SetAttribute("test_marker", std::string("context2"));
    
    EXPECT_EQ(info1.GetAttribute<std::string>("test_marker"), "context1");
    EXPECT_EQ(info2.GetAttribute<std::string>("test_marker"), "context2");
}

TEST_F(GenericCostFrameworkTest, CustomComparatorValidation) {
    // Test that the framework properly uses custom comparators
    Graph<SimpleNode, Priority> graph;
    
    graph.AddVertex(nodes[0]);
    graph.AddVertex(nodes[1]);
    graph.AddVertex(nodes[2]);
    
    // Edge with high priority (should be preferred)
    graph.AddEdge(nodes[0], nodes[1], Priority(1, 100.0));
    
    // Edge with low priority but lower cost (should not be preferred)
    graph.AddEdge(nodes[0], nodes[2], Priority(5, 1.0));
    
    // Both lead to same destination through different intermediate nodes
    graph.AddEdge(nodes[1], nodes[3], Priority(1, 1.0));
    graph.AddEdge(nodes[2], nodes[3], Priority(1, 1.0));
    
    graph.AddVertex(nodes[3]);
    
    auto path = Dijkstra::Search(&graph, nodes[0], nodes[3]);
    
    EXPECT_FALSE(path.empty());
    
    // Should go through node 1 (high priority) rather than node 2 (low priority)
    // even though node 2 has lower numeric cost
    EXPECT_EQ(path[1].id, 1);  // Second node should be 1, not 2
}

TEST_F(GenericCostFrameworkTest, BackwardCompatibility) {
    // Ensure existing double-based code still works
    Graph<SimpleNode, double> graph;
    
    for (const auto& node : nodes) {
        graph.AddVertex(node);
    }
    
    graph.AddEdge(nodes[0], nodes[1], 1.0);
    graph.AddEdge(nodes[1], nodes[2], 2.0);
    
    // Old API should still work
    auto path = Dijkstra::Search(&graph, nodes[0], nodes[2]);
    EXPECT_EQ(path.size(), 3);
    
    // SearchContext with double should work
    SearchContext<SimpleNode, double, DefaultIndexer<SimpleNode>> context;
    auto threadsafe_path = Dijkstra::Search(&graph, context, nodes[0], nodes[2]);
    EXPECT_EQ(threadsafe_path.size(), 3);
    
    // Legacy property access should work
    auto& info = context.GetSearchInfo(0);
    info.g_cost = 5.0;  // Property-based assignment
    EXPECT_EQ(info.g_cost, 5.0);  // Property-based access
    EXPECT_EQ(info.GetGCost<double>(), 5.0);  // Modern API
}

TEST_F(GenericCostFrameworkTest, EdgeCasesAndErrorConditions) {
    Graph<SimpleNode, Priority> graph;
    graph.AddVertex(nodes[0]);
    graph.AddVertex(nodes[1]);
    
    // No edges - should find no path
    auto no_path = Dijkstra::Search(&graph, nodes[0], nodes[1]);
    EXPECT_TRUE(no_path.empty());
    
    // Same start and goal
    auto same_path = Dijkstra::Search(&graph, nodes[0], nodes[0]);
    EXPECT_EQ(same_path.size(), 1);
    EXPECT_EQ(same_path[0].id, 0);
    
    // Test with CostTraits initialization
    SearchContext<SimpleNode, Priority, DefaultIndexer<SimpleNode>> context;
    auto& info = context.GetSearchInfo(999);
    auto default_cost = info.GetGCost<Priority>();
    
    // Should be initialized with CostTraits::infinity()
    EXPECT_EQ(default_cost.level, std::numeric_limits<int>::max());
    EXPECT_EQ(default_cost.weight, std::numeric_limits<double>::max());
}