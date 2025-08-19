/*
 * test_dfs.cpp
 *
 * Created on: Aug 2025
 * Description: Test cases for Depth-First Search implementation
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <vector>
#include <unordered_set>

#include "graph/graph.hpp"
#include "graph/search/dfs.hpp"

using namespace xmotion;

// Simple state for testing
struct SimpleState {
    int x, y;
    int id;
    
    SimpleState(int x = 0, int y = 0) : x(x), y(y), id(x * 1000 + y) {}
    
    bool operator==(const SimpleState& other) const {
        return x == other.x && y == other.y;
    }
    
    int GetId() const { return id; }
};

// Simple indexer
struct SimpleStateIndexer {
    int64_t operator()(const SimpleState& state) const {
        return state.GetId();
    }
};

void TestDFSBasicPath() {
    std::cout << "=== Testing DFS Basic Path Finding ===" << std::endl;
    
    // Create a simple graph: 0 -> 1 -> 2 -> 3
    Graph<SimpleState, double, SimpleStateIndexer> graph;
    
    SimpleState s0(0, 0), s1(1, 0), s2(2, 0), s3(3, 0);
    
    graph.AddVertex(s0);
    graph.AddVertex(s1);
    graph.AddVertex(s2);
    graph.AddVertex(s3);
    
    graph.AddEdge(s0, s1, 1.0);
    graph.AddEdge(s1, s2, 1.0);
    graph.AddEdge(s2, s3, 1.0);
    
    // Test DFS path finding
    auto path = DFS::Search(&graph, s0.GetId(), s3.GetId());
    
    std::cout << "Path from (0,0) to (3,0): ";
    for (const auto& state : path) {
        std::cout << "(" << state.x << "," << state.y << ") ";
    }
    std::cout << std::endl;
    
    if (path.size() == 4 && path[0] == s0 && path[3] == s3) {
        std::cout << "✓ DFS basic path test PASSED" << std::endl;
    } else {
        std::cout << "✗ DFS basic path test FAILED" << std::endl;
    }
}

void TestDFSWithBranching() {
    std::cout << "\n=== Testing DFS with Branching Graph ===" << std::endl;
    
    // Create a branching graph:
    //     1
    //    / \
    //   0   3
    //    \ /
    //     2
    Graph<SimpleState, double, SimpleStateIndexer> graph;
    
    SimpleState s0(0, 0), s1(1, 0), s2(0, 1), s3(1, 1);
    
    graph.AddVertex(s0);
    graph.AddVertex(s1);
    graph.AddVertex(s2);
    graph.AddVertex(s3);
    
    graph.AddEdge(s0, s1, 1.0);
    graph.AddEdge(s0, s2, 1.0);
    graph.AddEdge(s1, s3, 1.0);
    graph.AddEdge(s2, s3, 1.0);
    
    // Test DFS finds a path (may not be shortest)
    auto path = DFS::Search(&graph, s0.GetId(), s3.GetId());
    
    std::cout << "Path from (0,0) to (1,1): ";
    for (const auto& state : path) {
        std::cout << "(" << state.x << "," << state.y << ") ";
    }
    std::cout << std::endl;
    
    if (!path.empty() && path[0] == s0 && path.back() == s3) {
        std::cout << "✓ DFS branching graph test PASSED" << std::endl;
    } else {
        std::cout << "✗ DFS branching graph test FAILED" << std::endl;
    }
}

void TestDFSNoPath() {
    std::cout << "\n=== Testing DFS with No Path ===" << std::endl;
    
    // Create disconnected graph: 0 -> 1    2 -> 3
    Graph<SimpleState, double, SimpleStateIndexer> graph;
    
    SimpleState s0(0, 0), s1(1, 0), s2(2, 0), s3(3, 0);
    
    graph.AddVertex(s0);
    graph.AddVertex(s1);
    graph.AddVertex(s2);
    graph.AddVertex(s3);
    
    graph.AddEdge(s0, s1, 1.0);
    graph.AddEdge(s2, s3, 1.0);
    
    // Test DFS with no path
    auto path = DFS::Search(&graph, s0.GetId(), s3.GetId());
    
    if (path.empty()) {
        std::cout << "✓ DFS no path test PASSED" << std::endl;
    } else {
        std::cout << "✗ DFS no path test FAILED - found path when none should exist" << std::endl;
    }
}

void TestDFSReachability() {
    std::cout << "\n=== Testing DFS Reachability Check ===" << std::endl;
    
    Graph<SimpleState, double, SimpleStateIndexer> graph;
    
    SimpleState s0(0, 0), s1(1, 0), s2(2, 0), s3(3, 0);
    
    graph.AddVertex(s0);
    graph.AddVertex(s1);
    graph.AddVertex(s2);
    graph.AddVertex(s3);
    
    graph.AddEdge(s0, s1, 1.0);
    graph.AddEdge(s1, s2, 1.0);
    // s3 is disconnected
    
    bool reachable12 = DFS::IsReachable(&graph, s0.GetId(), s2.GetId());
    bool reachable13 = DFS::IsReachable(&graph, s0.GetId(), s3.GetId());
    
    if (reachable12 && !reachable13) {
        std::cout << "✓ DFS reachability test PASSED" << std::endl;
    } else {
        std::cout << "✗ DFS reachability test FAILED" << std::endl;
        std::cout << "  s0->s2 reachable: " << reachable12 << " (should be true)" << std::endl;
        std::cout << "  s0->s3 reachable: " << reachable13 << " (should be false)" << std::endl;
    }
}

void TestDFSTraverseAll() {
    std::cout << "\n=== Testing DFS Traverse All ===" << std::endl;
    
    Graph<SimpleState, double, SimpleStateIndexer> graph;
    
    SimpleState s0(0, 0), s1(1, 0), s2(2, 0), s3(3, 0);
    
    graph.AddVertex(s0);
    graph.AddVertex(s1);
    graph.AddVertex(s2);
    graph.AddVertex(s3);
    
    graph.AddEdge(s0, s1, 1.0);
    graph.AddEdge(s1, s2, 1.0);
    graph.AddEdge(s0, s3, 1.0);  // Alternative path
    
    SearchContext<SimpleState, double, SimpleStateIndexer> context;
    bool success = DFS::TraverseAll(&graph, context, s0.GetId());
    
    // Check that all reachable vertices were visited
    bool visited_0 = context.HasSearchInfo(s0.GetId());
    bool visited_1 = context.HasSearchInfo(s1.GetId());
    bool visited_2 = context.HasSearchInfo(s2.GetId());
    bool visited_3 = context.HasSearchInfo(s3.GetId());
    
    if (success && visited_0 && visited_1 && visited_2 && visited_3) {
        std::cout << "✓ DFS traverse all test PASSED" << std::endl;
    } else {
        std::cout << "✗ DFS traverse all test FAILED" << std::endl;
        std::cout << "  Success: " << success << std::endl;
        std::cout << "  Visited: s0=" << visited_0 << " s1=" << visited_1 
                  << " s2=" << visited_2 << " s3=" << visited_3 << std::endl;
    }
}

void TestDFSWithSearchContext() {
    std::cout << "\n=== Testing DFS with External SearchContext ===" << std::endl;
    
    Graph<SimpleState, double, SimpleStateIndexer> graph;
    
    SimpleState s0(0, 0), s1(1, 0), s2(2, 0);
    
    graph.AddVertex(s0);
    graph.AddVertex(s1);
    graph.AddVertex(s2);
    
    graph.AddEdge(s0, s1, 1.0);
    graph.AddEdge(s1, s2, 1.0);
    
    SearchContext<SimpleState, double, SimpleStateIndexer> context;
    auto path = DFS::Search(&graph, context, s0.GetId(), s2.GetId());
    
    // Check that search context contains information
    bool has_start_info = context.HasSearchInfo(s0.GetId());
    bool has_goal_info = context.HasSearchInfo(s2.GetId());
    
    if (!path.empty() && has_start_info && has_goal_info) {
        std::cout << "✓ DFS with SearchContext test PASSED" << std::endl;
    } else {
        std::cout << "✗ DFS with SearchContext test FAILED" << std::endl;
    }
}

void TestDFSThreadSafety() {
    std::cout << "\n=== Testing DFS Thread Safety ===" << std::endl;
    
    Graph<SimpleState, double, SimpleStateIndexer> graph;
    
    SimpleState s0(0, 0), s1(1, 0), s2(2, 0), s3(3, 0);
    
    graph.AddVertex(s0);
    graph.AddVertex(s1);
    graph.AddVertex(s2);
    graph.AddVertex(s3);
    
    graph.AddEdge(s0, s1, 1.0);
    graph.AddEdge(s1, s2, 1.0);
    graph.AddEdge(s2, s3, 1.0);
    
    // Multiple contexts for concurrent searches
    SearchContext<SimpleState, double, SimpleStateIndexer> context1;
    SearchContext<SimpleState, double, SimpleStateIndexer> context2;
    
    auto path1 = DFS::Search(&graph, context1, s0.GetId(), s3.GetId());
    auto path2 = DFS::Search(&graph, context2, s0.GetId(), s2.GetId());
    
    if (!path1.empty() && !path2.empty() && 
        path1.size() == 4 && path2.size() == 3) {
        std::cout << "✓ DFS thread safety test PASSED" << std::endl;
    } else {
        std::cout << "✗ DFS thread safety test FAILED" << std::endl;
    }
}

void TestDFSCustomCostType() {
    std::cout << "\n=== Testing DFS with Custom Cost Type ===" << std::endl;
    
    Graph<SimpleState, int, SimpleStateIndexer> graph;
    
    SimpleState s0(0, 0), s1(1, 0), s2(2, 0);
    
    graph.AddVertex(s0);
    graph.AddVertex(s1);
    graph.AddVertex(s2);
    
    graph.AddEdge(s0, s1, 1);
    graph.AddEdge(s1, s2, 2);
    
    SearchContext<SimpleState, int, SimpleStateIndexer> context;
    auto path = DFS::Search<SimpleState, int, SimpleStateIndexer>(
        &graph, context, s0.GetId(), s2.GetId());
    
    if (!path.empty() && path.size() == 3) {
        std::cout << "✓ DFS custom cost type test PASSED" << std::endl;
    } else {
        std::cout << "✗ DFS custom cost type test FAILED" << std::endl;
    }
}

void TestDFSSharedPtr() {
    std::cout << "\n=== Testing DFS with shared_ptr Graph ===" << std::endl;
    
    auto graph = std::make_shared<Graph<SimpleState, double, SimpleStateIndexer>>();
    
    SimpleState s0(0, 0), s1(1, 0);
    
    graph->AddVertex(s0);
    graph->AddVertex(s1);
    graph->AddEdge(s0, s1, 1.0);
    
    SearchContext<SimpleState, double, SimpleStateIndexer> context;
    auto path = DFS::Search(graph, context, s0.GetId(), s1.GetId());
    
    if (!path.empty() && path.size() == 2) {
        std::cout << "✓ DFS shared_ptr test PASSED" << std::endl;
    } else {
        std::cout << "✗ DFS shared_ptr test FAILED" << std::endl;
    }
}

int main() {
    std::cout << "Running DFS Algorithm Tests..." << std::endl;
    std::cout << "====================================" << std::endl;
    
    TestDFSBasicPath();
    TestDFSWithBranching();
    TestDFSNoPath();
    TestDFSReachability();
    TestDFSTraverseAll();
    TestDFSWithSearchContext();
    TestDFSThreadSafety();
    TestDFSCustomCostType();
    TestDFSSharedPtr();
    
    std::cout << "\n====================================" << std::endl;
    std::cout << "DFS Algorithm Tests Completed" << std::endl;
    
    return 0;
}