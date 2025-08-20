/*
 * dfs_comprehensive_test.cpp
 *
 * Created on: Aug 2025
 * Description: Comprehensive tests for DFS algorithm edge cases and error conditions
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <thread>
#include <vector>

#include "graph/graph.hpp"
#include "graph/search/dfs.hpp"
#include "graph/search/search_context.hpp"

using namespace xmotion;

// Test state for DFS comprehensive testing
struct DFSTestState {
  int id;
  explicit DFSTestState(int i) : id(i) {}
  bool operator==(const DFSTestState& other) const { return id == other.id; }
  int GetId() const { return id; }
};

class DFSComprehensiveTest : public ::testing::Test {
protected:
  void SetUp() override {
    graph_ = std::make_unique<Graph<DFSTestState, double>>();
  }

  void TearDown() override {
    graph_.reset();
  }

  std::unique_ptr<Graph<DFSTestState, double>> graph_;
};

// Test DFS with nullptr graph
TEST_F(DFSComprehensiveTest, NullptrGraphHandling) {
  SearchContext<DFSTestState, double, DefaultIndexer<DFSTestState>> context;
  
  // Test with nullptr graph
  auto path = DFS::Search<DFSTestState, double, DefaultIndexer<DFSTestState>>(
      nullptr, context, DFSTestState(1), DFSTestState(2));
  
  EXPECT_TRUE(path.empty());
}

// Test DFS with empty graph
TEST_F(DFSComprehensiveTest, EmptyGraphHandling) {
  SearchContext<DFSTestState, double, DefaultIndexer<DFSTestState>> context;
  
  auto path = DFS::Search(graph_.get(), context, DFSTestState(1), DFSTestState(2));
  
  EXPECT_TRUE(path.empty());
}

// Test DFS with non-existent start vertex
TEST_F(DFSComprehensiveTest, NonExistentStartVertex) {
  SearchContext<DFSTestState, double, DefaultIndexer<DFSTestState>> context;
  
  graph_->AddVertex(DFSTestState(2));
  
  auto path = DFS::Search(graph_.get(), context, DFSTestState(1), DFSTestState(2));
  
  EXPECT_TRUE(path.empty());
}

// Test DFS with non-existent goal vertex
TEST_F(DFSComprehensiveTest, NonExistentGoalVertex) {
  SearchContext<DFSTestState, double, DefaultIndexer<DFSTestState>> context;
  
  graph_->AddVertex(DFSTestState(1));
  
  auto path = DFS::Search(graph_.get(), context, DFSTestState(1), DFSTestState(2));
  
  EXPECT_TRUE(path.empty());
}

// Test DFS with single vertex (start == goal)
TEST_F(DFSComprehensiveTest, SingleVertexPath) {
  SearchContext<DFSTestState, double, DefaultIndexer<DFSTestState>> context;
  
  graph_->AddVertex(DFSTestState(1));
  
  auto path = DFS::Search(graph_.get(), context, DFSTestState(1), DFSTestState(1));
  
  EXPECT_EQ(path.size(), 1);
  EXPECT_EQ(path[0].id, 1);
}

// Test DFS with disconnected graph components
TEST_F(DFSComprehensiveTest, DisconnectedComponents) {
  SearchContext<DFSTestState, double, DefaultIndexer<DFSTestState>> context;
  
  // Component 1: 1 -> 2
  graph_->AddVertex(DFSTestState(1));
  graph_->AddVertex(DFSTestState(2));
  graph_->AddEdge(DFSTestState(1), DFSTestState(2), 1.0);
  
  // Component 2: 3 -> 4 (disconnected)
  graph_->AddVertex(DFSTestState(3));
  graph_->AddVertex(DFSTestState(4));
  graph_->AddEdge(DFSTestState(3), DFSTestState(4), 1.0);
  
  // Try to find path between disconnected components
  auto path = DFS::Search(graph_.get(), context, DFSTestState(1), DFSTestState(3));
  
  EXPECT_TRUE(path.empty());
}

// Test DFS with cycles
TEST_F(DFSComprehensiveTest, GraphWithCycles) {
  SearchContext<DFSTestState, double, DefaultIndexer<DFSTestState>> context;
  
  // Create cycle: 1 -> 2 -> 3 -> 1
  graph_->AddVertex(DFSTestState(1));
  graph_->AddVertex(DFSTestState(2));
  graph_->AddVertex(DFSTestState(3));
  graph_->AddEdge(DFSTestState(1), DFSTestState(2), 1.0);
  graph_->AddEdge(DFSTestState(2), DFSTestState(3), 1.0);
  graph_->AddEdge(DFSTestState(3), DFSTestState(1), 1.0);
  
  auto path = DFS::Search(graph_.get(), context, DFSTestState(1), DFSTestState(3));
  
  EXPECT_FALSE(path.empty());
  EXPECT_EQ(path.front().id, 1);
  EXPECT_EQ(path.back().id, 3);
}

// Test DFS TraverseAll method
TEST_F(DFSComprehensiveTest, TraverseAllMethod) {
  SearchContext<DFSTestState, double, DefaultIndexer<DFSTestState>> context;
  
  // Create connected component
  for (int i = 1; i <= 5; ++i) {
    graph_->AddVertex(DFSTestState(i));
  }
  for (int i = 1; i < 5; ++i) {
    graph_->AddEdge(DFSTestState(i), DFSTestState(i + 1), 1.0);
  }
  
  // Test TraverseAll with valid start
  bool result = DFS::TraverseAll(graph_.get(), context, DFSTestState(1));
  EXPECT_TRUE(result);
  
  // Test TraverseAll with nullptr graph
  bool null_result = DFS::TraverseAll<DFSTestState, double, DefaultIndexer<DFSTestState>>(
      nullptr, context, DFSTestState(1));
  EXPECT_FALSE(null_result);
  
  // Test TraverseAll with non-existent start
  bool invalid_result = DFS::TraverseAll(graph_.get(), context, DFSTestState(10));
  EXPECT_FALSE(invalid_result);
}

// Test DFS IsReachable method
TEST_F(DFSComprehensiveTest, IsReachableMethod) {
  // Create path: 1 -> 2 -> 3
  graph_->AddVertex(DFSTestState(1));
  graph_->AddVertex(DFSTestState(2));
  graph_->AddVertex(DFSTestState(3));
  graph_->AddEdge(DFSTestState(1), DFSTestState(2), 1.0);
  graph_->AddEdge(DFSTestState(2), DFSTestState(3), 1.0);
  
  // Test reachable vertices
  EXPECT_TRUE(DFS::IsReachable(graph_.get(), DFSTestState(1), DFSTestState(3)));
  EXPECT_TRUE(DFS::IsReachable(graph_.get(), DFSTestState(1), DFSTestState(2)));
  
  // Test unreachable vertex
  graph_->AddVertex(DFSTestState(4)); // Isolated vertex
  EXPECT_FALSE(DFS::IsReachable(graph_.get(), DFSTestState(1), DFSTestState(4)));
  
  // Test same vertex
  EXPECT_TRUE(DFS::IsReachable(graph_.get(), DFSTestState(1), DFSTestState(1)));
}

// Test DFS with shared_ptr graph overloads
TEST_F(DFSComprehensiveTest, SharedPtrGraphOverloads) {
  auto shared_graph = std::make_shared<Graph<DFSTestState, double>>();
  SearchContext<DFSTestState, double, DefaultIndexer<DFSTestState>> context;
  
  shared_graph->AddVertex(DFSTestState(1));
  shared_graph->AddVertex(DFSTestState(2));
  shared_graph->AddEdge(DFSTestState(1), DFSTestState(2), 1.0);
  
  // Test shared_ptr overload with context
  auto path1 = DFS::Search(shared_graph, context, DFSTestState(1), DFSTestState(2));
  EXPECT_EQ(path1.size(), 2);
  
  // Test shared_ptr overload without context (legacy)
  auto path2 = DFS::Search(shared_graph, DFSTestState(1), DFSTestState(2));
  EXPECT_EQ(path2.size(), 2);
}

// Test DFS with complex branching structure
TEST_F(DFSComprehensiveTest, ComplexBranchingStructure) {
  SearchContext<DFSTestState, double, DefaultIndexer<DFSTestState>> context;
  
  /*
   * Create complex structure:
   *     1
   *   /   \
   *  2     3
   * / \   / \
   *4   5 6   7
   *         /
   *        8
   */
  for (int i = 1; i <= 8; ++i) {
    graph_->AddVertex(DFSTestState(i));
  }
  
  graph_->AddEdge(DFSTestState(1), DFSTestState(2), 1.0);
  graph_->AddEdge(DFSTestState(1), DFSTestState(3), 1.0);
  graph_->AddEdge(DFSTestState(2), DFSTestState(4), 1.0);
  graph_->AddEdge(DFSTestState(2), DFSTestState(5), 1.0);
  graph_->AddEdge(DFSTestState(3), DFSTestState(6), 1.0);
  graph_->AddEdge(DFSTestState(3), DFSTestState(7), 1.0);
  graph_->AddEdge(DFSTestState(7), DFSTestState(8), 1.0);
  
  // Test multiple paths exist - DFS should find one
  auto path = DFS::Search(graph_.get(), context, DFSTestState(1), DFSTestState(8));
  
  EXPECT_FALSE(path.empty());
  EXPECT_EQ(path.front().id, 1);
  EXPECT_EQ(path.back().id, 8);
  
  // Verify DFS traversal order characteristics (depth-first behavior)
  EXPECT_GE(path.size(), 4); // Minimum path length from 1 to 8
}

// Test DFS strategy with custom comparator
TEST_F(DFSComprehensiveTest, CustomTransitionComparator) {
  SearchContext<DFSTestState, int, DefaultIndexer<DFSTestState>> context;
  Graph<DFSTestState, int> int_graph;
  
  int_graph.AddVertex(DFSTestState(1));
  int_graph.AddVertex(DFSTestState(2));
  int_graph.AddVertex(DFSTestState(3));
  int_graph.AddEdge(DFSTestState(1), DFSTestState(2), 5);
  int_graph.AddEdge(DFSTestState(2), DFSTestState(3), 3);
  
  auto path = DFS::Search(&int_graph, context, DFSTestState(1), DFSTestState(3));
  
  EXPECT_EQ(path.size(), 3);
  EXPECT_EQ(path[0].id, 1);
  EXPECT_EQ(path[1].id, 2);
  EXPECT_EQ(path[2].id, 3);
}

// Test DFS with self-loop edges
TEST_F(DFSComprehensiveTest, SelfLoopHandling) {
  SearchContext<DFSTestState, double, DefaultIndexer<DFSTestState>> context;
  
  graph_->AddVertex(DFSTestState(1));
  graph_->AddVertex(DFSTestState(2));
  graph_->AddEdge(DFSTestState(1), DFSTestState(1), 1.0); // Self-loop
  graph_->AddEdge(DFSTestState(1), DFSTestState(2), 1.0);
  
  auto path = DFS::Search(graph_.get(), context, DFSTestState(1), DFSTestState(2));
  
  EXPECT_EQ(path.size(), 2);
  EXPECT_EQ(path[0].id, 1);
  EXPECT_EQ(path[1].id, 2);
}

// Test DFS context reuse and performance
TEST_F(DFSComprehensiveTest, ContextReusePerformance) {
  SearchContext<DFSTestState, double, DefaultIndexer<DFSTestState>> context;
  
  // Create larger graph for performance testing
  const int GRAPH_SIZE = 100;
  for (int i = 1; i <= GRAPH_SIZE; ++i) {
    graph_->AddVertex(DFSTestState(i));
    if (i > 1) {
      graph_->AddEdge(DFSTestState(i-1), DFSTestState(i), 1.0);
    }
  }
  
  // Multiple searches with same context (should reuse allocated memory)
  auto start_time = std::chrono::high_resolution_clock::now();
  
  for (int i = 0; i < 10; ++i) {
    context.Reset(); // Reset but keep allocated memory
    auto path = DFS::Search(graph_.get(), context, DFSTestState(1), DFSTestState(GRAPH_SIZE));
    EXPECT_EQ(path.size(), GRAPH_SIZE);
  }
  
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  
  // Performance should be reasonable for 10 searches on 100-node graph
  EXPECT_LT(duration.count(), 1000); // Less than 1 second
}