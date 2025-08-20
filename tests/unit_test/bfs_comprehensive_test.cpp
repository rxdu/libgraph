/*
 * bfs_comprehensive_test.cpp
 *
 * Created on: Aug 2025
 * Description: Comprehensive tests for BFS algorithm edge cases and error conditions
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <vector>
#include <algorithm>

#include "graph/graph.hpp"
#include "graph/search/bfs.hpp"
#include "graph/search/search_context.hpp"

using namespace xmotion;

// Test state for BFS comprehensive testing
struct BFSTestState {
  int id;
  explicit BFSTestState(int i) : id(i) {}
  bool operator==(const BFSTestState& other) const { return id == other.id; }
  int GetId() const { return id; }
};

class BFSComprehensiveTest : public ::testing::Test {
protected:
  void SetUp() override {
    graph_ = std::make_unique<Graph<BFSTestState, double>>();
  }

  void TearDown() override {
    graph_.reset();
  }

  std::unique_ptr<Graph<BFSTestState, double>> graph_;
};

// Test BFS with nullptr graph
TEST_F(BFSComprehensiveTest, NullptrGraphHandling) {
  SearchContext<BFSTestState, double, DefaultIndexer<BFSTestState>> context;
  
  // Test with nullptr graph
  auto path = BFS::Search<BFSTestState, double, DefaultIndexer<BFSTestState>>(
      nullptr, context, BFSTestState(1), BFSTestState(2));
  
  EXPECT_TRUE(path.empty());
}

// Test BFS with empty graph
TEST_F(BFSComprehensiveTest, EmptyGraphHandling) {
  SearchContext<BFSTestState, double, DefaultIndexer<BFSTestState>> context;
  
  auto path = BFS::Search(graph_.get(), context, BFSTestState(1), BFSTestState(2));
  
  EXPECT_TRUE(path.empty());
}

// Test BFS with non-existent vertices
TEST_F(BFSComprehensiveTest, NonExistentVertices) {
  SearchContext<BFSTestState, double, DefaultIndexer<BFSTestState>> context;
  
  graph_->AddVertex(BFSTestState(2));
  
  // Non-existent start
  auto path1 = BFS::Search(graph_.get(), context, BFSTestState(1), BFSTestState(2));
  EXPECT_TRUE(path1.empty());
  
  // Non-existent goal
  auto path2 = BFS::Search(graph_.get(), context, BFSTestState(2), BFSTestState(1));
  EXPECT_TRUE(path2.empty());
}

// Test BFS with single vertex (start == goal)
TEST_F(BFSComprehensiveTest, SingleVertexPath) {
  SearchContext<BFSTestState, double, DefaultIndexer<BFSTestState>> context;
  
  graph_->AddVertex(BFSTestState(1));
  
  auto path = BFS::Search(graph_.get(), context, BFSTestState(1), BFSTestState(1));
  
  EXPECT_EQ(path.size(), 1);
  EXPECT_EQ(path[0].id, 1);
}

// Test BFS shortest path property
TEST_F(BFSComprehensiveTest, ShortestPathProperty) {
  SearchContext<BFSTestState, double, DefaultIndexer<BFSTestState>> context;
  
  /*
   * Create diamond structure with different path lengths:
   *     1
   *   /   \
   *  2     3
   *  |   / | \
   *  4  5  6  7
   *   \/    \/
   *    8    9
   *     \  /
   *      10
   */
  for (int i = 1; i <= 10; ++i) {
    graph_->AddVertex(BFSTestState(i));
  }
  
  // Upper paths
  graph_->AddEdge(BFSTestState(1), BFSTestState(2), 1.0);
  graph_->AddEdge(BFSTestState(1), BFSTestState(3), 1.0);
  
  // Left path (longer)
  graph_->AddEdge(BFSTestState(2), BFSTestState(4), 1.0);
  graph_->AddEdge(BFSTestState(4), BFSTestState(8), 1.0);
  graph_->AddEdge(BFSTestState(8), BFSTestState(10), 1.0);
  
  // Right path (shorter)
  graph_->AddEdge(BFSTestState(3), BFSTestState(5), 1.0);
  graph_->AddEdge(BFSTestState(3), BFSTestState(6), 1.0);
  graph_->AddEdge(BFSTestState(3), BFSTestState(7), 1.0);
  graph_->AddEdge(BFSTestState(5), BFSTestState(8), 1.0);
  graph_->AddEdge(BFSTestState(6), BFSTestState(9), 1.0);
  graph_->AddEdge(BFSTestState(7), BFSTestState(9), 1.0);
  graph_->AddEdge(BFSTestState(8), BFSTestState(10), 1.0);
  graph_->AddEdge(BFSTestState(9), BFSTestState(10), 1.0);
  
  auto path = BFS::Search(graph_.get(), context, BFSTestState(1), BFSTestState(10));
  
  EXPECT_FALSE(path.empty());
  EXPECT_EQ(path.front().id, 1);
  EXPECT_EQ(path.back().id, 10);
  
  // BFS should find shortest path (minimum number of edges)
  // Shortest should be: 1 -> 3 -> 6 -> 9 -> 10 (5 vertices, 4 edges)
  EXPECT_LE(path.size(), 5);
}

// Test BFS with cycles
TEST_F(BFSComprehensiveTest, GraphWithCycles) {
  SearchContext<BFSTestState, double, DefaultIndexer<BFSTestState>> context;
  
  // Create cycle: 1 -> 2 -> 3 -> 4 -> 1
  for (int i = 1; i <= 4; ++i) {
    graph_->AddVertex(BFSTestState(i));
  }
  graph_->AddEdge(BFSTestState(1), BFSTestState(2), 1.0);
  graph_->AddEdge(BFSTestState(2), BFSTestState(3), 1.0);
  graph_->AddEdge(BFSTestState(3), BFSTestState(4), 1.0);
  graph_->AddEdge(BFSTestState(4), BFSTestState(1), 1.0);
  
  // Add shortcut to test shortest path in presence of cycles
  graph_->AddEdge(BFSTestState(1), BFSTestState(4), 1.0);
  
  auto path = BFS::Search(graph_.get(), context, BFSTestState(1), BFSTestState(4));
  
  EXPECT_EQ(path.size(), 2); // Direct path: 1 -> 4
  EXPECT_EQ(path[0].id, 1);
  EXPECT_EQ(path[1].id, 4);
}

// Test BFS TraverseAll method
TEST_F(BFSComprehensiveTest, TraverseAllMethod) {
  SearchContext<BFSTestState, double, DefaultIndexer<BFSTestState>> context;
  
  // Create connected component
  for (int i = 1; i <= 5; ++i) {
    graph_->AddVertex(BFSTestState(i));
  }
  for (int i = 1; i < 5; ++i) {
    graph_->AddEdge(BFSTestState(i), BFSTestState(i + 1), 1.0);
  }
  
  // Test TraverseAll with valid start
  bool result = BFS::TraverseAll(graph_.get(), context, BFSTestState(1));
  EXPECT_TRUE(result);
  
  // Test TraverseAll with nullptr graph
  bool null_result = BFS::TraverseAll<BFSTestState, double, DefaultIndexer<BFSTestState>>(
      nullptr, context, BFSTestState(1));
  EXPECT_FALSE(null_result);
  
  // Test TraverseAll with non-existent start
  bool invalid_result = BFS::TraverseAll(graph_.get(), context, BFSTestState(10));
  EXPECT_FALSE(invalid_result);
}

// Test BFS IsReachable method
TEST_F(BFSComprehensiveTest, IsReachableMethod) {
  // Create path: 1 -> 2 -> 3
  graph_->AddVertex(BFSTestState(1));
  graph_->AddVertex(BFSTestState(2));
  graph_->AddVertex(BFSTestState(3));
  graph_->AddEdge(BFSTestState(1), BFSTestState(2), 1.0);
  graph_->AddEdge(BFSTestState(2), BFSTestState(3), 1.0);
  
  // Test reachable vertices
  EXPECT_TRUE(BFS::IsReachable(graph_.get(), BFSTestState(1), BFSTestState(3)));
  EXPECT_TRUE(BFS::IsReachable(graph_.get(), BFSTestState(1), BFSTestState(2)));
  
  // Test unreachable vertex
  graph_->AddVertex(BFSTestState(4)); // Isolated vertex
  EXPECT_FALSE(BFS::IsReachable(graph_.get(), BFSTestState(1), BFSTestState(4)));
  
  // Test same vertex
  EXPECT_TRUE(BFS::IsReachable(graph_.get(), BFSTestState(1), BFSTestState(1)));
}

// Test BFS with shared_ptr graph overloads
TEST_F(BFSComprehensiveTest, SharedPtrGraphOverloads) {
  auto shared_graph = std::make_shared<Graph<BFSTestState, double>>();
  SearchContext<BFSTestState, double, DefaultIndexer<BFSTestState>> context;
  
  shared_graph->AddVertex(BFSTestState(1));
  shared_graph->AddVertex(BFSTestState(2));
  shared_graph->AddEdge(BFSTestState(1), BFSTestState(2), 1.0);
  
  // Test shared_ptr overload with context
  auto path1 = BFS::Search(shared_graph, context, BFSTestState(1), BFSTestState(2));
  EXPECT_EQ(path1.size(), 2);
  
  // Test shared_ptr overload without context (legacy)
  auto path2 = BFS::Search(shared_graph, BFSTestState(1), BFSTestState(2));
  EXPECT_EQ(path2.size(), 2);
}

// Test BFS with disconnected components
TEST_F(BFSComprehensiveTest, DisconnectedComponents) {
  SearchContext<BFSTestState, double, DefaultIndexer<BFSTestState>> context;
  
  // Component 1: 1 <-> 2
  graph_->AddVertex(BFSTestState(1));
  graph_->AddVertex(BFSTestState(2));
  graph_->AddEdge(BFSTestState(1), BFSTestState(2), 1.0);
  
  // Component 2: 3 <-> 4 (disconnected)
  graph_->AddVertex(BFSTestState(3));
  graph_->AddVertex(BFSTestState(4));
  graph_->AddEdge(BFSTestState(3), BFSTestState(4), 1.0);
  
  // Try to find path between disconnected components
  auto path = BFS::Search(graph_.get(), context, BFSTestState(1), BFSTestState(3));
  
  EXPECT_TRUE(path.empty());
}

// Test BFS breadth-first exploration order
TEST_F(BFSComprehensiveTest, BreadthFirstExplorationOrder) {
  SearchContext<BFSTestState, double, DefaultIndexer<BFSTestState>> context;
  
  /*
   * Create tree structure:
   *       1
   *     /   \
   *    2     3
   *   / \   / \
   *  4   5 6   7
   */
  for (int i = 1; i <= 7; ++i) {
    graph_->AddVertex(BFSTestState(i));
  }
  
  graph_->AddEdge(BFSTestState(1), BFSTestState(2), 1.0);
  graph_->AddEdge(BFSTestState(1), BFSTestState(3), 1.0);
  graph_->AddEdge(BFSTestState(2), BFSTestState(4), 1.0);
  graph_->AddEdge(BFSTestState(2), BFSTestState(5), 1.0);
  graph_->AddEdge(BFSTestState(3), BFSTestState(6), 1.0);
  graph_->AddEdge(BFSTestState(3), BFSTestState(7), 1.0);
  
  // Test paths to different levels
  auto path_to_2 = BFS::Search(graph_.get(), context, BFSTestState(1), BFSTestState(2));
  EXPECT_EQ(path_to_2.size(), 2); // Level 1: 1 -> 2
  
  context.Reset();
  auto path_to_4 = BFS::Search(graph_.get(), context, BFSTestState(1), BFSTestState(4));
  EXPECT_EQ(path_to_4.size(), 3); // Level 2: 1 -> 2 -> 4
  
  context.Reset();
  auto path_to_7 = BFS::Search(graph_.get(), context, BFSTestState(1), BFSTestState(7));
  EXPECT_EQ(path_to_7.size(), 3); // Level 2: 1 -> 3 -> 7
}

// Test BFS with custom transition comparator
TEST_F(BFSComprehensiveTest, CustomTransitionComparator) {
  SearchContext<BFSTestState, int, DefaultIndexer<BFSTestState>> context;
  Graph<BFSTestState, int> int_graph;
  
  int_graph.AddVertex(BFSTestState(1));
  int_graph.AddVertex(BFSTestState(2));
  int_graph.AddVertex(BFSTestState(3));
  int_graph.AddEdge(BFSTestState(1), BFSTestState(2), 5);
  int_graph.AddEdge(BFSTestState(2), BFSTestState(3), 3);
  
  auto path = BFS::Search(&int_graph, context, BFSTestState(1), BFSTestState(3));
  
  EXPECT_EQ(path.size(), 3);
  EXPECT_EQ(path[0].id, 1);
  EXPECT_EQ(path[1].id, 2);
  EXPECT_EQ(path[2].id, 3);
}

// Test BFS with self-loop edges
TEST_F(BFSComprehensiveTest, SelfLoopHandling) {
  SearchContext<BFSTestState, double, DefaultIndexer<BFSTestState>> context;
  
  graph_->AddVertex(BFSTestState(1));
  graph_->AddVertex(BFSTestState(2));
  graph_->AddEdge(BFSTestState(1), BFSTestState(1), 1.0); // Self-loop
  graph_->AddEdge(BFSTestState(1), BFSTestState(2), 1.0);
  
  auto path = BFS::Search(graph_.get(), context, BFSTestState(1), BFSTestState(2));
  
  EXPECT_EQ(path.size(), 2);
  EXPECT_EQ(path[0].id, 1);
  EXPECT_EQ(path[1].id, 2);
}

// Test BFS large graph performance
TEST_F(BFSComprehensiveTest, LargeGraphPerformance) {
  SearchContext<BFSTestState, double, DefaultIndexer<BFSTestState>> context;
  
  // Create larger graph for performance testing
  const int GRAPH_SIZE = 200;
  
  // Create linear chain
  for (int i = 1; i <= GRAPH_SIZE; ++i) {
    graph_->AddVertex(BFSTestState(i));
    if (i > 1) {
      graph_->AddEdge(BFSTestState(i-1), BFSTestState(i), 1.0);
    }
  }
  
  auto start_time = std::chrono::high_resolution_clock::now();
  
  auto path = BFS::Search(graph_.get(), context, BFSTestState(1), BFSTestState(GRAPH_SIZE));
  
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  
  EXPECT_EQ(path.size(), GRAPH_SIZE);
  EXPECT_LT(duration.count(), 100); // Should complete in less than 100ms
}

// Test BFS with multiple equally short paths
TEST_F(BFSComprehensiveTest, MultipleEquallyShortPaths) {
  SearchContext<BFSTestState, double, DefaultIndexer<BFSTestState>> context;
  
  /*
   * Create structure with multiple 2-edge paths:
   *   2
   *  / \
   * 1   4
   *  \ /
   *   3
   */
  for (int i = 1; i <= 4; ++i) {
    graph_->AddVertex(BFSTestState(i));
  }
  
  graph_->AddEdge(BFSTestState(1), BFSTestState(2), 1.0);
  graph_->AddEdge(BFSTestState(1), BFSTestState(3), 1.0);
  graph_->AddEdge(BFSTestState(2), BFSTestState(4), 1.0);
  graph_->AddEdge(BFSTestState(3), BFSTestState(4), 1.0);
  
  auto path = BFS::Search(graph_.get(), context, BFSTestState(1), BFSTestState(4));
  
  // Should find one of the two equally short paths
  EXPECT_EQ(path.size(), 3);
  EXPECT_EQ(path[0].id, 1);
  EXPECT_EQ(path[2].id, 4);
  // Middle node should be either 2 or 3
  EXPECT_TRUE(path[1].id == 2 || path[1].id == 3);
}

// Test BFS context reuse
TEST_F(BFSComprehensiveTest, ContextReuseOptimization) {
  SearchContext<BFSTestState, double, DefaultIndexer<BFSTestState>> context;
  
  // Create graph for multiple searches
  for (int i = 1; i <= 10; ++i) {
    graph_->AddVertex(BFSTestState(i));
    if (i > 1) {
      graph_->AddEdge(BFSTestState(i-1), BFSTestState(i), 1.0);
    }
  }
  
  // Multiple searches with context reuse
  for (int target = 2; target <= 10; ++target) {
    context.Reset(); // Reset but keep allocated memory
    auto path = BFS::Search(graph_.get(), context, BFSTestState(1), BFSTestState(target));
    EXPECT_EQ(path.size(), target);
    EXPECT_EQ(path.front().id, 1);
    EXPECT_EQ(path.back().id, target);
  }
  
  // Verify context has accumulated search data
  EXPECT_GT(context.Size(), 0);
}