/*
 * error_condition_test.cpp
 *
 * Created on: [Current Date]
 * Description: Tests for error conditions and edge cases
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <memory>
#include <functional>

#include "gtest/gtest.h"

#include "graph/graph.hpp"
#include "graph/search/astar.hpp"
#include "graph/search/dijkstra.hpp"

using namespace xmotion;

struct TestState {
  TestState(int64_t id) : id_(id) {}
  int64_t id_;
};

class ErrorConditionTest : public testing::Test {
protected:
  void SetUp() override {
    graph.reset(new Graph<TestState>());
  }

  std::unique_ptr<Graph<TestState>> graph;
};

// ===== EMPTY GRAPH OPERATIONS =====

TEST_F(ErrorConditionTest, EmptyGraphOperations) {
  // Test operations on empty graph
  EXPECT_EQ(graph->GetTotalVertexNumber(), 0) << "Empty graph should have 0 vertices";
  EXPECT_EQ(graph->GetTotalEdgeNumber(), 0) << "Empty graph should have 0 edges";
  
  // Test iteration on empty graph
  EXPECT_EQ(graph->vertex_begin(), graph->vertex_end()) << "Empty graph iterators should be equal";
  
  int vertex_count = 0;
  for (auto it = graph->vertex_begin(); it != graph->vertex_end(); ++it) {
    vertex_count++;
  }
  EXPECT_EQ(vertex_count, 0) << "Should not iterate over vertices in empty graph";
}

TEST_F(ErrorConditionTest, FindVertexInEmptyGraph) {
  // Test finding vertices in empty graph
  auto result = graph->FindVertex(1);
  EXPECT_EQ(result, graph->vertex_end()) << "Should not find vertex in empty graph";
  
  auto result_by_state = graph->FindVertex(TestState(1));
  EXPECT_EQ(result_by_state, graph->vertex_end()) << "Should not find vertex by state in empty graph";
}

TEST_F(ErrorConditionTest, RemoveFromEmptyGraph) {
  // Test removing from empty graph (should not crash)
  EXPECT_NO_THROW(graph->RemoveVertex(1)) << "RemoveVertex on empty graph should not throw";
  EXPECT_NO_THROW(graph->RemoveVertex(TestState(1))) << "RemoveVertex by state on empty graph should not throw";
  EXPECT_NO_THROW(graph->RemoveEdge(TestState(1), TestState(2))) << "RemoveEdge on empty graph should not throw";
}

TEST_F(ErrorConditionTest, GetAllEdgesEmptyGraph) {
  // Test GetAllEdges on empty graph
  auto edges = graph->GetAllEdges();
  EXPECT_TRUE(edges.empty()) << "GetAllEdges should return empty vector for empty graph";
}

// ===== INVALID VERTEX OPERATIONS =====

TEST_F(ErrorConditionTest, InvalidVertexAccess) {
  // Add one vertex for testing
  graph->AddVertex(TestState(1));
  
  // Test accessing non-existent vertex
  auto invalid_vertex = graph->FindVertex(999);
  EXPECT_EQ(invalid_vertex, graph->vertex_end()) << "Should not find non-existent vertex";
  
  // Test accessing with negative ID
  auto negative_vertex = graph->FindVertex(-1);
  EXPECT_EQ(negative_vertex, graph->vertex_end()) << "Should not find vertex with negative ID";
}

TEST_F(ErrorConditionTest, DoubleVertexRemoval) {
  // Test removing same vertex twice
  auto vertex_it = graph->AddVertex(TestState(1));
  EXPECT_EQ(graph->GetTotalVertexNumber(), 1) << "Should have 1 vertex after adding";
  
  graph->RemoveVertex(1);
  EXPECT_EQ(graph->GetTotalVertexNumber(), 0) << "Should have 0 vertices after first removal";
  
  // Second removal should not crash
  EXPECT_NO_THROW(graph->RemoveVertex(1)) << "Double vertex removal should not throw";
  EXPECT_EQ(graph->GetTotalVertexNumber(), 0) << "Should still have 0 vertices after second removal";
}

TEST_F(ErrorConditionTest, InvalidEdgeOperations) {
  // Test edge operations with non-existent vertices
  EXPECT_NO_THROW(graph->RemoveEdge(TestState(999), TestState(888))) 
      << "Remove non-existent edge should not throw";
  
  // Add vertices and test invalid edge removal
  graph->AddVertex(TestState(1));
  graph->AddVertex(TestState(2));
  
  EXPECT_FALSE(graph->RemoveEdge(TestState(1), TestState(999))) 
      << "Should return false when removing edge to non-existent vertex";
  EXPECT_FALSE(graph->RemoveEdge(TestState(999), TestState(2))) 
      << "Should return false when removing edge from non-existent vertex";
}

TEST_F(ErrorConditionTest, DoubleEdgeRemoval) {
  // Test removing same edge twice
  graph->AddEdge(TestState(1), TestState(2), 1.0);
  EXPECT_EQ(graph->GetTotalEdgeNumber(), 1) << "Should have 1 edge after adding";
  
  bool first_removal = graph->RemoveEdge(TestState(1), TestState(2));
  EXPECT_TRUE(first_removal) << "First edge removal should succeed";
  EXPECT_EQ(graph->GetTotalEdgeNumber(), 0) << "Should have 0 edges after removal";
  
  bool second_removal = graph->RemoveEdge(TestState(1), TestState(2));
  EXPECT_FALSE(second_removal) << "Second edge removal should return false";
}

// ===== EDGE CASE SCENARIOS =====

TEST_F(ErrorConditionTest, SelfLoopEdges) {
  // Test self-loop edges (vertex connects to itself)
  graph->AddEdge(TestState(1), TestState(1), 5.0);
  
  EXPECT_EQ(graph->GetTotalVertexNumber(), 1) << "Should have 1 vertex for self-loop";
  EXPECT_EQ(graph->GetTotalEdgeNumber(), 1) << "Should have 1 edge for self-loop";
  
  auto vertex = graph->FindVertex(1);
  EXPECT_NE(vertex, graph->vertex_end()) << "Should find self-loop vertex";
  
  auto neighbors = vertex->GetNeighbours();
  EXPECT_EQ(neighbors.size(), 1) << "Self-loop vertex should have 1 neighbor (itself)";
  EXPECT_EQ(neighbors[0]->vertex_id, 1) << "Self-loop neighbor should be itself";
}

TEST_F(ErrorConditionTest, SingleVertexGraph) {
  // Test operations on single vertex graph
  auto vertex = graph->AddVertex(TestState(42));
  
  EXPECT_EQ(graph->GetTotalVertexNumber(), 1) << "Should have exactly 1 vertex";
  EXPECT_EQ(graph->GetTotalEdgeNumber(), 0) << "Single vertex should have 0 edges";
  
  auto neighbors = vertex->GetNeighbours();
  EXPECT_TRUE(neighbors.empty()) << "Single vertex should have no neighbors";
  
  EXPECT_FALSE(vertex->CheckNeighbour(42)) << "Single vertex should not be neighbor to itself";
  EXPECT_FALSE(vertex->CheckNeighbour(999)) << "Single vertex should not have any neighbors";
}

TEST_F(ErrorConditionTest, DisconnectedGraphComponents) {
  // Create disconnected graph components
  graph->AddEdge(TestState(1), TestState(2), 1.0);  // Component 1
  graph->AddEdge(TestState(3), TestState(4), 2.0);  // Component 2
  
  EXPECT_EQ(graph->GetTotalVertexNumber(), 4) << "Should have 4 vertices in disconnected graph";
  EXPECT_EQ(graph->GetTotalEdgeNumber(), 2) << "Should have 2 edges in disconnected graph";
  
  // Verify vertices can't reach each other across components
  auto vertex1 = graph->FindVertex(1);
  auto vertex3 = graph->FindVertex(3);
  
  EXPECT_FALSE(vertex1->CheckNeighbour(3)) << "Vertex 1 should not reach vertex 3";
  EXPECT_FALSE(vertex1->CheckNeighbour(4)) << "Vertex 1 should not reach vertex 4";
  EXPECT_FALSE(vertex3->CheckNeighbour(1)) << "Vertex 3 should not reach vertex 1";
  EXPECT_FALSE(vertex3->CheckNeighbour(2)) << "Vertex 3 should not reach vertex 2";
}

// ===== SEARCH ALGORITHM ERROR CONDITIONS =====

TEST_F(ErrorConditionTest, SearchOnEmptyGraph) {
  // Test search algorithms on empty graph
  std::function<double(TestState, TestState)> heuristic = [](TestState a, TestState b) -> double {
    return std::abs(static_cast<double>(a.id_ - b.id_));
  };
  
  auto astar_result = AStar::Search(graph.get(), 1, 2, heuristic);
  EXPECT_TRUE(astar_result.empty()) << "A* on empty graph should return empty path";
  
  auto dijkstra_result = Dijkstra::Search(graph.get(), 1, 2);
  EXPECT_TRUE(dijkstra_result.empty()) << "Dijkstra on empty graph should return empty path";
}

TEST_F(ErrorConditionTest, SearchWithNonExistentVertices) {
  // Add some vertices but search for non-existent ones
  graph->AddEdge(TestState(1), TestState(2), 1.0);
  
  std::function<double(TestState, TestState)> heuristic = [](TestState a, TestState b) -> double {
    return std::abs(static_cast<double>(a.id_ - b.id_));
  };
  
  auto astar_result = AStar::Search(graph.get(), 999, 888, heuristic);
  EXPECT_TRUE(astar_result.empty()) << "A* with non-existent vertices should return empty path";
  
  auto dijkstra_result = Dijkstra::Search(graph.get(), 999, 888);
  EXPECT_TRUE(dijkstra_result.empty()) << "Dijkstra with non-existent vertices should return empty path";
}

TEST_F(ErrorConditionTest, SearchSameStartAndGoal) {
  // Test search where start and goal are the same
  graph->AddVertex(TestState(1));
  
  std::function<double(TestState, TestState)> heuristic = [](TestState a, TestState b) -> double {
    return std::abs(static_cast<double>(a.id_ - b.id_));
  };
  
  auto astar_result = AStar::Search(graph.get(), 1, 1, heuristic);
  EXPECT_EQ(astar_result.size(), 1) << "A* with same start/goal should return single vertex path";
  EXPECT_EQ(astar_result[0].id_, 1) << "Path should contain the start/goal vertex";
  
  auto dijkstra_result = Dijkstra::Search(graph.get(), 1, 1);
  EXPECT_EQ(dijkstra_result.size(), 1) << "Dijkstra with same start/goal should return single vertex path";
  EXPECT_EQ(dijkstra_result[0].id_, 1) << "Path should contain the start/goal vertex";
}