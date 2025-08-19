/*
 * advanced_graph_operations_test.cpp
 *
 * Created on: Aug 2025
 * Description: Advanced tests for graph operations to improve graph_impl.hpp coverage
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <algorithm>
#include <unordered_set>

#include "graph/graph.hpp"

using namespace xmotion;

// Test state for advanced operations
struct AdvancedTestState {
  int id;
  std::string name;
  
  AdvancedTestState(int i, const std::string& n = "") : id(i), name(n) {}
  bool operator==(const AdvancedTestState& other) const { return id == other.id; }
  int GetId() const { return id; }
};

class AdvancedGraphOperationsTest : public ::testing::Test {
protected:
  void SetUp() override {
    graph_ = std::make_unique<Graph<AdvancedTestState, double>>();
  }

  void TearDown() override {
    graph_.reset();
  }

  std::unique_ptr<Graph<AdvancedTestState, double>> graph_;
};

// Test complex vertex removal scenarios
TEST_F(AdvancedGraphOperationsTest, ComplexVertexRemovalScenarios) {
  // Create complex graph with multiple edges
  for (int i = 1; i <= 6; ++i) {
    graph_->AddVertex(AdvancedTestState(i, "vertex_" + std::to_string(i)));
  }
  
  // Create hub topology: vertex 3 connected to all others
  for (int i = 1; i <= 6; ++i) {
    if (i != 3) {
      graph_->AddEdge(AdvancedTestState(3), AdvancedTestState(i), static_cast<double>(i));
      graph_->AddEdge(AdvancedTestState(i), AdvancedTestState(3), static_cast<double>(i * 2));
    }
  }
  
  // Add some additional edges
  graph_->AddEdge(AdvancedTestState(1), AdvancedTestState(2), 1.5);
  graph_->AddEdge(AdvancedTestState(4), AdvancedTestState(5), 2.5);
  
  size_t initial_vertex_count = graph_->GetVertexCount();
  size_t initial_edge_count = graph_->GetEdgeCount();
  
  // Remove hub vertex (should remove many edges)
  bool removed = graph_->RemoveVertex(AdvancedTestState(3));
  
  EXPECT_TRUE(removed);
  EXPECT_EQ(graph_->GetVertexCount(), initial_vertex_count - 1);
  EXPECT_LT(graph_->GetEdgeCount(), initial_edge_count); // Should have removed many edges
  
  // Verify vertex 3 no longer exists
  EXPECT_EQ(graph_->FindVertex(AdvancedTestState(3)), graph_->vertex_end());
  
  // Verify remaining vertices still exist
  for (int i = 1; i <= 6; ++i) {
    if (i != 3) {
      EXPECT_NE(graph_->FindVertex(AdvancedTestState(i)), graph_->vertex_end());
    }
  }
}

// Test edge removal edge cases
TEST_F(AdvancedGraphOperationsTest, EdgeRemovalEdgeCases) {
  graph_->AddVertex(AdvancedTestState(1));
  graph_->AddVertex(AdvancedTestState(2));
  graph_->AddVertex(AdvancedTestState(3));
  
  // Add multiple edges between same vertices with different weights
  graph_->AddEdge(AdvancedTestState(1), AdvancedTestState(2), 1.0);
  graph_->AddEdge(AdvancedTestState(1), AdvancedTestState(2), 2.0);
  graph_->AddEdge(AdvancedTestState(2), AdvancedTestState(1), 3.0);
  
  // Self-loop
  graph_->AddEdge(AdvancedTestState(1), AdvancedTestState(1), 5.0);
  
  size_t initial_edge_count = graph_->GetEdgeCount();
  
  // Remove specific edge
  bool removed1 = graph_->RemoveEdge(AdvancedTestState(1), AdvancedTestState(2), 1.0);
  EXPECT_TRUE(removed1);
  EXPECT_EQ(graph_->GetEdgeCount(), initial_edge_count - 1);
  
  // Try to remove non-existent edge
  bool removed2 = graph_->RemoveEdge(AdvancedTestState(1), AdvancedTestState(3), 1.0);
  EXPECT_FALSE(removed2);
  EXPECT_EQ(graph_->GetEdgeCount(), initial_edge_count - 1);
  
  // Remove self-loop
  bool removed3 = graph_->RemoveEdge(AdvancedTestState(1), AdvancedTestState(1), 5.0);
  EXPECT_TRUE(removed3);
  EXPECT_EQ(graph_->GetEdgeCount(), initial_edge_count - 2);
}

// Test ClearVertexEdges functionality
TEST_F(AdvancedGraphOperationsTest, ClearVertexEdgesComprehensive) {
  for (int i = 1; i <= 5; ++i) {
    graph_->AddVertex(AdvancedTestState(i));
  }
  
  // Create star topology with vertex 1 at center
  for (int i = 2; i <= 5; ++i) {
    graph_->AddEdge(AdvancedTestState(1), AdvancedTestState(i), static_cast<double>(i));
    graph_->AddEdge(AdvancedTestState(i), AdvancedTestState(1), static_cast<double>(i * 2));
  }
  
  // Add self-loop
  graph_->AddEdge(AdvancedTestState(1), AdvancedTestState(1), 10.0);
  
  size_t initial_edge_count = graph_->GetEdgeCount();
  
  // Clear all edges for vertex 1
  graph_->ClearVertexEdges(AdvancedTestState(1));
  
  // Vertex should still exist
  EXPECT_NE(graph_->FindVertex(AdvancedTestState(1)), graph_->vertex_end());
  
  // All edges involving vertex 1 should be gone
  auto vertex1_it = graph_->FindVertex(AdvancedTestState(1));
  EXPECT_EQ(vertex1_it->edges_to.size(), 0);
  
  // Check that edges from other vertices to vertex 1 are also removed
  for (int i = 2; i <= 5; ++i) {
    auto vertex_it = graph_->FindVertex(AdvancedTestState(i));
    EXPECT_NE(vertex_it, graph_->vertex_end());
    
    // Should not have edges to vertex 1
    bool has_edge_to_1 = false;
    for (const auto& edge : vertex_it->edges_to) {
      if (edge.dst->state.id == 1) {
        has_edge_to_1 = true;
        break;
      }
    }
    EXPECT_FALSE(has_edge_to_1);
  }
  
  // Edge count should be significantly reduced
  EXPECT_LT(graph_->GetEdgeCount(), initial_edge_count);
}

// Test GetAllEdges with complex topology
TEST_F(AdvancedGraphOperationsTest, GetAllEdgesComplex) {
  // Create complex topology
  for (int i = 1; i <= 4; ++i) {
    graph_->AddVertex(AdvancedTestState(i));
  }
  
  // Add various types of edges
  graph_->AddEdge(AdvancedTestState(1), AdvancedTestState(2), 1.5);
  graph_->AddEdge(AdvancedTestState(2), AdvancedTestState(3), 2.5);
  graph_->AddEdge(AdvancedTestState(3), AdvancedTestState(4), 3.5);
  graph_->AddEdge(AdvancedTestState(4), AdvancedTestState(1), 4.5);
  graph_->AddEdge(AdvancedTestState(1), AdvancedTestState(1), 5.5); // Self-loop
  graph_->AddEdge(AdvancedTestState(2), AdvancedTestState(4), 6.5); // Skip edge
  
  auto all_edges = graph_->GetAllEdges();
  
  EXPECT_EQ(all_edges.size(), 6);
  
  // Verify edge properties
  std::unordered_set<int> src_ids, dst_ids;
  std::vector<double> weights;
  
  for (const auto& edge_info : all_edges) {
    src_ids.insert(edge_info.src_vertex->state.id);
    dst_ids.insert(edge_info.dst_vertex->state.id);
    weights.push_back(edge_info.edge.cost);
  }
  
  // Should have edges from all vertices
  EXPECT_EQ(src_ids.size(), 4);
  
  // Should have expected weight values
  std::sort(weights.begin(), weights.end());
  EXPECT_DOUBLE_EQ(weights[0], 1.5);
  EXPECT_DOUBLE_EQ(weights[5], 6.5);
}

// Test complex graph construction patterns
TEST_F(AdvancedGraphOperationsTest, ComplexGraphConstructionPatterns) {
  // Test complete graph construction (K_5)
  const int n = 5;
  
  for (int i = 1; i <= n; ++i) {
    graph_->AddVertex(AdvancedTestState(i));
  }
  
  // Add edges for complete graph
  int edge_count = 0;
  for (int i = 1; i <= n; ++i) {
    for (int j = 1; j <= n; ++j) {
      if (i != j) {
        graph_->AddEdge(AdvancedTestState(i), AdvancedTestState(j), 
                       static_cast<double>(i * 10 + j));
        edge_count++;
      }
    }
  }
  
  EXPECT_EQ(graph_->GetVertexCount(), n);
  EXPECT_EQ(graph_->GetEdgeCount(), edge_count);
  
  // Verify every vertex has edges to all others
  for (int i = 1; i <= n; ++i) {
    auto vertex_it = graph_->FindVertex(AdvancedTestState(i));
    EXPECT_NE(vertex_it, graph_->vertex_end());
    EXPECT_EQ(vertex_it->edges_to.size(), n - 1); // n-1 outgoing edges
  }
}

// Test vertex iteration with modifications
TEST_F(AdvancedGraphOperationsTest, VertexIterationWithModifications) {
  // Add initial vertices
  for (int i = 1; i <= 10; ++i) {
    graph_->AddVertex(AdvancedTestState(i));
  }
  
  std::vector<int> vertex_ids;
  
  // Collect vertex IDs
  for (auto it = graph_->vertex_begin(); it != graph_->vertex_end(); ++it) {
    vertex_ids.push_back(it->state.id);
  }
  
  EXPECT_EQ(vertex_ids.size(), 10);
  std::sort(vertex_ids.begin(), vertex_ids.end());
  
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(vertex_ids[i], i + 1);
  }
  
  // Test const iteration
  const auto& const_graph = *graph_;
  std::vector<int> const_vertex_ids;
  
  for (auto it = const_graph.vertex_begin(); it != const_graph.vertex_end(); ++it) {
    const_vertex_ids.push_back(it->state.id);
  }
  
  EXPECT_EQ(const_vertex_ids.size(), vertex_ids.size());
}

// Test edge finding with complex scenarios
TEST_F(AdvancedGraphOperationsTest, EdgeFindingComplexScenarios) {
  graph_->AddVertex(AdvancedTestState(1));
  graph_->AddVertex(AdvancedTestState(2));
  graph_->AddVertex(AdvancedTestState(3));
  
  // Add multiple edges with different weights
  graph_->AddEdge(AdvancedTestState(1), AdvancedTestState(2), 1.0);
  graph_->AddEdge(AdvancedTestState(1), AdvancedTestState(2), 2.0);
  graph_->AddEdge(AdvancedTestState(1), AdvancedTestState(3), 3.0);
  
  auto vertex1_it = graph_->FindVertex(AdvancedTestState(1));
  EXPECT_NE(vertex1_it, graph_->vertex_end());
  
  // Find specific edges
  auto edge_it1 = vertex1_it->FindEdge(AdvancedTestState(2));
  EXPECT_NE(edge_it1, vertex1_it->edges_to.end());
  
  // Should find first edge to vertex 2
  bool found_edge_to_2 = false;
  for (const auto& edge : vertex1_it->edges_to) {
    if (edge.dst->state.id == 2) {
      found_edge_to_2 = true;
      break;
    }
  }
  EXPECT_TRUE(found_edge_to_2);
  
  // Test edge finding with non-existent target
  auto edge_it_missing = vertex1_it->FindEdge(AdvancedTestState(10));
  EXPECT_EQ(edge_it_missing, vertex1_it->edges_to.end());
}

// Test neighbor operations
TEST_F(AdvancedGraphOperationsTest, NeighborOperations) {
  for (int i = 1; i <= 6; ++i) {
    graph_->AddVertex(AdvancedTestState(i));
  }
  
  // Create adjacencies: 1 -> {2,3,4}, 2 -> {1,5}, 3 -> {6}
  graph_->AddEdge(AdvancedTestState(1), AdvancedTestState(2), 1.0);
  graph_->AddEdge(AdvancedTestState(1), AdvancedTestState(3), 1.0);
  graph_->AddEdge(AdvancedTestState(1), AdvancedTestState(4), 1.0);
  graph_->AddEdge(AdvancedTestState(2), AdvancedTestState(1), 1.0);
  graph_->AddEdge(AdvancedTestState(2), AdvancedTestState(5), 1.0);
  graph_->AddEdge(AdvancedTestState(3), AdvancedTestState(6), 1.0);
  
  auto vertex1_it = graph_->FindVertex(AdvancedTestState(1));
  EXPECT_NE(vertex1_it, graph_->vertex_end());
  
  // Test CheckNeighbour
  EXPECT_TRUE(vertex1_it->CheckNeighbour(AdvancedTestState(2)));
  EXPECT_TRUE(vertex1_it->CheckNeighbour(AdvancedTestState(3)));
  EXPECT_TRUE(vertex1_it->CheckNeighbour(AdvancedTestState(4)));
  EXPECT_FALSE(vertex1_it->CheckNeighbour(AdvancedTestState(5)));
  EXPECT_FALSE(vertex1_it->CheckNeighbour(AdvancedTestState(6)));
  
  // Test GetNeighbours
  auto neighbors = vertex1_it->GetNeighbours();
  EXPECT_EQ(neighbors.size(), 3);
  
  std::vector<int> neighbor_ids;
  for (const auto& neighbor : neighbors) {
    neighbor_ids.push_back(neighbor->state.id);
  }
  std::sort(neighbor_ids.begin(), neighbor_ids.end());
  
  EXPECT_EQ(neighbor_ids[0], 2);
  EXPECT_EQ(neighbor_ids[1], 3);
  EXPECT_EQ(neighbor_ids[2], 4);
}

// Test graph degree calculations
TEST_F(AdvancedGraphOperationsTest, DegreeCalculations) {
  for (int i = 1; i <= 4; ++i) {
    graph_->AddVertex(AdvancedTestState(i));
  }
  
  // Create directed edges: 1->2, 1->3, 2->1, 3->1, 4 isolated
  graph_->AddEdge(AdvancedTestState(1), AdvancedTestState(2), 1.0);
  graph_->AddEdge(AdvancedTestState(1), AdvancedTestState(3), 1.0);
  graph_->AddEdge(AdvancedTestState(2), AdvancedTestState(1), 1.0);
  graph_->AddEdge(AdvancedTestState(3), AdvancedTestState(1), 1.0);
  
  // Vertex 1: out-degree=2, in-degree=2
  EXPECT_EQ(graph_->GetVertexDegree(AdvancedTestState(1)), 2); // Out-degree
  
  // Vertex 2: out-degree=1, in-degree=1  
  EXPECT_EQ(graph_->GetVertexDegree(AdvancedTestState(2)), 1);
  
  // Vertex 3: out-degree=1, in-degree=1
  EXPECT_EQ(graph_->GetVertexDegree(AdvancedTestState(3)), 1);
  
  // Vertex 4: isolated, degree=0
  EXPECT_EQ(graph_->GetVertexDegree(AdvancedTestState(4)), 0);
  
  // Non-existent vertex
  EXPECT_EQ(graph_->GetVertexDegree(AdvancedTestState(10)), 0);
}

// Test massive graph operations for performance edge cases
TEST_F(AdvancedGraphOperationsTest, MassiveGraphOperations) {
  const int LARGE_N = 100;
  
  // Create large linear chain
  for (int i = 1; i <= LARGE_N; ++i) {
    graph_->AddVertex(AdvancedTestState(i));
  }
  
  // Add linear edges
  for (int i = 1; i < LARGE_N; ++i) {
    graph_->AddEdge(AdvancedTestState(i), AdvancedTestState(i + 1), 1.0);
  }
  
  EXPECT_EQ(graph_->GetVertexCount(), LARGE_N);
  EXPECT_EQ(graph_->GetEdgeCount(), LARGE_N - 1);
  
  // Test finding vertices throughout the chain
  for (int i = 1; i <= LARGE_N; i += 10) {
    auto vertex_it = graph_->FindVertex(AdvancedTestState(i));
    EXPECT_NE(vertex_it, graph_->vertex_end());
    EXPECT_EQ(vertex_it->state.id, i);
  }
  
  // Test removing vertices from middle
  for (int i = 50; i <= 60; ++i) {
    bool removed = graph_->RemoveVertex(AdvancedTestState(i));
    EXPECT_TRUE(removed);
  }
  
  EXPECT_EQ(graph_->GetVertexCount(), LARGE_N - 11);
  
  // Verify removed vertices don't exist
  for (int i = 50; i <= 60; ++i) {
    EXPECT_EQ(graph_->FindVertex(AdvancedTestState(i)), graph_->vertex_end());
  }
}