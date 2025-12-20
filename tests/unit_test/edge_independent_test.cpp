/*
 * edge_independent_test.cpp
 *
 * Created on: Aug 2025
 * Description: Tests for independent Edge class after refactoring
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <memory>

#include "gtest/gtest.h"

#include "graph/edge.hpp"
#include "graph/vertex.hpp"
#include "graph/graph.hpp"

using namespace xmotion;

struct TestState {
  TestState(int64_t id) : id_(id) {}
  int64_t id_;
};

class EdgeIndependentTest : public testing::Test {
protected:
  void SetUp() override {
    // Create a simple graph to get valid vertex pointers
    graph.reset(new Graph<TestState>());

    src_vertex_it = graph->AddVertex(TestState(1));
    dst_vertex_it = graph->AddVertex(TestState(2));
    other_vertex_it = graph->AddVertex(TestState(3));

    // Get stable Vertex* pointers from iterators
    src_vertex = &(*src_vertex_it);
    dst_vertex = &(*dst_vertex_it);
    other_vertex = &(*other_vertex_it);
  }

  std::unique_ptr<Graph<TestState>> graph;
  Graph<TestState>::vertex_iterator src_vertex_it;
  Graph<TestState>::vertex_iterator dst_vertex_it;
  Graph<TestState>::vertex_iterator other_vertex_it;

  // Stable Vertex* pointers for edge construction
  Graph<TestState>::Vertex* src_vertex;
  Graph<TestState>::Vertex* dst_vertex;
  Graph<TestState>::Vertex* other_vertex;
};

TEST_F(EdgeIndependentTest, EdgeConstruction) {
  // Test Edge construction with Vertex* pointers (now stable across rehash)
  Graph<TestState>::Edge edge(src_vertex, dst_vertex, 5.5);

  EXPECT_EQ(edge.src, src_vertex) << "Edge src should be set correctly";
  EXPECT_EQ(edge.dst, dst_vertex) << "Edge dst should be set correctly";
  EXPECT_EQ(edge.cost, 5.5) << "Edge cost should be set correctly";
}

TEST_F(EdgeIndependentTest, EdgeEquality) {
  // Test Edge equality operator
  Graph<TestState>::Edge edge1(src_vertex, dst_vertex, 5.5);
  Graph<TestState>::Edge edge2(src_vertex, dst_vertex, 5.5);
  Graph<TestState>::Edge edge3(src_vertex, other_vertex, 5.5);
  Graph<TestState>::Edge edge4(src_vertex, dst_vertex, 3.0);

  EXPECT_TRUE(edge1 == edge2) << "Identical edges should be equal";
  EXPECT_FALSE(edge1 == edge3) << "Edges with different dst should not be equal";
  EXPECT_FALSE(edge1 == edge4) << "Edges with different cost should not be equal";
}

TEST_F(EdgeIndependentTest, EdgePrintFunctionality) {
  // Test that PrintEdge doesn't crash (output testing is complex)
  Graph<TestState>::Edge edge(src_vertex, dst_vertex, 2.5);

  // This should not crash or throw
  EXPECT_NO_THROW(edge.PrintEdge()) << "PrintEdge should not throw exceptions";
}

TEST_F(EdgeIndependentTest, EdgeTypeAliases) {
  // Test that Edge type aliases work correctly
  using EdgeType = Graph<TestState>::Edge;
  using VertexType = Graph<TestState>::Vertex;

  // Should compile - testing type system
  EdgeType edge(src_vertex, dst_vertex, 1.0);
  EXPECT_EQ(edge.cost, 1.0) << "Type aliases should work correctly";
}

TEST_F(EdgeIndependentTest, EdgeWithDifferentCostTypes) {
  // Test Edge with different transition types
  Graph<TestState, int> int_graph;
  auto int_src_it = int_graph.AddVertex(TestState(10));
  auto int_dst_it = int_graph.AddVertex(TestState(20));

  // Get Vertex* pointers for edge construction
  auto* int_src = &(*int_src_it);
  auto* int_dst = &(*int_dst_it);

  Graph<TestState, int>::Edge int_edge(int_src, int_dst, 42);
  EXPECT_EQ(int_edge.cost, 42) << "Edge should work with integer cost types";
}

TEST_F(EdgeIndependentTest, EdgeAccessThroughPointers) {
  // Test accessing vertex data through edge pointers
  Graph<TestState>::Edge edge(src_vertex, dst_vertex, 7.5);

  EXPECT_EQ(edge.src->vertex_id, 1) << "Should access src vertex ID correctly";
  EXPECT_EQ(edge.dst->vertex_id, 2) << "Should access dst vertex ID correctly";
  EXPECT_EQ(edge.src->state.id_, 1) << "Should access src state correctly";
  EXPECT_EQ(edge.dst->state.id_, 2) << "Should access dst state correctly";
}

TEST_F(EdgeIndependentTest, EdgePointerStabilityAfterRehash) {
  // Test that Vertex* pointers remain stable after graph rehash
  // This is the key test for the iterator invalidation fix

  // Create initial edge with current vertex pointers
  Graph<TestState>::Edge edge(src_vertex, dst_vertex, 1.0);

  // Verify initial state
  EXPECT_EQ(edge.src->vertex_id, 1);
  EXPECT_EQ(edge.dst->vertex_id, 2);

  // Force rehash by adding many vertices
  for (int64_t i = 100; i < 200; ++i) {
    graph->AddVertex(TestState(i));
  }

  // Verify pointers are still valid after rehash
  // With the old iterator-based approach, this would be undefined behavior
  EXPECT_EQ(edge.src->vertex_id, 1) << "Vertex* should remain valid after rehash";
  EXPECT_EQ(edge.dst->vertex_id, 2) << "Vertex* should remain valid after rehash";
  EXPECT_EQ(edge.src->state.id_, 1) << "State data should be accessible after rehash";
  EXPECT_EQ(edge.dst->state.id_, 2) << "State data should be accessible after rehash";
}
