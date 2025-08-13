/*
 * edge_independent_test.cpp
 *
 * Created on: [Current Date]
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
    // Create a simple graph to get valid vertex iterators
    graph.reset(new Graph<TestState>());
    
    src_vertex = graph->AddVertex(TestState(1));
    dst_vertex = graph->AddVertex(TestState(2));
    other_vertex = graph->AddVertex(TestState(3));
  }

  std::unique_ptr<Graph<TestState>> graph;
  Graph<TestState>::vertex_iterator src_vertex;
  Graph<TestState>::vertex_iterator dst_vertex;
  Graph<TestState>::vertex_iterator other_vertex;
};

TEST_F(EdgeIndependentTest, EdgeConstruction) {
  // Test Edge construction with vertex iterators
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
  auto int_src = int_graph.AddVertex(TestState(10));
  auto int_dst = int_graph.AddVertex(TestState(20));
  
  Graph<TestState, int>::Edge int_edge(int_src, int_dst, 42);
  EXPECT_EQ(int_edge.cost, 42) << "Edge should work with integer cost types";
}

TEST_F(EdgeIndependentTest, EdgeAccessThroughIterators) {
  // Test accessing vertex data through edge iterators
  Graph<TestState>::Edge edge(src_vertex, dst_vertex, 7.5);
  
  EXPECT_EQ(edge.src->vertex_id, 1) << "Should access src vertex ID correctly";
  EXPECT_EQ(edge.dst->vertex_id, 2) << "Should access dst vertex ID correctly";
  EXPECT_EQ(edge.src->state.id_, 1) << "Should access src state correctly";
  EXPECT_EQ(edge.dst->state.id_, 2) << "Should access dst state correctly";
}