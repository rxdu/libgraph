/*
 * graph_bigfive_test.cpp
 *
 * Created on: Sep 04, 2018 02:15
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include <stdio.h>
#include <vector>
#include <memory>

#include "gtest/gtest.h"

#include "graph/graph.hpp"

using namespace rdu;

struct TestState {
  TestState(uint64_t id) : id_(id){};

  int64_t id_;
};

struct GraphBigFiveTest : testing::Test {
  std::vector<TestState *> nodes;
  std::vector<std::shared_ptr<TestState>> shared_nodes;

  GraphBigFiveTest() {
    for (int i = 0; i < 9; i++) {
      nodes.push_back(new TestState(i));
      shared_nodes.push_back(std::make_shared<TestState>(i));
    }
  }

  virtual ~GraphBigFiveTest() {
    for (auto &nd : nodes) delete nd;
  }
};

TEST_F(GraphBigFiveTest, DefaultConstructor) {
  // create a graph
  Graph<TestState> graph;

  graph.AddEdge(*(nodes[0]), *(nodes[1]), 1.0);
  graph.AddEdge(*(nodes[0]), *(nodes[3]), 1.5);
  graph.AddEdge(*(nodes[1]), *(nodes[0]), 2.0);
  graph.AddEdge(*(nodes[1]), *(nodes[4]), 2.5);
  graph.AddEdge(*(nodes[1]), *(nodes[2]), 1.0);
  graph.AddEdge(*(nodes[2]), *(nodes[5]), 2.0);

  ASSERT_EQ(graph.GetTotalVertexNumber(), 6)
      << "Failed to add vertices to original graph ";
  ASSERT_EQ(graph.GetTotalEdgeNumber(), 6)
      << "Failed to add edges to original graph ";
}

TEST_F(GraphBigFiveTest, CopyConstructor) {
  // create a graph
  Graph<TestState> graph;

  graph.AddEdge(*(nodes[0]), *(nodes[1]), 1.0);
  graph.AddEdge(*(nodes[0]), *(nodes[3]), 1.5);
  graph.AddEdge(*(nodes[1]), *(nodes[0]), 2.0);
  graph.AddEdge(*(nodes[1]), *(nodes[4]), 2.5);
  graph.AddEdge(*(nodes[1]), *(nodes[2]), 1.0);
  graph.AddEdge(*(nodes[2]), *(nodes[5]), 2.0);

  ASSERT_EQ(graph.GetTotalVertexNumber(), 6)
      << "Failed to add vertices to original graph ";
  ASSERT_EQ(graph.GetTotalEdgeNumber(), 6)
      << "Failed to add edges to original graph ";

  Graph<TestState> copy_graph(graph);

  ASSERT_EQ(copy_graph.GetTotalVertexNumber(), 6)
      << "Failed to add vertices to copied graph ";
  ASSERT_EQ(copy_graph.GetTotalEdgeNumber(), 6)
      << "Failed to add vertices to copied graph ";

  Graph<TestState> copy_graph2 = copy_graph;

  ASSERT_EQ(copy_graph2.GetTotalVertexNumber(), 6)
      << "Failed to add vertices to copied graph 2 ";
  ASSERT_EQ(copy_graph2.GetTotalEdgeNumber(), 6)
      << "Failed to add vertices to copied graph 2 ";
}

TEST_F(GraphBigFiveTest, AssignmentOperator) {
  // create a graph
  Graph<TestState> graph;

  graph.AddEdge(*(nodes[0]), *(nodes[1]), 1.0);
  graph.AddEdge(*(nodes[0]), *(nodes[3]), 1.5);
  graph.AddEdge(*(nodes[1]), *(nodes[0]), 2.0);
  graph.AddEdge(*(nodes[1]), *(nodes[4]), 2.5);
  graph.AddEdge(*(nodes[1]), *(nodes[2]), 1.0);
  graph.AddEdge(*(nodes[2]), *(nodes[5]), 2.0);

  ASSERT_EQ(graph.GetTotalVertexNumber(), 6)
      << "Failed to add vertices to original graph ";
  ASSERT_EQ(graph.GetTotalEdgeNumber(), 6)
      << "Failed to add edges to original graph ";

  Graph<TestState> assign_graph;
  assign_graph = graph;

  ASSERT_EQ(assign_graph.GetTotalVertexNumber(), 6)
      << "Failed to add vertices to assigned graph ";
  ASSERT_EQ(assign_graph.GetTotalEdgeNumber(), 6)
      << "Failed to add vertices to assigned graph ";
}

Graph<TestState> CreateGraph() {
  Graph<TestState> graph;

  graph.AddEdge(TestState(0), TestState(1), 1.0);
  graph.AddEdge(TestState(0), TestState(3), 1.5);
  graph.AddEdge(TestState(1), TestState(0), 2.0);
  graph.AddEdge(TestState(1), TestState(4), 2.5);
  graph.AddEdge(TestState(1), TestState(2), 1.0);
  graph.AddEdge(TestState(2), TestState(5), 2.0);

  return graph;
}

TEST_F(GraphBigFiveTest, MoveConstructor) {
  // create a graph
  Graph<TestState> move_graph(CreateGraph());

  ASSERT_EQ(move_graph.GetTotalVertexNumber(), 6)
      << "Failed to add vertices to moved graph ";
  ASSERT_EQ(move_graph.GetTotalEdgeNumber(), 6)
      << "Failed to add vertices to moved graph ";
}

TEST_F(GraphBigFiveTest, MoveAssignConstructor) {
  // create a graph
  Graph<TestState> move_graph;
  move_graph = CreateGraph();

  ASSERT_EQ(move_graph.GetTotalVertexNumber(), 6)
      << "Failed to add vertices to move assigned graph ";
  ASSERT_EQ(move_graph.GetTotalEdgeNumber(), 6)
      << "Failed to add vertices to move assigned graph ";
}
