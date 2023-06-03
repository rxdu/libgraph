/*
 * tree_bigfive_test.cpp
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

#include "graph/tree.hpp"

using namespace xmotion;

struct TestState {
  TestState(uint64_t id) : id_(id){};

  int64_t id_;
};

struct TreeBigFiveTest : testing::Test {
  std::vector<TestState *> nodes;
  std::vector<std::shared_ptr<TestState>> shared_nodes;

  TreeBigFiveTest() {
    for (int i = 0; i < 9; i++) {
      nodes.push_back(new TestState(i));
      shared_nodes.push_back(std::make_shared<TestState>(i));
    }
  }

  virtual ~TreeBigFiveTest() {
    for (auto &nd : nodes) delete nd;
  }
};

TEST_F(TreeBigFiveTest, DefaultConstructor) {
  // create a tree
  Tree<TestState> tree;

  tree.AddEdge(*(nodes[0]), *(nodes[1]), 1.0);
  tree.AddEdge(*(nodes[0]), *(nodes[3]), 1.5);
  tree.AddEdge(*(nodes[1]), *(nodes[0]), 2.0);
  tree.AddEdge(*(nodes[1]), *(nodes[4]), 2.5);
  tree.AddEdge(*(nodes[1]), *(nodes[2]), 1.0);
  tree.AddEdge(*(nodes[2]), *(nodes[5]), 2.0);

  ASSERT_EQ(tree.GetTotalVertexNumber(), 6)
      << "Failed to add vertices to original tree ";
  ASSERT_EQ(tree.GetTotalEdgeNumber(), 6)
      << "Failed to add edges to original tree ";
}

TEST_F(TreeBigFiveTest, CopyConstructor) {
  // create a tree
  Tree<TestState> tree;

  tree.AddEdge(*(nodes[0]), *(nodes[1]), 1.0);
  tree.AddEdge(*(nodes[0]), *(nodes[3]), 1.5);
  tree.AddEdge(*(nodes[1]), *(nodes[0]), 2.0);
  tree.AddEdge(*(nodes[1]), *(nodes[4]), 2.5);
  tree.AddEdge(*(nodes[1]), *(nodes[2]), 1.0);
  tree.AddEdge(*(nodes[2]), *(nodes[5]), 2.0);

  ASSERT_EQ(tree.GetTotalVertexNumber(), 6)
      << "Failed to add vertices to original tree ";
  ASSERT_EQ(tree.GetTotalEdgeNumber(), 6)
      << "Failed to add edges to original tree ";

  Tree<TestState> copy_tree(tree);

  ASSERT_EQ(copy_tree.GetTotalVertexNumber(), 6)
      << "Failed to add vertices to copied tree ";
  ASSERT_EQ(copy_tree.GetTotalEdgeNumber(), 6)
      << "Failed to add vertices to copied tree ";

  Tree<TestState> copy_tree2 = copy_tree;

  ASSERT_EQ(copy_tree2.GetTotalVertexNumber(), 6)
      << "Failed to add vertices to copied tree 2 ";
  ASSERT_EQ(copy_tree2.GetTotalEdgeNumber(), 6)
      << "Failed to add vertices to copied tree 2 ";
}

TEST_F(TreeBigFiveTest, AssignmentOperator) {
  // create a tree
  Tree<TestState> tree;

  tree.AddEdge(*(nodes[0]), *(nodes[1]), 1.0);
  tree.AddEdge(*(nodes[0]), *(nodes[3]), 1.5);
  tree.AddEdge(*(nodes[1]), *(nodes[0]), 2.0);
  tree.AddEdge(*(nodes[1]), *(nodes[4]), 2.5);
  tree.AddEdge(*(nodes[1]), *(nodes[2]), 1.0);
  tree.AddEdge(*(nodes[2]), *(nodes[5]), 2.0);

  ASSERT_EQ(tree.GetTotalVertexNumber(), 6)
      << "Failed to add vertices to original tree ";
  ASSERT_EQ(tree.GetTotalEdgeNumber(), 6)
      << "Failed to add edges to original tree ";

  Tree<TestState> assign_tree;
  assign_tree = tree;

  ASSERT_EQ(assign_tree.GetTotalVertexNumber(), 6)
      << "Failed to add vertices to assigned tree ";
  ASSERT_EQ(assign_tree.GetTotalEdgeNumber(), 6)
      << "Failed to add vertices to assigned tree ";
}

Tree<TestState> CreateTree() {
  Tree<TestState> tree;

  tree.AddEdge(TestState(0), TestState(1), 1.0);
  tree.AddEdge(TestState(0), TestState(3), 1.5);
  tree.AddEdge(TestState(1), TestState(0), 2.0);
  tree.AddEdge(TestState(1), TestState(4), 2.5);
  tree.AddEdge(TestState(1), TestState(2), 1.0);
  tree.AddEdge(TestState(2), TestState(5), 2.0);

  return tree;
}

TEST_F(TreeBigFiveTest, MoveConstructor) {
  // create a tree
  Tree<TestState> move_tree(CreateTree());

  ASSERT_EQ(move_tree.GetTotalVertexNumber(), 6)
      << "Failed to add vertices to moved tree ";
  ASSERT_EQ(move_tree.GetTotalEdgeNumber(), 6)
      << "Failed to add vertices to moved tree ";
}

TEST_F(TreeBigFiveTest, MoveAssignConstructor) {
  // create a tree
  Tree<TestState> move_tree;
  move_tree = CreateTree();

  ASSERT_EQ(move_tree.GetTotalVertexNumber(), 6)
      << "Failed to add vertices to move assigned tree ";
  ASSERT_EQ(move_tree.GetTotalEdgeNumber(), 6)
      << "Failed to add vertices to move assigned tree ";
}
