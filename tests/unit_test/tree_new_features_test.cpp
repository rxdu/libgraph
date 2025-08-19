/*
 * tree_new_features_test.cpp
 *
 * Tests for new Tree class features added in refactoring
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include "graph/tree.hpp"
#include <vector>

using namespace xmotion;

struct TreeNewFeaturesTest : testing::Test {
  struct TestState {
    TestState(uint64_t id) : id_(id) {}
    int64_t id_;
  };
  
  Tree<TestState*> tree;
  std::vector<TestState*> nodes;
  
  TreeNewFeaturesTest() {
    for (int i = 0; i < 10; i++) {
      nodes.push_back(new TestState(i));
    }
    
    // Build a simple tree structure:
    //       0
    //      / \
    //     1   2
    //    / \   \
    //   3   4   5
    //  /       / \
    // 6       7   8
    tree.AddEdge(nodes[0], nodes[1], 1.0);
    tree.AddEdge(nodes[0], nodes[2], 1.0);
    tree.AddEdge(nodes[1], nodes[3], 1.0);
    tree.AddEdge(nodes[1], nodes[4], 1.0);
    tree.AddEdge(nodes[2], nodes[5], 1.0);
    tree.AddEdge(nodes[3], nodes[6], 1.0);
    tree.AddEdge(nodes[5], nodes[7], 1.0);
    tree.AddEdge(nodes[5], nodes[8], 1.0);
  }
  
  virtual ~TreeNewFeaturesTest() {
    for (auto& nd : nodes) delete nd;
  }
};

TEST_F(TreeNewFeaturesTest, HasEdgeAndGetEdgeWeight) {
  // Test HasEdge
  EXPECT_TRUE(tree.HasEdge(nodes[0], nodes[1]));
  EXPECT_TRUE(tree.HasEdge(nodes[5], nodes[8]));
  EXPECT_FALSE(tree.HasEdge(nodes[1], nodes[5]));
  EXPECT_FALSE(tree.HasEdge(nodes[3], nodes[4]));
  
  // Test GetEdgeWeight
  EXPECT_DOUBLE_EQ(tree.GetEdgeWeight(nodes[0], nodes[1]), 1.0);
  EXPECT_DOUBLE_EQ(tree.GetEdgeWeight(nodes[5], nodes[7]), 1.0);
  EXPECT_DOUBLE_EQ(tree.GetEdgeWeight(nodes[1], nodes[5]), 0.0); // Non-existent edge
}

TEST_F(TreeNewFeaturesTest, GetEdgeCount) {
  EXPECT_EQ(tree.GetEdgeCount(), 8);
  
  // Add another edge and check
  tree.AddEdge(nodes[4], nodes[9], 1.0);
  EXPECT_EQ(tree.GetEdgeCount(), 9);
}

TEST_F(TreeNewFeaturesTest, SafeVertexAccess) {
  // Test GetVertex with valid IDs
  auto* vertex0 = tree.GetVertex(0);
  ASSERT_NE(vertex0, nullptr);
  EXPECT_EQ(vertex0->vertex_id, 0);
  
  auto* vertex5 = tree.GetVertex(5);
  ASSERT_NE(vertex5, nullptr);
  EXPECT_EQ(vertex5->vertex_id, 5);
  
  // Test GetVertex with invalid ID
  auto* vertex_invalid = tree.GetVertex(999);
  EXPECT_EQ(vertex_invalid, nullptr);
  
  // Test const version
  const Tree<TestState*>& const_tree = tree;
  const auto* const_vertex = const_tree.GetVertex(3);
  ASSERT_NE(const_vertex, nullptr);
  EXPECT_EQ(const_vertex->vertex_id, 3);
}

TEST_F(TreeNewFeaturesTest, IsValidTree) {
  EXPECT_TRUE(tree.IsValidTree());
  
  // Empty tree should be valid
  Tree<TestState*> empty_tree;
  EXPECT_TRUE(empty_tree.IsValidTree());
}

TEST_F(TreeNewFeaturesTest, GetTreeHeight) {
  EXPECT_EQ(tree.GetTreeHeight(), 3);
  
  // Empty tree has height 0
  Tree<TestState*> empty_tree;
  EXPECT_EQ(empty_tree.GetTreeHeight(), 0);
  
  // Single node tree has height 0
  Tree<TestState*> single_tree;
  single_tree.AddRoot(nodes[9]);
  EXPECT_EQ(single_tree.GetTreeHeight(), 0);
}

TEST_F(TreeNewFeaturesTest, GetLeafNodes) {
  auto leaves = tree.GetLeafNodes();
  EXPECT_EQ(leaves.size(), 4);
  
  // Check that all leaf nodes are actually leaves
  std::vector<int64_t> expected_leaves = {4, 6, 7, 8};
  std::vector<int64_t> actual_leaves;
  for (const auto& leaf : leaves) {
    actual_leaves.push_back(leaf->vertex_id);
    EXPECT_TRUE(leaf->edges_to.empty());
  }
  
  std::sort(actual_leaves.begin(), actual_leaves.end());
  EXPECT_EQ(actual_leaves, expected_leaves);
}

TEST_F(TreeNewFeaturesTest, GetChildren) {
  // Test root children
  auto root_children = tree.GetChildren(0);
  EXPECT_EQ(root_children.size(), 2);
  
  // Test internal node children
  auto node1_children = tree.GetChildren(1);
  EXPECT_EQ(node1_children.size(), 2);
  
  auto node5_children = tree.GetChildren(5);
  EXPECT_EQ(node5_children.size(), 2);
  
  // Test leaf node (no children)
  auto leaf_children = tree.GetChildren(6);
  EXPECT_EQ(leaf_children.size(), 0);
  
  // Test non-existent node
  auto invalid_children = tree.GetChildren(999);
  EXPECT_EQ(invalid_children.size(), 0);
}

TEST_F(TreeNewFeaturesTest, GetSubtreeSize) {
  // Full tree size from root
  EXPECT_EQ(tree.GetSubtreeSize(0), 9);
  
  // Subtree sizes
  EXPECT_EQ(tree.GetSubtreeSize(1), 4); // nodes 1,3,4,6
  EXPECT_EQ(tree.GetSubtreeSize(2), 4); // nodes 2,5,7,8
  EXPECT_EQ(tree.GetSubtreeSize(5), 3); // nodes 5,7,8
  
  // Leaf nodes have subtree size 1
  EXPECT_EQ(tree.GetSubtreeSize(6), 1);
  EXPECT_EQ(tree.GetSubtreeSize(7), 1);
  
  // Non-existent node
  EXPECT_EQ(tree.GetSubtreeSize(999), 0);
}

TEST_F(TreeNewFeaturesTest, IsConnected) {
  EXPECT_TRUE(tree.IsConnected());
  
  // Empty tree is connected
  Tree<TestState*> empty_tree;
  EXPECT_TRUE(empty_tree.IsConnected());
  
  // Tree with disconnected component is not connected
  // Note: We can't easily test this without breaking tree invariants
  // A proper tree should always be connected if constructed correctly
}

TEST_F(TreeNewFeaturesTest, ExceptionHandling) {
  // Test GetParentVertex with invalid ID
  EXPECT_THROW(tree.GetParentVertex(999), ElementNotFoundError);
  
  // Test GetParentVertex for root (should return end())
  EXPECT_EQ(tree.GetParentVertex(0), tree.vertex_end());
  
  // Test GetParentVertex for valid non-root node
  auto parent = tree.GetParentVertex(6);
  EXPECT_EQ(parent->vertex_id, 3);
}