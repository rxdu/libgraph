/* 
 * tree_mod_test.cpp
 * 
 * Created on: Mar 14, 2018 15:37
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include <stdio.h>
#include <vector>

#include "gtest/gtest.h"

#include "graph/tree.hpp"

using namespace librav;

struct TestState
{
    TestState(uint64_t id) : id_(id){};

    int64_t id_;
};

struct TreeModificationTest : testing::Test
{
    TreeModificationTest()
    {
        for (int i = 0; i < 9; i++)
            nodes.push_back(new TestState(i));
    }

    virtual ~TreeModificationTest()
    {
        for (auto &nd : nodes)
            delete nd;
    }

    std::vector<TestState *> nodes;
};

TEST_F(TreeModificationTest, RootVertexMod)
{
    // create a tree
    Tree<TestState *> tree;

    ASSERT_EQ(tree.GetTotalVertexNumber(), 0) << "Tree should have no vertex now";
    ASSERT_EQ(tree.GetRootVertex(), tree.vertex_end()) << "Root vertex should not exist";

    tree.AddVertex(nodes[0]);
    ASSERT_EQ(tree.GetRootVertex()->GetVertexID(), 0) << "Root vertex should have id 0";

    tree.ClearAll();
    ASSERT_EQ(tree.GetRootVertex(), tree.vertex_end()) << "Root vertex should not exist after ClearAll 1";

    tree.AddEdge(nodes[1], nodes[2], 1.5);
    ASSERT_EQ(tree.GetRootVertex()->GetVertexID(), 1) << "Root vertex should have id 1";

    tree.ClearAll();
    ASSERT_EQ(tree.GetRootVertex(), tree.vertex_end()) << "Root vertex should not exist after ClearAll 2";

    tree.AddEdge(nodes[3], nodes[1], 1.5);
    ASSERT_EQ(tree.GetRootVertex()->GetVertexID(), 3) << "Root vertex should have id 3";
}

TEST_F(TreeModificationTest, VertexMod)
{
    // create a tree
    Tree<TestState *> tree;

    ASSERT_EQ(tree.GetTotalVertexNumber(), 0) << "Tree should have no vertex now";

    tree.AddVertex(nodes[0]);
    tree.AddVertex(nodes[1]);

    ASSERT_EQ(tree.GetTotalVertexNumber(), 1) << "Failed to add vertices to pointer-type tree ";

    tree.AddEdge(nodes[0], nodes[1], 1.5);
    ASSERT_EQ(tree.FindVertex(0)->vertex_id_, 0) << "Failed to find added vertex by associated state ID from pointer-type tree ";
    ASSERT_EQ(tree.FindVertex(nodes[1])->vertex_id_, 1) << "Failed to find added vertex by associated state from pointer-type tree ";

    tree.RemoveVertex(1);

    ASSERT_EQ(tree.GetTotalVertexNumber(), 1) << "Failed to remove vertex by associated state ID from pointer-type tree ";
    ASSERT_TRUE(tree.FindVertex(1) == tree.vertex_end()) << "Failed to remove vertex by associated state ID from pointer-type tree ";
    ASSERT_EQ(tree.FindVertex(0)->vertex_id_, 0) << "Removed wrong vertex by associated state ID from pointer-type tree ";

    tree.RemoveVertex(nodes[0]);

    ASSERT_EQ(tree.GetTotalVertexNumber(), 0) << "Failed to remove vertex by associated state from pointer-type tree ";
    ASSERT_TRUE(tree.FindVertex(0) == tree.vertex_end()) << "Failed to remove vertex by associated state from pointer-type tree ";

    tree.AddEdge(nodes[0], nodes[1], 1.5);
    tree.AddEdge(nodes[0], nodes[2], 1.5);
    tree.AddEdge(nodes[1], nodes[3], 1.5);
    tree.AddEdge(nodes[2], nodes[4], 1.5);

    ASSERT_EQ(tree.GetTotalVertexNumber(), 5) << "Failed to add vertices to pointer-type tree ";

    tree.RemoveVertex(nodes[1]);
    ASSERT_EQ(tree.GetTotalVertexNumber(), 3) << "Failed to remove vertex and subtree by associated state ID from pointer-type tree ";
    ASSERT_TRUE(tree.FindVertex(1) == tree.vertex_end()) << "Failed to remove subtree by associated state ID from pointer-type tree ";
    ASSERT_TRUE(tree.FindVertex(3) == tree.vertex_end()) << "Failed to remove subtree by associated state ID from pointer-type tree ";
}

TEST_F(TreeModificationTest, EdgeMod)
{
    Tree<TestState *> tree;

    ASSERT_EQ(tree.GetTotalEdgeNumber(), 0) << "Tree should have no edge now";

    tree.AddEdge(nodes[0], nodes[1], 1.2);
    tree.AddEdge(nodes[1], nodes[2], 1.5);

    ASSERT_EQ(tree.GetTotalEdgeNumber(), 2) << "Failed to add edges to pointer-type tree";
    std::vector<Tree<TestState *>::edge_iterator> edges;
    for (auto it = tree.FindVertex(0)->edge_begin(); it != tree.FindVertex(0)->edge_end(); ++it)
        edges.push_back(it);
    ASSERT_EQ(edges.size(), 1) << "Wrong number of edges added to vertex in pointer-type tree";
    ASSERT_EQ(edges.front()->src_->vertex_id_, 0) << "Wrong src of edges added to vertex in pointer-type tree";
    ASSERT_EQ(edges.front()->dst_->vertex_id_, 1) << "Wrong dst of edges added to vertex in pointer-type tree";
    ASSERT_EQ(edges.front()->cost_, 1.2) << "Wrong cost of edges added to vertex in pointer-type tree";

    ASSERT_EQ(tree.FindVertex(nodes[1])->edges_to_.size(), 1) << "Failed to add edge to vertex";
    ASSERT_EQ(tree.FindVertex(nodes[2])->vertices_from_.size(), 1) << "Failed to maintain list of vertices_from_";

    tree.RemoveEdge(nodes[1], nodes[2]);

    ASSERT_EQ(tree.FindVertex(nodes[1])->edges_to_.size(), 0) << "Failed to remove edge frome vertex";
    ASSERT_EQ(tree.GetTotalEdgeNumber(), 1) << "Failed to remove a edge from pointer-type tree";

    tree.AddEdge(nodes[1], nodes[2], 1.2);
    tree.AddEdge(nodes[2], nodes[3], 1.5);

    ASSERT_EQ(tree.GetTotalEdgeNumber(), 3) << "Failed to remove a edge from pointer-type tree";
    tree.RemoveEdge(nodes[1], nodes[2]);
    ASSERT_EQ(tree.GetTotalEdgeNumber(), 1) << "Failed to remove a edge from pointer-type tree";

    edges.clear();
    for (auto it = tree.FindVertex(0)->edge_begin(); it != tree.FindVertex(0)->edge_end(); ++it)
        edges.push_back(it);
    bool edge_intact = (edges.size() == 1) && (edges.front()->src_->vertex_id_ == 0) && (edges.front()->dst_->vertex_id_ == 1) && (edges.front()->cost_ == 1.2);
    ASSERT_TRUE(edge_intact) << "A wrong edge is removed from pointer-type tree";

    tree.AddUndirectedEdge(nodes[3], nodes[4], 1.8);
    tree.AddUndirectedEdge(nodes[4], nodes[5], 2.0);
    ASSERT_EQ(tree.GetTotalEdgeNumber(), 3) << "Failed to add a unedge from pointer-type tree";

    tree.RemoveUndirectedEdge(nodes[4], nodes[5]);
    ASSERT_EQ(tree.GetTotalEdgeNumber(), 2) << "Failed to remove a unedge from pointer-type tree";
}

TEST_F(TreeModificationTest, SubtreeMod)
{
    // create a tree
    Tree<TestState *> tree;

    ASSERT_EQ(tree.GetTotalVertexNumber(), 0) << "Tree should have no vertex now";

    tree.AddEdge(nodes[0], nodes[1], 1.5);
    tree.AddEdge(nodes[0], nodes[2], 1.5);
    tree.AddEdge(nodes[2], nodes[3], 1.5);
    tree.AddEdge(nodes[1], nodes[4], 1.5);
    tree.AddEdge(nodes[1], nodes[5], 1.5);
    tree.AddEdge(nodes[5], nodes[6], 1.5);
    tree.AddEdge(nodes[5], nodes[7], 1.5);
    tree.AddEdge(nodes[7], nodes[8], 1.5);

    ASSERT_EQ(tree.GetTotalVertexNumber(), 9) << "Failed to add vertices to pointer-type tree ";
    ASSERT_EQ(tree.GetTotalEdgeNumber(), 8) << "Failed to add vertices to pointer-type tree ";

    ASSERT_EQ(tree.GetVertexDepth(nodes[0]), 0) << "Failed to get depth of vertex in the tree ";
    ASSERT_EQ(tree.GetVertexDepth(nodes[1]), 1) << "Failed to get depth of vertex in the tree ";
    ASSERT_EQ(tree.GetVertexDepth(nodes[2]), 1) << "Failed to get depth of vertex in the tree ";
    ASSERT_EQ(tree.GetVertexDepth(nodes[7]), 3) << "Failed to get depth of vertex in the tree ";
    ASSERT_EQ(tree.GetVertexDepth(nodes[8]), 4) << "Failed to get depth of vertex in the tree ";

    tree.RemoveSubtree(1);

    ASSERT_EQ(tree.GetTotalVertexNumber(), 3) << "Failed to remove vertex and subtree by associated state ID from pointer-type tree ";
    ASSERT_EQ(tree.GetTotalEdgeNumber(), 2) << "Failed to add vertices to pointer-type tree ";
}

TEST_F(TreeModificationTest, ClearVertexEdge)
{
    Tree<TestState *> tree;

    ASSERT_EQ(tree.GetTotalVertexNumber(), 0) << "Tree should have no vertex at beginning";
    ASSERT_EQ(tree.GetTotalEdgeNumber(), 0) << "Tree should have no edge at beginning";

    tree.AddEdge(nodes[0], nodes[1], 1.2);
    tree.AddEdge(nodes[1], nodes[2], 1.5);

    ASSERT_TRUE(tree.GetTotalVertexNumber() == 3 && tree.GetTotalEdgeNumber() == 2) << "Tree should have some vertices and edges now";

    tree.ClearAll();

    ASSERT_TRUE(tree.GetTotalVertexNumber() == 0 && tree.GetTotalEdgeNumber() == 0) << "Tree should be empty now";
}

TEST_F(TreeModificationTest, VertexAccessEdge)
{
    Tree<TestState *> tree;

    tree.AddEdge(nodes[0], nodes[1], 1.2);
    tree.AddEdge(nodes[0], nodes[2], 1.5);
    tree.AddEdge(nodes[0], nodes[3], 1.8);

    std::vector<int64_t> nc = {1, 2, 3};

    auto neighbours = tree.FindVertex(0)->GetNeighbours();
    std::vector<int64_t> nids2;
    for (auto &n : neighbours)
        nids2.push_back(n->vertex_id_);
    ASSERT_TRUE(nids2 == nc) << "Tree should have 3 neighbors (checked from vertex pointer)";

    auto nbs = tree.FindVertex(0)->GetNeighbours();
    std::vector<int64_t> nids;
    for (auto &nb : nbs)
        nids.push_back(nb->vertex_id_);
    ASSERT_TRUE(nids.size() == 3) << "Tree should have 3 neighbors";
    ASSERT_TRUE(nids == nc) << "Tree should have 3 neighbors";

    auto edge_cost1 = tree.FindVertex(0)->FindEdge(1)->cost_;
    auto edge_cost2 = tree.FindVertex(0)->FindEdge(2)->cost_;
    auto edge_cost3 = tree.FindVertex(0)->FindEdge(3)->cost_;

    ASSERT_TRUE(edge_cost1 == 1.2) << "Edge cost to vertex 1 should be 1.2";
    ASSERT_TRUE(edge_cost2 == 1.5) << "Edge cost to vertex 2 should be 1.5";
    ASSERT_TRUE(edge_cost3 == 1.8) << "Edge cost to vertex 3 should be 1.8";

    bool check_neighbour = tree.FindVertex(0)->CheckNeighbour(1) && tree.FindVertex(0)->CheckNeighbour(2) && tree.FindVertex(0)->CheckNeighbour(3);
    ASSERT_TRUE(check_neighbour) << "Vertex 0 and 1,2,3 should be neighbours";
}
