/* 
 * graph_mod_test.cpp
 * 
 * Created on: Mar 14, 2018 15:37
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include <stdio.h>
#include <vector>

#include "gtest/gtest.h"

#include "graph/graph.hpp"

using namespace librav;

struct TestState
{
	TestState(uint64_t id) : any_unique_id_(id){};

	int64_t any_unique_id_;

	int64_t GetUniqueID() const
	{
		return any_unique_id_;
	}
};

struct GraphModificationTest : testing::Test
{
	GraphModificationTest()
	{
		for (int i = 0; i < 9; i++)
			nodes.push_back(new TestState(i));
	}

	virtual ~GraphModificationTest()
	{
		for (auto &nd : nodes)
			delete nd;
	}

	std::vector<TestState *> nodes;
};

TEST_F(GraphModificationTest, VertexMod)
{
	// create a graph
	Graph_t<TestState *> graph;

	ASSERT_EQ(graph.GetGraphVertexNumber(), 0) << "Graph should have no vertex now";

	graph.AddVertex(nodes[0]);
	graph.AddVertex(nodes[1]);

	ASSERT_EQ(graph.GetGraphVertexNumber(), 2) << "Failed to add vertices to pointer-type graph ";

	ASSERT_EQ(graph.FindVertex(0)->vertex_id_, 0) << "Failed to find added vertex by associated state ID from pointer-type graph ";
	ASSERT_EQ(graph.FindVertex(nodes[1])->vertex_id_, 1) << "Failed to find added vertex by associated state from pointer-type graph ";

	graph.RemoveVertex(0);

	ASSERT_EQ(graph.GetGraphVertexNumber(), 1) << "Failed to remove vertex by associated state ID from pointer-type graph ";
	ASSERT_TRUE(graph.FindVertex(0) == graph.vertex_end()) << "Failed to remove vertex by associated state ID from pointer-type graph ";
	ASSERT_EQ(graph.FindVertex(1)->vertex_id_, 1) << "Removed wrong vertex by associated state ID from pointer-type graph ";

	graph.RemoveVertex(nodes[1]);

	ASSERT_EQ(graph.GetGraphVertexNumber(), 0) << "Failed to remove vertex by associated state from pointer-type graph ";
	ASSERT_TRUE(graph.FindVertex(1) == graph.vertex_end()) << "Failed to remove vertex by associated state from pointer-type graph ";
}

TEST_F(GraphModificationTest, EdgeMod)
{
	Graph_t<TestState *> graph;

	ASSERT_EQ(graph.GetGraphEdgeNumber(), 0) << "Graph should have no edge now";

	graph.AddEdge(nodes[0], nodes[1], 1.2);
	graph.AddEdge(nodes[1], nodes[2], 1.5);

	ASSERT_EQ(graph.GetGraphEdgeNumber(), 2) << "Failed to add directed edges to pointer-type graph";
	std::vector<Graph_t<TestState *>::edge_iterator> edges;
	for (auto it = graph.FindVertex(0)->edge_begin(); it != graph.FindVertex(0)->edge_end(); ++it)
		edges.push_back(it);
	ASSERT_EQ(edges.size(), 1) << "Wrong number of directed edges added to vertex in pointer-type graph";
	ASSERT_EQ(edges.front()->src_->vertex_id_, 0) << "Wrong src of directed edges added to vertex in pointer-type graph";
	ASSERT_EQ(edges.front()->dst_->vertex_id_, 1) << "Wrong dst of directed edges added to vertex in pointer-type graph";
	ASSERT_EQ(edges.front()->cost_, 1.2) << "Wrong cost of directed edges added to vertex in pointer-type graph";

	graph.RemoveEdge(nodes[1], nodes[2]);

	ASSERT_EQ(graph.GetGraphEdgeNumber(), 1) << "Failed to remove a directed edge from pointer-type graph";

	edges.clear();
	for (auto it = graph.FindVertex(0)->edge_begin(); it != graph.FindVertex(0)->edge_end(); ++it)
		edges.push_back(it);
	bool edge_intact = (edges.size() == 1) && (edges.front()->src_->vertex_id_ == 0) && (edges.front()->dst_->vertex_id_ == 1) && (edges.front()->cost_ == 1.2);
	ASSERT_TRUE(edge_intact) << "A wrong edge is removed from pointer-type graph";

	graph.AddUndirectedEdge(nodes[3], nodes[4], 1.8);
	graph.AddUndirectedEdge(nodes[4], nodes[5], 2.0);
	ASSERT_EQ(graph.GetGraphEdgeNumber(), 5) << "Failed to add a undirected edge from pointer-type graph";

	graph.RemoveUndirectedEdge(nodes[4], nodes[5]);
	ASSERT_EQ(graph.GetGraphEdgeNumber(), 3) << "Failed to remove a undirected edge from pointer-type graph";
}

TEST_F(GraphModificationTest, ClearVertexEdge)
{
	Graph_t<TestState *> graph;

	ASSERT_EQ(graph.GetGraphVertexNumber(), 0) << "Graph should have no vertex at beginning";
	ASSERT_EQ(graph.GetGraphEdgeNumber(), 0) << "Graph should have no edge at beginning";

	graph.AddEdge(nodes[0], nodes[1], 1.2);
	graph.AddEdge(nodes[1], nodes[2], 1.5);

	ASSERT_TRUE(graph.GetGraphVertexNumber() == 3 && graph.GetGraphEdgeNumber() == 2) << "Graph should have some vertices and edges now";

	graph.ClearGraph();

	ASSERT_TRUE(graph.GetGraphVertexNumber() == 0 && graph.GetGraphEdgeNumber() == 0) << "Graph should be empty now";
}
