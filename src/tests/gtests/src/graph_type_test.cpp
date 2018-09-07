/* 
 * graph_iter_test.cpp
 * 
 * Created on: Mar 13, 2018 12:26
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include <stdio.h>
#include <vector>
#include <memory>

#include "gtest/gtest.h"

#include "graph/graph.hpp"

using namespace librav;

struct TestState
{
	TestState(uint64_t id) : id_(id){};

	int64_t id_;

	bool operator==(const TestState &other)
	{
		if (id_ == other.id_)
			return true;
		return false;
	}
};

struct SharedPtrTestStateIndexer
{
	int64_t operator()(std::shared_ptr<TestState> state)
	{
		return state->id_;
	}
};

struct GraphTypeTest : testing::Test
{
	std::vector<TestState *> nodes;
	std::vector<std::shared_ptr<TestState>> shared_nodes;

	GraphTypeTest()
	{
		for (int i = 0; i < 9; i++)
		{
			nodes.push_back(new TestState(i));
			shared_nodes.push_back(std::make_shared<TestState>(i));
		}
	}

	virtual ~GraphTypeTest()
	{
		for (auto &nd : nodes)
			delete nd;
	}
};

TEST_F(GraphTypeTest, ValueType)
{
	// create a graph
	Graph<TestState> graph;

	graph.AddEdge(*(nodes[0]), *(nodes[1]), 1.0);
	graph.AddEdge(*(nodes[0]), *(nodes[3]), 1.5);
	graph.AddEdge(*(nodes[1]), *(nodes[0]), 2.0);
	graph.AddEdge(*(nodes[1]), *(nodes[4]), 2.5);
	graph.AddEdge(*(nodes[1]), *(nodes[2]), 1.0);
	graph.AddEdge(*(nodes[2]), *(nodes[1]), 1.5);
	graph.AddEdge(*(nodes[2]), *(nodes[5]), 2.0);
	graph.AddEdge(*(nodes[3]), *(nodes[0]), 2.5);
	graph.AddEdge(*(nodes[3]), *(nodes[4]), 2.5);
	graph.AddEdge(*(nodes[4]), *(nodes[1]), 2.5);
	graph.AddEdge(*(nodes[4]), *(nodes[3]), 2.5);
	graph.AddEdge(*(nodes[4]), *(nodes[5]), 2.5);
	graph.AddEdge(*(nodes[5]), *(nodes[2]), 2.5);
	graph.AddEdge(*(nodes[5]), *(nodes[4]), 2.5);
	graph.AddEdge(*(nodes[5]), *(nodes[8]), 2.5);
	graph.AddEdge(*(nodes[5]), *(nodes[6]), 2.5);
	graph.AddEdge(*(nodes[7]), *(nodes[4]), 2.5);
	graph.AddEdge(*(nodes[7]), *(nodes[8]), 2.5);
	graph.AddEdge(*(nodes[8]), *(nodes[5]), 2.5);
	graph.AddEdge(*(nodes[8]), *(nodes[7]), 2.5);

	ASSERT_EQ(graph.GetGraphVertexNumber(), 9) << "Failed to add vertices to value-type graph ";
	ASSERT_EQ(graph.GetGraphEdgeNumber(), 20) << "Failed to add vertices to value-type graph ";
}

TEST_F(GraphTypeTest, PointerType)
{
	Graph<TestState *> graph;

	graph.AddEdge((nodes[0]), (nodes[1]), 1.0);
	graph.AddEdge((nodes[0]), (nodes[3]), 1.5);
	graph.AddEdge((nodes[1]), (nodes[0]), 2.0);
	graph.AddEdge((nodes[1]), (nodes[4]), 2.5);
	graph.AddEdge((nodes[1]), (nodes[2]), 3.0);
	graph.AddEdge((nodes[2]), (nodes[1]), 3.5);
	graph.AddEdge((nodes[2]), (nodes[5]), 4.0);
	graph.AddEdge((nodes[3]), (nodes[0]), 4.5);
	graph.AddEdge((nodes[3]), (nodes[4]), 5.0);
	graph.AddEdge((nodes[4]), (nodes[1]), 5.5);
	graph.AddEdge((nodes[4]), (nodes[3]), 6.0);
	graph.AddEdge((nodes[4]), (nodes[5]), 6.5);
	graph.AddEdge((nodes[5]), (nodes[2]), 7.0);
	graph.AddEdge((nodes[5]), (nodes[4]), 7.5);
	graph.AddEdge((nodes[5]), (nodes[8]), 8.0);
	graph.AddEdge((nodes[5]), (nodes[6]), 9.0);
	graph.AddEdge((nodes[7]), (nodes[4]), 9.5);
	graph.AddEdge((nodes[7]), (nodes[8]), 10.0);
	graph.AddEdge((nodes[8]), (nodes[5]), 10.5);
	graph.AddEdge((nodes[8]), (nodes[7]), 11.0);

	ASSERT_EQ(graph.GetGraphVertexNumber(), 9) << "Failed to add vertices to pointer-type graph ";
	ASSERT_EQ(graph.GetGraphEdgeNumber(), 20) << "Failed to add vertices to pointer-type graph ";
}

TEST_F(GraphTypeTest, SharedPointerType)
{
	Graph<std::shared_ptr<TestState>, double, SharedPtrTestStateIndexer> graph;

	graph.AddEdge((shared_nodes[0]), (shared_nodes[1]), 1.0);
	graph.AddEdge((shared_nodes[0]), (shared_nodes[3]), 1.5);
	graph.AddEdge((shared_nodes[1]), (shared_nodes[0]), 2.0);
	graph.AddEdge((shared_nodes[1]), (shared_nodes[4]), 2.5);
	graph.AddEdge((shared_nodes[1]), (shared_nodes[2]), 3.0);
	graph.AddEdge((shared_nodes[2]), (shared_nodes[1]), 3.5);
	graph.AddEdge((shared_nodes[2]), (shared_nodes[5]), 4.0);
	graph.AddEdge((shared_nodes[3]), (shared_nodes[0]), 4.5);
	graph.AddEdge((shared_nodes[3]), (shared_nodes[4]), 5.0);
	graph.AddEdge((shared_nodes[4]), (shared_nodes[1]), 5.5);
	graph.AddEdge((shared_nodes[4]), (shared_nodes[3]), 6.0);
	graph.AddEdge((shared_nodes[4]), (shared_nodes[5]), 6.5);
	graph.AddEdge((shared_nodes[5]), (shared_nodes[2]), 7.0);
	graph.AddEdge((shared_nodes[5]), (shared_nodes[4]), 7.5);
	graph.AddEdge((shared_nodes[5]), (shared_nodes[8]), 8.0);
	graph.AddEdge((shared_nodes[5]), (shared_nodes[6]), 9.0);
	graph.AddEdge((shared_nodes[7]), (shared_nodes[4]), 9.5);
	graph.AddEdge((shared_nodes[7]), (shared_nodes[8]), 10.0);
	graph.AddEdge((shared_nodes[8]), (shared_nodes[5]), 10.5);
	graph.AddEdge((shared_nodes[8]), (shared_nodes[7]), 11.0);

	ASSERT_EQ(graph.GetGraphVertexNumber(), 9) << "Failed to add vertices to pointer-type graph ";
	ASSERT_EQ(graph.GetGraphEdgeNumber(), 20) << "Failed to add vertices to pointer-type graph ";
}
