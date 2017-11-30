/*
 * graph_unittest.cpp
 *
 *  Created on: Jul 20, 2016
 *      Author: rdu
 */

#include <stdio.h>
#include <vector>

#include "gtest/gtest.h"

#include "planning/graph/graph.h"
#include "planning/graph/astar.h"
#include "planning/graph/bds_example.h"

using namespace librav;

struct TestState
{
	TestState(uint64_t id):any_unique_id_(id){};

	int64_t any_unique_id_;

	int64_t GetUniqueID() const
	{
		return any_unique_id_;
	}
};

struct GraphTemplateTest: testing::Test
{
	std::vector<TestState*> nodes;

	GraphTemplateTest()
	{
		for(int i = 0; i < 9; i++) {
			nodes.push_back(new TestState(i));
		}
	}

	virtual ~GraphTemplateTest()
	{
		for(auto& nd : nodes)
			delete nd;
	}
};

TEST_F(GraphTemplateTest, ValueType)
{
	// create a graph
	Graph_t<TestState> graph;

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
	graph.AddEdge(*(nodes[7]), *(nodes[4]), 2.5);
	graph.AddEdge(*(nodes[7]), *(nodes[8]), 2.5);
	graph.AddEdge(*(nodes[8]), *(nodes[5]), 2.5);
	graph.AddEdge(*(nodes[8]), *(nodes[7]), 2.5);

	ASSERT_NE(graph.GetGraphVertices().size(), 0) << "Failed to add a vertex of value type to the graph";
}

TEST_F(GraphTemplateTest, PointerType)
{
	Graph_t<TestState*> graph;

	graph.AddEdge((nodes[0]), (nodes[1]), 1.0);
	graph.AddEdge((nodes[0]), (nodes[3]), 1.5);
	graph.AddEdge((nodes[1]), (nodes[0]), 2.0);
	graph.AddEdge((nodes[1]), (nodes[4]), 2.5);
	graph.AddEdge((nodes[1]), (nodes[2]), 1.0);
	graph.AddEdge((nodes[2]), (nodes[1]), 1.5);
	graph.AddEdge((nodes[2]), (nodes[5]), 2.0);
	graph.AddEdge((nodes[3]), (nodes[0]), 2.5);
	graph.AddEdge((nodes[3]), (nodes[4]), 2.5);
	graph.AddEdge((nodes[4]), (nodes[1]), 2.5);
	graph.AddEdge((nodes[4]), (nodes[3]), 2.5);
	graph.AddEdge((nodes[4]), (nodes[5]), 2.5);
	graph.AddEdge((nodes[5]), (nodes[2]), 2.5);
	graph.AddEdge((nodes[5]), (nodes[4]), 2.5);
	graph.AddEdge((nodes[5]), (nodes[8]), 2.5);
	graph.AddEdge((nodes[7]), (nodes[4]), 2.5);
	graph.AddEdge((nodes[7]), (nodes[8]), 2.5);
	graph.AddEdge((nodes[8]), (nodes[5]), 2.5);
	graph.AddEdge((nodes[8]), (nodes[7]), 2.5);

	ASSERT_NE(graph.GetGraphVertices().size(), 0) << "Failed to add a vertex of pointer type to the graph";
}

TEST_F(GraphTemplateTest, ConstRefType)
{
	Graph_t<const TestState&> graph;

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
	graph.AddEdge(*(nodes[7]), *(nodes[4]), 2.5);
	graph.AddEdge(*(nodes[7]), *(nodes[8]), 2.5);
	graph.AddEdge(*(nodes[8]), *(nodes[5]), 2.5);
	graph.AddEdge(*(nodes[8]), *(nodes[7]), 2.5);

	ASSERT_NE(graph.GetGraphVertices().size(), 0) << "Failed to add a vertex of const reference type to the graph";
}
