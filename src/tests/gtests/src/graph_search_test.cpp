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

#include "gtest/gtest.h"

#include "graph/graph.hpp"
#include "graph/algorithms/astar.hpp"
#include "graph/algorithms/dijkstra.hpp"

using namespace librav;

#define ROW_SIZE 4
#define COL_SIZE 4

struct SimpleState
{
	SimpleState(int64_t row, int64_t col) : row_(row),
											col_(col)
	{
	}

	SimpleState() = delete;

	int64_t row_;
	int64_t col_;

	int64_t GetUniqueID() const
	{
		// You can return the state id directly if you have one and it's unique (see StateExample class)
		// or you can use some kind of hash functions to generate one
		return row_ * COL_SIZE + col_;
	}
};

struct SimpleStateIndexer
{
	int64_t operator()(SimpleState *state)
	{
		return state->row_ * COL_SIZE + state->col_;
	}

	int64_t operator()(const SimpleState &state)
	{
		return state.row_ * COL_SIZE + state.col_;
	}
};

double CalcHeuristicPtr(SimpleState *node1, SimpleState *node2)
{
	int64_t dist_row = node1->row_ - node2->row_;
	int64_t dist_col = node1->col_ - node2->col_;

	return std::sqrt(dist_row * dist_row + dist_col * dist_col);
}

double CalcHeuristicVal(SimpleState node1, SimpleState node2)
{
	int64_t dist_row = node1.row_ - node2.row_;
	int64_t dist_col = node1.col_ - node2.col_;

	return std::sqrt(dist_row * dist_row + dist_col * dist_col);
}

struct GraphSearchTest : testing::Test
{
	std::vector<SimpleState *> nodes;
	std::vector<int64_t> spath;
	std::vector<int64_t> spath2;

	Graph<SimpleState, double, SimpleStateIndexer> graph_val;
	Graph<SimpleState *, double, SimpleStateIndexer> graph_ptr;

	GraphSearchTest()
	{
		for (int i = 0; i < ROW_SIZE; i++)
			for (int j = 0; j < COL_SIZE; j++)
				nodes.push_back(new SimpleState(i, j));

		spath.push_back(0);
		spath.push_back(1);
		spath.push_back(6);
		spath.push_back(9);
		spath.push_back(13);

		spath2.push_back(0);
		spath2.push_back(1);
		spath2.push_back(6);
		spath2.push_back(10);
		spath2.push_back(13);

		ConstructGraphs();
	}

	virtual ~GraphSearchTest()
	{
		for (auto &nd : nodes)
			delete nd;
	}

	void ConstructGraphs()
	{
		graph_val.AddEdge(*nodes[0], *nodes[1], 1.0);
		graph_val.AddEdge(*nodes[1], *nodes[0], 1.0);
		graph_val.AddEdge(*nodes[1], *nodes[2], 1.0);
		graph_val.AddEdge(*nodes[1], *nodes[6], 1.414);
		graph_val.AddEdge(*nodes[2], *nodes[1], 1.0);
		graph_val.AddEdge(*nodes[2], *nodes[3], 1.0);
		graph_val.AddEdge(*nodes[2], *nodes[6], 1.0);
		graph_val.AddEdge(*nodes[3], *nodes[2], 1.0);
		graph_val.AddEdge(*nodes[3], *nodes[6], 1.414);
		graph_val.AddEdge(*nodes[6], *nodes[1], 1.414);
		graph_val.AddEdge(*nodes[6], *nodes[2], 1.0);
		graph_val.AddEdge(*nodes[6], *nodes[3], 1.414);
		graph_val.AddEdge(*nodes[6], *nodes[9], 1.414);
		graph_val.AddEdge(*nodes[6], *nodes[10], 1.0);
		graph_val.AddEdge(*nodes[9], *nodes[6], 1.414);
		graph_val.AddEdge(*nodes[9], *nodes[10], 1.0);
		graph_val.AddEdge(*nodes[9], *nodes[13], 1.0);
		graph_val.AddEdge(*nodes[10], *nodes[6], 1.0);
		graph_val.AddEdge(*nodes[10], *nodes[9], 1.0);
		graph_val.AddEdge(*nodes[10], *nodes[13], 1.414);
		graph_val.AddEdge(*nodes[13], *nodes[9], 1.0);
		graph_val.AddEdge(*nodes[13], *nodes[10], 1.414);

		graph_ptr.AddEdge(nodes[0], nodes[1], 1.0);
		graph_ptr.AddEdge(nodes[1], nodes[0], 1.0);
		graph_ptr.AddEdge(nodes[1], nodes[2], 1.0);
		graph_ptr.AddEdge(nodes[1], nodes[6], 1.414);
		graph_ptr.AddEdge(nodes[2], nodes[1], 1.0);
		graph_ptr.AddEdge(nodes[2], nodes[3], 1.0);
		graph_ptr.AddEdge(nodes[2], nodes[6], 1.0);
		graph_ptr.AddEdge(nodes[3], nodes[2], 1.0);
		graph_ptr.AddEdge(nodes[3], nodes[6], 1.414);
		graph_ptr.AddEdge(nodes[6], nodes[1], 1.414);
		graph_ptr.AddEdge(nodes[6], nodes[2], 1.0);
		graph_ptr.AddEdge(nodes[6], nodes[3], 1.414);
		graph_ptr.AddEdge(nodes[6], nodes[9], 1.414);
		graph_ptr.AddEdge(nodes[6], nodes[10], 1.0);
		graph_ptr.AddEdge(nodes[9], nodes[6], 1.414);
		graph_ptr.AddEdge(nodes[9], nodes[10], 1.0);
		graph_ptr.AddEdge(nodes[9], nodes[13], 1.0);
		graph_ptr.AddEdge(nodes[10], nodes[6], 1.0);
		graph_ptr.AddEdge(nodes[10], nodes[9], 1.0);
		graph_ptr.AddEdge(nodes[10], nodes[13], 1.414);
		graph_ptr.AddEdge(nodes[13], nodes[9], 1.0);
		graph_ptr.AddEdge(nodes[13], nodes[10], 1.414);
	}
};

TEST_F(GraphSearchTest, ValueTypeDijkstra)
{
	Path<SimpleState> path = Dijkstra::Search(&graph_val, 0, 13);
	std::vector<int64_t> path_ids;
	for (auto &e : path)
		path_ids.push_back(e.GetUniqueID());

	ASSERT_TRUE(path_ids == spath || path_ids == spath2) << "Path found by Dijkstra in value-type graph is not correct";
}

TEST_F(GraphSearchTest, PointerTypeDijkstra)
{
	Path<SimpleState *> path = Dijkstra::Search(&graph_ptr, 0, 13);
	std::vector<int64_t> path_ids;
	for (auto &e : path)
		path_ids.push_back(e->GetUniqueID());

	ASSERT_TRUE(path_ids == spath || path_ids == spath2) << "Path found by Dijkstra in pointer-type graph is not correct";
}

TEST_F(GraphSearchTest, ValueTypeAStar)
{
	Path<SimpleState> path = AStar::Search(&graph_val, 0, 13, CalcHeuristicFunc_t<SimpleState>(CalcHeuristicVal));
	std::vector<int64_t> path_ids;
	for (auto &e : path)
		path_ids.push_back(e.GetUniqueID());

	ASSERT_TRUE(path_ids == spath || path_ids == spath2) << "Path found by A* in value-type graph is not correct";
}

TEST_F(GraphSearchTest, PointerTypeAStar)
{
	Path<SimpleState*> path = AStar::Search(&graph_ptr, 0, 13, CalcHeuristicFunc_t<SimpleState *>(CalcHeuristicPtr));
	std::vector<int64_t> path_ids;
	for (auto &e : path)
		path_ids.push_back(e->GetUniqueID());

	ASSERT_TRUE(path_ids == spath || path_ids == spath2) << "Path found by A* in pointer-type graph is not correct";
}
