/* 
 * graph_iter_test.cpp
 * 
 * Created on: Mar 14, 2018 14:05
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include <stdio.h>
#include <vector>
#include <set>

#include "gtest/gtest.h"

#include "graph/graph.hpp"

using namespace librav;

struct TestState
{
    TestState(uint64_t id) : id_(id){};

    int64_t id_;
};

struct GraphIteratorTest : testing::Test
{
    std::vector<TestState *> nodes;
    Graph<TestState *> graph;

    std::set<int64_t> vertex_id_set;
    std::set<double> edge_cost_set;

    GraphIteratorTest()
    {
        for (int i = 0; i < 9; i++)
        {
            nodes.push_back(new TestState(i));
            vertex_id_set.insert(i);
        }

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
        graph.AddEdge((nodes[5]), (nodes[6]), 8.5);
        graph.AddEdge((nodes[7]), (nodes[4]), 9.0);
        graph.AddEdge((nodes[7]), (nodes[8]), 9.5);
        graph.AddEdge((nodes[8]), (nodes[5]), 10.0);
        graph.AddEdge((nodes[8]), (nodes[7]), 10.5);

        for (int i = 0; i < 20; ++i)
            edge_cost_set.insert(1.0 + 0.5 * i);
    }

    virtual ~GraphIteratorTest()
    {
        for (auto &nd : nodes)
            delete nd;
    }
};

TEST_F(GraphIteratorTest, VertexEdgeIterator)
{
    std::set<int64_t> vertex_ids;
    std::set<double> edge_costs;

    for (auto it = graph.vertex_begin(); it != graph.vertex_end(); ++it)
    {
        vertex_ids.insert((*it).vertex_id_);
        for (auto ite = (*it).edge_begin(); ite != (*it).edge_end(); ++ite)
            edge_costs.insert((*ite).cost_);
    }

    ASSERT_TRUE(vertex_ids == vertex_id_set) << "Failed to access all vertices in the graph (with iterator * operator)";
    ASSERT_TRUE(edge_costs == edge_cost_set) << "Failed to access all edges in the graph (with iterator * operator)";

    vertex_ids.clear();
    edge_costs.clear();
    for (auto it = graph.vertex_begin(); it != graph.vertex_end(); ++it)
    {
        vertex_ids.insert(it->vertex_id_);
        for (auto ite = it->edge_begin(); ite != it->edge_end(); ++ite)
            edge_costs.insert(ite->cost_);
    }

    ASSERT_TRUE(vertex_ids == vertex_id_set) << "Failed to access all vertices in the graph (with iterator -> operator)";
    ASSERT_TRUE(edge_costs == edge_cost_set) << "Failed to access all edges in the graph (with iterator -> operator)";

    Graph<TestState *>::const_vertex_iterator cbegin_vtx = graph.vertex_begin();
    Graph<TestState *>::const_edge_iterator cbegin_edge = cbegin_vtx->edge_begin();

    ASSERT_TRUE(cbegin_vtx->vertex_id_ == graph.vertex_begin()->vertex_id_) << "Failed to access const vertex iterator)";
}