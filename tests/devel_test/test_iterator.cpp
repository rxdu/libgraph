// standard libaray
#include <iostream>
#include <vector>
#include <ctime>
#include <cmath>

// user
#include "graph/graph.hpp"
#include "graph/algorithms/astar.hpp"
#include "graph/algorithms/dijkstra.hpp"

using namespace xmotion;

#define ROW_SIZE 4
#define COL_SIZE 4

struct BasicState
{
    BasicState(int32_t row, int32_t col) : row_(row),
                                           col_(col)
    {
    }

    int32_t row_;
    int32_t col_;

    int64_t GetUniqueID() const
    {
        // You can return the state id directly if you have one and it's unique (see StateExample class)
        // or you can use some kind of hash functions to generate one
        return row_ * COL_SIZE + col_;
    }
};

double
CalcHeuristic(BasicState *node1, BasicState *node2)
{
    int32_t dist_row = node1->row_ - node2->row_;
    int32_t dist_col = node1->col_ - node2->col_;

    return std::sqrt(dist_row * dist_row + dist_col * dist_col);
}

int main(int argc, char **argv)
{
    std::vector<BasicState *> nodes;

    // create nodes
    for (int i = 0; i < ROW_SIZE; i++)
        for (int j = 0; j < COL_SIZE; j++)
            nodes.push_back(new BasicState(i, j));

    // create a graph
    Graph_t<BasicState *> graph;

    graph.AddEdge(nodes[0], nodes[1], 1.0);
    graph.AddEdge(nodes[1], nodes[0], 1.0);
    graph.AddEdge(nodes[1], nodes[2], 1.0);
    graph.AddEdge(nodes[1], nodes[6], 1.414);
    graph.AddEdge(nodes[2], nodes[1], 1.0);
    graph.AddEdge(nodes[2], nodes[3], 1.0);
    graph.AddEdge(nodes[2], nodes[6], 1.0);
    graph.AddEdge(nodes[3], nodes[2], 1.0);
    graph.AddEdge(nodes[3], nodes[6], 1.414);
    graph.AddEdge(nodes[6], nodes[1], 1.414);
    graph.AddEdge(nodes[6], nodes[2], 1.0);
    graph.AddEdge(nodes[6], nodes[3], 1.414);
    graph.AddEdge(nodes[6], nodes[9], 1.414);
    graph.AddEdge(nodes[6], nodes[10], 1.0);
    graph.AddEdge(nodes[9], nodes[6], 1.414);
    graph.AddEdge(nodes[9], nodes[10], 1.0);
    graph.AddEdge(nodes[9], nodes[13], 1.0);
    graph.AddEdge(nodes[10], nodes[6], 1.0);
    graph.AddEdge(nodes[10], nodes[9], 1.0);
    graph.AddEdge(nodes[10], nodes[13], 1.414);
    graph.AddEdge(nodes[13], nodes[9], 1.0);
    graph.AddEdge(nodes[13], nodes[10], 1.414);

    auto all_edges = graph.GetAllEdges();

    for (auto &e : all_edges)
        e->PrintEdge();

    std::vector<Graph_t<BasicState *>::vertex_iterator> vextices;
    std::vector<Graph_t<BasicState *>::edge_iterator> edges;

    auto vtx = graph.find(1);
    for (auto it = vtx->edge_begin(); it != vtx->edge_end(); ++it)
    {
        std::cout << "edge iterator * : " << (*it).dst_->vertex_id_ << std::endl;
        std::cout << "edge iterator -> : " << it->dst_->vertex_id_ << std::endl;
        edges.push_back(it);
    }

    for (auto it = graph.vertex_begin(); it != graph.vertex_end(); ++it)
    {
        std::cout << "vertex iterator * : " << (*it).vertex_id_ << std::endl;
        std::cout << "vertex iterator -> : " << it->vertex_id_ << std::endl;
    }

    for (auto it = graph.vertex_begin(); it != graph.vertex_end(); ++it)
    {
        std::cout << "edges of vertex: " << (*it).vertex_id_ << std::endl;
        for (auto ite = it->edge_begin(); ite != it->edge_end(); ++ite)
            std::cout << "edge " << (*ite).dst_->vertex_id_ << std::endl;
    }

    // need to delete all nodes, the graph only maintains pointers to these nodes
    for (auto &e : nodes)
        delete e;

    return 0;
}