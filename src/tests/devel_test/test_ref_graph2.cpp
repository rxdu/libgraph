// standard libaray
#include <iostream>
#include <vector>
#include <ctime>
#include <cmath>
#include <functional>

// user
#include "graph/graph.hpp"
#include "graph/algorithms/astar.hpp"
#include "graph/algorithms/dijkstra.hpp"

using namespace librav;

#define ROW_SIZE 4
#define COL_SIZE 4

class BasicState
{
  public:
    BasicState(int32_t row, int32_t col) : row_(row),
                                           col_(col)
    {
    }

    BasicState(const BasicState &s) : row_(s.row_),
                                      col_(s.col_)
    {
    }

    // BasicState(BasicState &) = default;
    // BasicState(const BasicState &) = default;
    // BasicState &operator=(const BasicState &) = default;

    int32_t row_ = -1;
    int32_t col_ = -1;

    int64_t GetUniqueID() const
    {
        // std::cout << "called (r,c): " << row_ << " , " << col_ << std::endl;

        // You can return the state id directly if you have one and it's unique (see StateExample class)
        // or you can use some kind of hash functions to generate one
        return row_ * COL_SIZE + col_;
    }
};

int main(int argc, char **argv)
{
    std::vector<BasicState> nodes;

    // create nodes
    for (int i = 0; i < ROW_SIZE; i++)
        for (int j = 0; j < COL_SIZE; j++)
            nodes.push_back(BasicState(i, j));

    // create a graph
    Graph_t<BasicState &> graph;

    graph.AddEdge(nodes[0], nodes[1], 1.0);
    graph.AddEdge(nodes[1], nodes[2], 2.0);
    graph.AddEdge(nodes[2], nodes[1], 3.0);

    auto all_edges = graph.GetAllEdges();
    for (auto &e : all_edges)
        e->PrintEdge();

    for (auto it = graph.vertex_begin(); it != graph.vertex_end(); ++it)
        std::cout << "checking final graph: state address " << std::hex << &(it->state_)
                  << std::dec << " , id: " << it->state_.GetUniqueID() << " , (row, col): " << it->state_.row_ << " , " << it->state_.col_ << std::endl;

    // need to delete all nodes, the graph only maintains pointers to these nodes
    // for (auto &e : nodes)
    //     delete e;

    return 0;
}