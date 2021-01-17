// standard libaray
#include <iostream>
#include <vector>
#include <ctime>
#include <cmath>

// user
#include "graph/graph.hpp"
#include "graph/algorithms/astar.hpp"
#include "graph/algorithms/dijkstra.hpp"

using namespace rdu;

#define ROW_SIZE 4
#define COL_SIZE 4

class BasicState
{
  public:
    BasicState(int32_t row, int32_t col) : row_(row),
                                           col_(col)
    {
        my_id = id++;
    }

    BasicState(BasicState &) = default;
    BasicState(const BasicState &) = default;
    BasicState &operator=(const BasicState &) = default;

    int32_t row_ = -1;
    int32_t col_ = -1;

    static int32_t id;
    int32_t my_id;

    int64_t GetUniqueID() const
    {
        // std::cout << "called (r,c): " << row_ << " , " << col_ << std::endl;

        // You can return the state id directly if you have one and it's unique (see StateExample class)
        // or you can use some kind of hash functions to generate one
        // return row_ * COL_SIZE + col_;
        return this->my_id;
    }
};

int32_t BasicState::id = 0;

double CalcHeuristic(const BasicState &node1, const BasicState &node2)
{
    int32_t dist_row = node1.row_ - node2.row_;
    int32_t dist_col = node1.col_ - node2.col_;

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
    Graph_t<const BasicState *> graph;

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

    // auto all_edges = graph.GetAllEdges();
    // for (auto &e : all_edges)
    //     e->PrintEdge();

    // for (auto it = graph.vertex_begin(); it != graph.vertex_end(); ++it)
    //     std::cout << "checking graph3: " << it->state_.GetUniqueID() << std::endl;

    auto path = Dijkstra::Search(&graph, 0, 13);
    std::vector<int64_t> path_ids;
    for (auto e : path)
    {
        path_ids.push_back(e->GetUniqueID());
        std::cout << "ref path dijkstra state id: " << e->GetUniqueID() << std::endl;
    }
    std::cout << "ref path length: " << path.size() << std::endl;

    // need to delete all nodes, the graph only maintains pointers to these nodes
    for (auto &e : nodes)
        delete e;

    return 0;
}