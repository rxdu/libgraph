// standard libaray
#include <iostream>
#include <vector>
#include <ctime>
#include <memory>

// user
#include "graph/tree.hpp"

using namespace rdu;

struct StateExample
{
    StateExample(uint64_t id) : id_(id){};

    int64_t id_;

    bool operator==(const StateExample &other)
    {
        if (id_ == other.id_)
            return true;
        else
            return false;
    }
};

int main(int argc, char **argv)
{
    std::vector<StateExample *> nodes;

    // create nodes
    for (int i = 0; i < 9; i++)
    {
        nodes.push_back(new StateExample(i));
    }

    // create a graph
    Tree<StateExample *> tree;

    tree.AddEdge(nodes[0], nodes[1], 1.0);
    tree.AddEdge(nodes[0], nodes[3], 1.5);
    tree.AddEdge(nodes[1], nodes[4], 2.0);
    tree.AddEdge(nodes[2], nodes[5], 2.5);
    tree.AddEdge(nodes[2], nodes[6], 2.5);

    auto all_edges = tree.GetAllEdges();

    for (auto &e : all_edges)
        e->PrintEdge();

    for (auto it = tree.vertex_begin(); it != tree.vertex_end(); ++it)
    {
        for (auto eit = it->edge_begin(); eit != it->edge_end(); ++eit)
        {
            eit->PrintEdge();
        }
    }

    auto root = tree.GetRootVertex();
    if (root != tree.vertex_end())
        std::cout << "root id: " << root->GetVertexID() << std::endl;
    else
        std::cout << "root not set" << std::endl;

    // tree.RemoveEdge(nodes[3], nodes[0]);

    // std::cout << "after change" << std::endl;

    // for (auto it = tree.vertex_begin(); it != tree.vertex_end(); ++it)
    // {
    //     for (auto eit = it->edge_begin(); eit != it->edge_end(); ++eit)
    //     {
    //         eit->PrintEdge();
    //     }
    // }

    // need to delete all nodes, the graph only maintains pointers to these nodes
    for (auto e : nodes)
        delete e;

    return 0;
}
