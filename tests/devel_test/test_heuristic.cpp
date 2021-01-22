// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>

// user
#include "graph/graph.hpp"
#include "graph/algorithms/astar.hpp"
#include "demo/state_example.hpp"

using namespace rdu;

double CalcHeuristic(StateExample node1, StateExample node2)
{
	return 0.0;
}

int main(int argc, char** argv )
{
	std::vector<StateExample> nodes;

	// create nodes
	for(int i = 0; i < 9; i++) {
		nodes.push_back(StateExample(i));
	}

	// create a graph
	Graph_t<StateExample> graph_val;

	graph_val.AddEdge(nodes[0], nodes[1], 1.0);
	graph_val.AddEdge(nodes[0], nodes[3], 1.0);
	graph_val.AddEdge(nodes[1], nodes[0], 1.0);
	graph_val.AddEdge(nodes[1], nodes[4], 1.0);
	graph_val.AddEdge(nodes[1], nodes[2], 1.0);
	graph_val.AddEdge(nodes[2], nodes[1], 1.0);
	graph_val.AddEdge(nodes[2], nodes[5], 1.0);
	graph_val.AddEdge(nodes[3], nodes[0], 1.0);
	graph_val.AddEdge(nodes[3], nodes[4], 1.0);
	graph_val.AddEdge(nodes[4], nodes[1], 1.0);
	graph_val.AddEdge(nodes[4], nodes[3], 1.0);
	graph_val.AddEdge(nodes[4], nodes[5], 1.0);
	graph_val.AddEdge(nodes[5], nodes[2], 1.0);
	graph_val.AddEdge(nodes[5], nodes[4], 1.0);
	graph_val.AddEdge(nodes[5], nodes[8], 1.0);
	graph_val.AddEdge(nodes[7], nodes[4], 1.0);
	graph_val.AddEdge(nodes[7], nodes[8], 1.0);
	graph_val.AddEdge(nodes[8], nodes[5], 1.0);
	graph_val.AddEdge(nodes[8], nodes[7], 1.0);

	auto all_edges = graph_val.GetAllEdges();

	for(auto& e : all_edges)
		e->PrintEdge();

	auto path = AStar::Search(&graph_val, 0, 8, CalcHeuristicFunc_t<StateExample>(CalcHeuristic));

	for(auto& e : path)
		std::cout << "id: " << e.GetUniqueID() << std::endl;

	return 0;
}