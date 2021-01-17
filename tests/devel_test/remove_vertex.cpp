/*
 * remove_vertex.cpp
 *
 *  Created on: Dec 11, 2016
 *      Author: rdu
 */

// standard libaray
#include <iostream>
#include <vector>
#include <ctime>

// user
#include "graph/graph.h"
#include "graph/astar.h"
#include "demo/state_example.h"

using namespace rdu;

int main(int argc, char* argv[])
{
//	std::vector<StateExample*> nodes;
//
//	// create nodes
//	for(int i = 0; i < 9; i++) {
//		nodes.push_back(new StateExample(i));
//	}

	std::vector<StateExample> nodes;

	// create nodes
	for(int i = 0; i < 9; i++) {
		nodes.push_back(StateExample(i));
	}

	// create a graph
	Graph_t<StateExample> test_graph;

	test_graph.AddEdge(nodes[0], nodes[1], 1.0);
	test_graph.AddEdge(nodes[0], nodes[3], 1.5);
	test_graph.AddEdge(nodes[1], nodes[0], 2.0);
	test_graph.AddEdge(nodes[1], nodes[4], 2.5);
	test_graph.AddEdge(nodes[1], nodes[2], 1.0);
	test_graph.AddEdge(nodes[2], nodes[1], 1.5);
	test_graph.AddEdge(nodes[2], nodes[5], 2.0);
	test_graph.AddEdge(nodes[3], nodes[0], 2.5);
	test_graph.AddEdge(nodes[3], nodes[4], 2.5);
	test_graph.AddEdge(nodes[4], nodes[1], 2.5);
	test_graph.AddEdge(nodes[4], nodes[3], 2.5);
	test_graph.AddEdge(nodes[4], nodes[5], 2.5);
	test_graph.AddEdge(nodes[5], nodes[2], 2.5);
	test_graph.AddEdge(nodes[5], nodes[4], 2.5);
	test_graph.AddEdge(nodes[5], nodes[8], 2.5);
	test_graph.AddEdge(nodes[7], nodes[4], 2.5);
	test_graph.AddEdge(nodes[7], nodes[8], 2.5);
	test_graph.AddEdge(nodes[8], nodes[5], 2.5);
	test_graph.AddEdge(nodes[8], nodes[7], 2.5);

	std::cout << "graph edge num: " << test_graph.GetGraphEdges().size() << std::endl;
	for(auto& e : test_graph.GetGraphEdges())
		e.PrintEdge();

	std::cout << "---------------------- after remove ----------------------" << std::endl;
	test_graph.RemoveVertex(nodes[4]);

	std::cout << "graph edge num: " << test_graph.GetGraphEdges().size() << std::endl;
	for(auto& e : test_graph.GetGraphEdges())
		e.PrintEdge();

	// need to delete all nodes, the graph only maintains pointers to these nodes
//	for(auto e : nodes)
//		delete e;
}


