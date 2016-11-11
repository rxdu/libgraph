/*
 * simple_graph.cpp
 *
 *  Created on: Mar 30, 2016
 *      Author: rdu
 */


// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
//#include <tuple>

// user
#include "graph/graph.h"
#include "demo/bds_example.h"

using namespace srcl_ctrl;

int main(int argc, char** argv )
{
	std::vector<BDSExample*> nodes;

	// create nodes
	for(int i = 0; i < 9; i++) {
		nodes.push_back(new BDSExample(i));
	}

	// create a graph
	Graph_t<BDSExample*> graph;

//	graph.AddEdge(*(nodes[0]), *(nodes[1]), 1.0);
//	graph.AddEdge(*(nodes[0]), *(nodes[3]), 1.5);
//	graph.AddEdge(*(nodes[1]), *(nodes[0]), 2.0);
//	graph.AddEdge(*(nodes[1]), *(nodes[4]), 2.5);
//	graph.AddEdge(*(nodes[1]), *(nodes[2]), 1.0);
//	graph.AddEdge(*(nodes[2]), *(nodes[1]), 1.5);
//	graph.AddEdge(*(nodes[2]), *(nodes[5]), 2.0);
//	graph.AddEdge(*(nodes[3]), *(nodes[0]), 2.5);
//	graph.AddEdge(*(nodes[3]), *(nodes[4]), 2.5);
//	graph.AddEdge(*(nodes[4]), *(nodes[1]), 2.5);
//	graph.AddEdge(*(nodes[4]), *(nodes[3]), 2.5);
//	graph.AddEdge(*(nodes[4]), *(nodes[5]), 2.5);
//	graph.AddEdge(*(nodes[5]), *(nodes[2]), 2.5);
//	graph.AddEdge(*(nodes[5]), *(nodes[4]), 2.5);
//	graph.AddEdge(*(nodes[5]), *(nodes[8]), 2.5);
//	graph.AddEdge(*(nodes[7]), *(nodes[4]), 2.5);
//	graph.AddEdge(*(nodes[7]), *(nodes[8]), 2.5);
//	graph.AddEdge(*(nodes[8]), *(nodes[5]), 2.5);
//	graph.AddEdge(*(nodes[8]), *(nodes[7]), 2.5);
	graph.AddEdge(nodes[0], nodes[1], 1.0);
	graph.AddEdge(nodes[0], nodes[3], 1.5);
	graph.AddEdge(nodes[1], nodes[0], 2.0);
	graph.AddEdge(nodes[1], nodes[4], 2.5);
	graph.AddEdge(nodes[1], nodes[2], 1.0);
	graph.AddEdge(nodes[2], nodes[1], 1.5);
	graph.AddEdge(nodes[2], nodes[5], 2.0);
	graph.AddEdge(nodes[3], nodes[0], 2.5);
	graph.AddEdge(nodes[3], nodes[4], 2.5);
	graph.AddEdge(nodes[4], nodes[1], 2.5);
	graph.AddEdge(nodes[4], nodes[3], 2.5);
	graph.AddEdge(nodes[4], nodes[5], 2.5);
	graph.AddEdge(nodes[5], nodes[2], 2.5);
	graph.AddEdge(nodes[5], nodes[4], 2.5);
	graph.AddEdge(nodes[5], nodes[8], 2.5);
	graph.AddEdge(nodes[7], nodes[4], 2.5);
	graph.AddEdge(nodes[7], nodes[8], 2.5);
	graph.AddEdge(nodes[8], nodes[5], 2.5);
	graph.AddEdge(nodes[8], nodes[7], 2.5);

	auto all_edges = graph.GetGraphEdges();
	for(auto& e : all_edges)
		e.PrintEdge();

	graph.RemoveEdge(nodes[8], nodes[7]);
	graph.RemoveEdge(nodes[0], nodes[3]);

	std::cout << "-------------------------------" << std::endl;
	auto new_edges = graph.GetGraphEdges();
	for(auto& e : new_edges)
		e.PrintEdge();

//	std::cout << "test a*" << std::endl;
//	auto path = graph.AStarSearch(graph.GetVertexFromID(0), graph.GetVertexFromID(8));
//
//	for(auto& e : path)
//		std::cout << "id: " << e->vertex_id_ << std::endl;

	// delete all nodes
	for(auto e : nodes)
		delete e;

	return 0;
}
