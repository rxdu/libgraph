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
#include <tuple>

// user
#include "graph/graph.h"
#include "graph/astar.h"
#include "demo/bds_example.h"

using namespace srcl_ctrl;

std::vector<std::tuple<BDSExample,double>> GetNeighbour(BDSExample node)
{
	std::vector<std::tuple<BDSExample,double>> empty;

	switch(node.data_id_)
	{
	case 0:
		empty.emplace_back(BDSExample(1),1.0);
		empty.emplace_back(BDSExample(3),1.0);
		break;
	case 1:
		empty.emplace_back(BDSExample(0),1.0);
		empty.emplace_back(BDSExample(4),1.0);
		empty.emplace_back(BDSExample(2),1.0);
		break;
	case 2:
		empty.emplace_back(BDSExample(1),1.0);
		empty.emplace_back(BDSExample(5),1.0);
		break;
	case 3:
		empty.emplace_back(BDSExample(0),1.0);
		empty.emplace_back(BDSExample(4),1.0);
		break;
	case 4:
		empty.emplace_back(BDSExample(1),1.0);
		empty.emplace_back(BDSExample(3),1.0);
		empty.emplace_back(BDSExample(5),1.0);
		break;
	case 5:
		empty.emplace_back(BDSExample(2),1.0);
		empty.emplace_back(BDSExample(4),1.0);
		empty.emplace_back(BDSExample(8),1.0);
		break;
	case 7:
		empty.emplace_back(BDSExample(4),1.0);
		empty.emplace_back(BDSExample(8),1.0);
		break;
	case 8:
		empty.emplace_back(BDSExample(5),1.0);
		empty.emplace_back(BDSExample(7),1.0);
		break;
	}

	return empty;
}

std::vector<std::tuple<BDSExample,double>> GetNeighbour2(BDSExample node)
{
	std::vector<std::tuple<BDSExample,double>> empty;

	switch(node.data_id_)
	{
	case 0:
		empty.emplace_back(BDSExample(1),1.0);
		empty.emplace_back(BDSExample(3),1.5);
		break;
	case 1:
		empty.emplace_back(BDSExample(0),2.0);
		empty.emplace_back(BDSExample(4),2.5);
		empty.emplace_back(BDSExample(2),1.0);
		break;
	case 2:
		empty.emplace_back(BDSExample(1),1.5);
		empty.emplace_back(BDSExample(5),2.0);
		break;
	case 3:
		empty.emplace_back(BDSExample(0),2.5);
		empty.emplace_back(BDSExample(4),2.5);
		break;
	case 4:
		empty.emplace_back(BDSExample(1),2.5);
		empty.emplace_back(BDSExample(3),2.5);
		empty.emplace_back(BDSExample(5),2.5);
		break;
	case 5:
		empty.emplace_back(BDSExample(2),2.5);
		empty.emplace_back(BDSExample(4),2.5);
		empty.emplace_back(BDSExample(8),2.5);
		break;
	case 7:
		empty.emplace_back(BDSExample(4),2.5);
		empty.emplace_back(BDSExample(8),2.5);
		break;
	case 8:
		empty.emplace_back(BDSExample(5),2.5);
		empty.emplace_back(BDSExample(7),2.5);
		break;
	}

	return empty;
}

int main(int argc, char** argv )
{
	std::vector<BDSExample> nodes;

	// create nodes
	for(int i = 0; i < 9; i++) {
		nodes.push_back(BDSExample(i));
	}

	// create a graph
//	Graph_t<BDSExample> graph_val;
//
//	graph_val.AddEdge(nodes[0], nodes[1], 1.0);
//	graph_val.AddEdge(nodes[0], nodes[3], 1.0);
//	graph_val.AddEdge(nodes[1], nodes[0], 1.0);
//	graph_val.AddEdge(nodes[1], nodes[4], 1.0);
//	graph_val.AddEdge(nodes[1], nodes[2], 1.0);
//	graph_val.AddEdge(nodes[2], nodes[1], 1.0);
//	graph_val.AddEdge(nodes[2], nodes[5], 1.0);
//	graph_val.AddEdge(nodes[3], nodes[0], 1.0);
//	graph_val.AddEdge(nodes[3], nodes[4], 1.0);
//	graph_val.AddEdge(nodes[4], nodes[1], 1.0);
//	graph_val.AddEdge(nodes[4], nodes[3], 1.0);
//	graph_val.AddEdge(nodes[4], nodes[5], 1.0);
//	graph_val.AddEdge(nodes[5], nodes[2], 1.0);
//	graph_val.AddEdge(nodes[5], nodes[4], 1.0);
//	graph_val.AddEdge(nodes[5], nodes[8], 1.0);
//	graph_val.AddEdge(nodes[7], nodes[4], 1.0);
//	graph_val.AddEdge(nodes[7], nodes[8], 1.0);
//	graph_val.AddEdge(nodes[8], nodes[5], 1.0);
//	graph_val.AddEdge(nodes[8], nodes[7], 1.0);
//
//	auto all_edges = graph_val.GetGraphEdges();
//
//	for(auto& e : all_edges)
//		e.PrintEdge();

//	auto path = AStar::IncSearch(BDSExample(0), BDSExample(8), std::function<std::vector<std::tuple<BDSExample,double>>(BDSExample)>(GetNeighbour2));
//	AStar astar;
	auto path = AStar::IncSearch(BDSExample(0), BDSExample(8), GetNeighbourBDSFunc_t<BDSExample>(GetNeighbour2));

	for(auto& e : path)
		std::cout << "id: " << e->vertex_id_ << std::endl;

	return 0;
}
