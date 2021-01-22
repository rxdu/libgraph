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
#include "demo/state_example.h"

using namespace rdu;

std::vector<std::tuple<StateExample,double>> GetNeighbour(StateExample node)
{
	std::vector<std::tuple<StateExample,double>> empty;

	switch(node.GetUniqueID())
	{
	case 0:
		empty.emplace_back(StateExample(1),1.0);
		empty.emplace_back(StateExample(3),1.0);
		break;
	case 1:
		empty.emplace_back(StateExample(0),1.0);
		empty.emplace_back(StateExample(4),1.0);
		empty.emplace_back(StateExample(2),1.0);
		break;
	case 2:
		empty.emplace_back(StateExample(1),1.0);
		empty.emplace_back(StateExample(5),1.0);
		break;
	case 3:
		empty.emplace_back(StateExample(0),1.0);
		empty.emplace_back(StateExample(4),1.0);
		break;
	case 4:
		empty.emplace_back(StateExample(1),1.0);
		empty.emplace_back(StateExample(3),1.0);
		empty.emplace_back(StateExample(5),1.0);
		break;
	case 5:
		empty.emplace_back(StateExample(2),1.0);
		empty.emplace_back(StateExample(4),1.0);
		empty.emplace_back(StateExample(8),1.0);
		break;
	case 7:
		empty.emplace_back(StateExample(4),1.0);
		empty.emplace_back(StateExample(8),1.0);
		break;
	case 8:
		empty.emplace_back(StateExample(5),1.0);
		empty.emplace_back(StateExample(7),1.0);
		break;
	}

	return empty;
}

std::vector<std::tuple<StateExample,double>> GetNeighbour2(StateExample node)
{
	std::vector<std::tuple<StateExample,double>> empty;

	switch(node.GetUniqueID())
	{
	case 0:
		empty.emplace_back(StateExample(1),1.0);
		empty.emplace_back(StateExample(3),1.5);
		break;
	case 1:
		empty.emplace_back(StateExample(0),2.0);
		empty.emplace_back(StateExample(4),2.5);
		empty.emplace_back(StateExample(2),1.0);
		break;
	case 2:
		empty.emplace_back(StateExample(1),1.5);
		empty.emplace_back(StateExample(5),2.0);
		break;
	case 3:
		empty.emplace_back(StateExample(0),2.5);
		empty.emplace_back(StateExample(4),2.5);
		break;
	case 4:
		empty.emplace_back(StateExample(1),2.5);
		empty.emplace_back(StateExample(3),2.5);
		empty.emplace_back(StateExample(5),2.5);
		break;
	case 5:
		empty.emplace_back(StateExample(2),2.5);
		empty.emplace_back(StateExample(4),2.5);
		empty.emplace_back(StateExample(8),2.5);
		break;
	case 7:
		empty.emplace_back(StateExample(4),2.5);
		empty.emplace_back(StateExample(8),2.5);
		break;
	case 8:
		empty.emplace_back(StateExample(5),2.5);
		empty.emplace_back(StateExample(7),2.5);
		break;
	}

	return empty;
}

int main(int argc, char** argv )
{
	std::vector<StateExample> nodes;

	// create nodes
	for(int i = 0; i < 9; i++) {
		nodes.push_back(StateExample(i));
	}

	// create a graph
//	Graph_t<StateExample> graph_val;
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

//	auto path = AStar::IncSearch(StateExample(0), StateExample(8), std::function<std::vector<std::tuple<StateExample,double>>(StateExample)>(GetNeighbour2));
//	AStar astar;
	auto path = AStar::IncSearch(StateExample(0), StateExample(8), GetNeighbourBDSFunc_t<StateExample>(GetNeighbour2));

	for(auto& e : path)
		std::cout << "id: " << e.GetUniqueID() << std::endl;

	return 0;
}
