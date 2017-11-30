/*
 * graph_demo.cpp
 *
 *  Created on: Mar 30, 2016
 *      Author: rdu
 *
 *  Description: demo on how to create different types of graphs and perform A* search on each.
 *
 */


// standard libaray
#include <iostream>
#include <vector>
#include <ctime>

// user
#include "graph/graph.hpp"
#include "graph/algorithms/astar.hpp"
#include "state_example.hpp"

using namespace librav;

void ValueTypeGraphDemo();
void PointerTypeGraphDemo();
void ConstRefTypeGraphDemo();

double CalcHeuristicVal(StateExample node1, StateExample node2)
{
	return 0.0;
}

double CalcHeuristicPtr(StateExample* node1, StateExample* node2)
{
	return 0.0;
}

double CalcHeuristicRef(const StateExample& node1, const StateExample& node2)
{
	return 0.0;
}

void ValueTypeGraphDemo()
{
	std::vector<StateExample> nodes;

	// create nodes
	for(int i = 0; i < 9; i++) {
		nodes.push_back(StateExample(i));
	}

	// create a graph
	Graph_t<StateExample> graph_val;

	graph_val.AddEdge(nodes[0], nodes[1], 1.0);
	graph_val.AddEdge(nodes[0], nodes[3], 1.5);
	graph_val.AddEdge(nodes[1], nodes[0], 2.0);
	graph_val.AddEdge(nodes[1], nodes[4], 2.5);
	graph_val.AddEdge(nodes[1], nodes[2], 1.0);
	graph_val.AddEdge(nodes[2], nodes[1], 1.5);
	graph_val.AddEdge(nodes[2], nodes[5], 2.0);
	graph_val.AddEdge(nodes[3], nodes[0], 2.5);
	graph_val.AddEdge(nodes[3], nodes[4], 2.5);
	graph_val.AddEdge(nodes[4], nodes[1], 2.5);
	graph_val.AddEdge(nodes[4], nodes[3], 2.5);
	graph_val.AddEdge(nodes[4], nodes[5], 2.5);
	graph_val.AddEdge(nodes[5], nodes[2], 2.5);
	graph_val.AddEdge(nodes[5], nodes[4], 2.5);
	graph_val.AddEdge(nodes[5], nodes[8], 2.5);
	graph_val.AddEdge(nodes[7], nodes[4], 2.5);
	graph_val.AddEdge(nodes[7], nodes[8], 2.5);
	graph_val.AddEdge(nodes[8], nodes[5], 2.5);
	graph_val.AddEdge(nodes[8], nodes[7], 2.5);

	auto all_edges = graph_val.GetGraphEdges();

	for(auto& e : all_edges)
		e.PrintEdge();

	auto path = AStar::Search(graph_val, graph_val.GetVertexFromID(0), graph_val.GetVertexFromID(8), CalcHeuristicFunc_t<StateExample>(CalcHeuristicVal));

	for(auto& e : path)
		std::cout << "id: " << e->vertex_id_ << std::endl;

	// no need to deallocate memory, all data structures are copied to graph
}

void PointerTypeGraphDemo()
{
	std::vector<StateExample*> nodes;

	// create nodes
	for(int i = 0; i < 9; i++) {
		nodes.push_back(new StateExample(i));
	}

	// create a graph
	Graph_t<StateExample*> graph_ptr;

	graph_ptr.AddEdge(nodes[0], nodes[1], 1.0);
	graph_ptr.AddEdge(nodes[0], nodes[3], 1.5);
	graph_ptr.AddEdge(nodes[1], nodes[0], 2.0);
	graph_ptr.AddEdge(nodes[1], nodes[4], 2.5);
	graph_ptr.AddEdge(nodes[1], nodes[2], 1.0);
	graph_ptr.AddEdge(nodes[2], nodes[1], 1.5);
	graph_ptr.AddEdge(nodes[2], nodes[5], 2.0);
	graph_ptr.AddEdge(nodes[3], nodes[0], 2.5);
	graph_ptr.AddEdge(nodes[3], nodes[4], 2.5);
	graph_ptr.AddEdge(nodes[4], nodes[1], 2.5);
	graph_ptr.AddEdge(nodes[4], nodes[3], 2.5);
	graph_ptr.AddEdge(nodes[4], nodes[5], 2.5);
	graph_ptr.AddEdge(nodes[5], nodes[2], 2.5);
	graph_ptr.AddEdge(nodes[5], nodes[4], 2.5);
	graph_ptr.AddEdge(nodes[5], nodes[8], 2.5);
	graph_ptr.AddEdge(nodes[7], nodes[4], 2.5);
	graph_ptr.AddEdge(nodes[7], nodes[8], 2.5);
	graph_ptr.AddEdge(nodes[8], nodes[5], 2.5);
	graph_ptr.AddEdge(nodes[8], nodes[7], 2.5);

	auto all_edges = graph_ptr.GetGraphEdges();

	for(auto& e : all_edges)
		e.PrintEdge();

	auto path = AStar::Search(graph_ptr, graph_ptr.GetVertexFromID(0), graph_ptr.GetVertexFromID(8),CalcHeuristicFunc_t<StateExample*>(CalcHeuristicPtr));

	for(auto& e : path)
		std::cout << "id: " << e->vertex_id_ << std::endl;

	// need to delete all nodes, the graph only maintains pointers to these nodes
	for(auto e : nodes)
		delete e;
}

void ConstRefTypeGraphDemo()
{
	std::vector<StateExample> nodes;

	// create nodes
	for(int i = 0; i < 9; i++) {
		nodes.push_back(StateExample(i));
	}

	// create a graph
	Graph_t<const StateExample&> graph_ptr;

	graph_ptr.AddEdge((nodes[0]), (nodes[1]), 1.0);
	graph_ptr.AddEdge((nodes[0]), (nodes[3]), 1.5);
	graph_ptr.AddEdge((nodes[1]), (nodes[0]), 2.0);
	graph_ptr.AddEdge((nodes[1]), (nodes[4]), 2.5);
	graph_ptr.AddEdge((nodes[1]), (nodes[2]), 1.0);
	graph_ptr.AddEdge((nodes[2]), (nodes[1]), 1.5);
	graph_ptr.AddEdge((nodes[2]), (nodes[5]), 2.0);
	graph_ptr.AddEdge((nodes[3]), (nodes[0]), 2.5);
	graph_ptr.AddEdge((nodes[3]), (nodes[4]), 2.5);
	graph_ptr.AddEdge((nodes[4]), (nodes[1]), 2.5);
	graph_ptr.AddEdge((nodes[4]), (nodes[3]), 2.5);
	graph_ptr.AddEdge((nodes[4]), (nodes[5]), 2.5);
	graph_ptr.AddEdge((nodes[5]), (nodes[2]), 2.5);
	graph_ptr.AddEdge((nodes[5]), (nodes[4]), 2.5);
	graph_ptr.AddEdge((nodes[5]), (nodes[8]), 2.5);
	graph_ptr.AddEdge((nodes[7]), (nodes[4]), 2.5);
	graph_ptr.AddEdge((nodes[7]), (nodes[8]), 2.5);
	graph_ptr.AddEdge((nodes[8]), (nodes[5]), 2.5);
	graph_ptr.AddEdge((nodes[8]), (nodes[7]), 2.5);

	auto all_edges = graph_ptr.GetGraphEdges();

	for(auto& e : all_edges)
		e.PrintEdge();

	auto path = AStar::Search(graph_ptr, graph_ptr.GetVertexFromID(0), graph_ptr.GetVertexFromID(8),CalcHeuristicFunc_t<const StateExample&>(CalcHeuristicRef));

	for(auto& e : path)
		std::cout << "id: " << e->vertex_id_ << std::endl;

	// need to delete all nodes, the graph only maintains references to these nodes
	// for(auto e : nodes)
	// 	delete e;
}

int main(int argc, char** argv )
{
	std::cout << "\n------------- value type graph -------------\n" << std::endl;
	ValueTypeGraphDemo();

	std::cout << "\n------------- pointer type graph -------------\n" << std::endl;
	PointerTypeGraphDemo();

	std::cout << "\n------------- const reference type graph -------------\n" << std::endl;
	ConstRefTypeGraphDemo();

	return 0;
}
