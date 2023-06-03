// standard libaray
#include <iostream>
#include <vector>
#include <ctime>
#include <memory>

// user
#include "graph/tree.hpp"

using namespace xmotion;

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

void ValueTypeGraphDemo();
void PointerTypeGraphDemo();
void SmartPointerTypeGraphDemo();

void ValueTypeGraphDemo()
{
	std::vector<StateExample> nodes;

	// create nodes
	for (int i = 0; i < 9; i++)
	{
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

	auto all_edges = graph_val.GetAllEdges();

	for (auto &e : all_edges)
		e->PrintEdge();

	// no need to deallocate memory, all data structures are copied to graph
}

void PointerTypeGraphDemo()
{
	std::vector<StateExample *> nodes;

	// create nodes
	for (int i = 0; i < 9; i++)
	{
		nodes.push_back(new StateExample(i));
	}

	// create a graph
	// Graph<StateExample *, double, IndexFunction> graph_ptr;
	Graph<StateExample *> graph_ptr;

	graph_ptr.AddEdge(nodes[0], nodes[1], 1.0);
	graph_ptr.AddEdge(nodes[0], nodes[3], 1.5);
	graph_ptr.AddEdge(nodes[1], nodes[0], 2.0);
	// graph_ptr.AddEdge(nodes[1], nodes[4], 2.5);
	// graph_ptr.AddEdge(nodes[1], nodes[2], 1.0);
	// graph_ptr.AddEdge(nodes[2], nodes[1], 1.5);
	// graph_ptr.AddEdge(nodes[2], nodes[5], 2.0);
	graph_ptr.AddEdge(nodes[3], nodes[0], 2.5);
	// graph_ptr.AddEdge(nodes[3], nodes[4], 2.5);
	// graph_ptr.AddEdge(nodes[4], nodes[1], 2.5);
	// graph_ptr.AddEdge(nodes[4], nodes[3], 2.5);
	// graph_ptr.AddEdge(nodes[4], nodes[5], 2.5);
	// graph_ptr.AddEdge(nodes[5], nodes[2], 2.5);
	// graph_ptr.AddEdge(nodes[5], nodes[4], 2.5);
	// graph_ptr.AddEdge(nodes[5], nodes[8], 2.5);
	// graph_ptr.AddEdge(nodes[7], nodes[4], 2.5);
	// graph_ptr.AddEdge(nodes[7], nodes[8], 2.5);
	// graph_ptr.AddEdge(nodes[8], nodes[5], 2.5);
	// graph_ptr.AddEdge(nodes[8], nodes[7], 2.5);

	// auto all_edges = graph_ptr.GetAllEdges();

	// for(auto& e : all_edges)
	// 	e->PrintEdge();

	for (auto it = graph_ptr.vertex_begin(); it != graph_ptr.vertex_end(); ++it)
	{
		for (auto eit = it->edge_begin(); eit != it->edge_end(); ++eit)
		{
			eit->PrintEdge();
		}
	}

	graph_ptr.RemoveEdge(nodes[3], nodes[0]);

	std::cout << "after change" << std::endl;

	for (auto it = graph_ptr.vertex_begin(); it != graph_ptr.vertex_end(); ++it)
	{
		for (auto eit = it->edge_begin(); eit != it->edge_end(); ++eit)
		{
			eit->PrintEdge();
		}
	}

	// need to delete all nodes, the graph only maintains pointers to these nodes
	for (auto e : nodes)
		delete e;
}

int main(int argc, char **argv)
{
	std::cout << "\n------------- value type graph --------------\n"
			  << std::endl;
	ValueTypeGraphDemo();

	std::cout << "\n------------- pointer type graph -------------\n"
			  << std::endl;
	PointerTypeGraphDemo();

	return 0;
}
