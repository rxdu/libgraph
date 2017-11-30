/* 
 * dijkstra.hpp
 * 
 * Created on: Nov 30, 2017 14:22
 * Description: Dijkstra's algorithm 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include <vector>
#include <tuple>
#include <queue>
#include <functional>
#include <utility>
#include <cmath>
#include <algorithm>
#include <type_traits>
#include <functional>
#include <iostream>
#include <memory>

#include "graph/graph.hpp"
#include "graph/details/priority_queue.hpp"

#define MINIMAL_PRINTOUT 1

namespace librav {

/// Dijkstra search algorithm.
class Dijkstra{

public:

	/// Search using vertices
	template<typename StateType, typename TransitionType>
	static Path_t<StateType, TransitionType> Search(Graph_t<StateType, TransitionType>& graph, Vertex_t<StateType, TransitionType> *start, Vertex_t<StateType, TransitionType> *goal)
	{
		// reset last search information
		graph.ResetGraphVertices();

		// start a new search and return result
		return Search(start, goal);
	}

	template<typename StateType, typename TransitionType>
	static Path_t<StateType, TransitionType> Search(std::shared_ptr<Graph_t<StateType, TransitionType>> graph, Vertex_t<StateType, TransitionType> *start, Vertex_t<StateType, TransitionType> *goal)
	{
		// reset last search information
		graph->ResetGraphVertices();

		// start a new search and return result
		return Search(start, goal);
	}

	template<typename StateType, typename TransitionType>
	static Path_t<StateType, TransitionType> Search(Graph_t<StateType, TransitionType>* graph, Vertex_t<StateType, TransitionType> *start, Vertex_t<StateType, TransitionType> *goal)
	{
		// reset last search information
		graph->ResetGraphVertices();

		// start a new search and return result
		return Search(start, goal);
	}

	/// Search using vertex ids
	template<typename StateType, typename TransitionType>
	static Path_t<StateType, TransitionType> Search(Graph_t<StateType, TransitionType>& graph, uint64_t start_id, uint64_t goal_id)
	{
		// reset last search information
		graph.ResetGraphVertices();

		auto start = graph.GetVertexFromID(start_id);
		auto goal = graph.GetVertexFromID(goal_id);

		Path_t<StateType, TransitionType> empty;

		// start a new search and return result
		if(start != nullptr && goal != nullptr)
			return Search(start, goal);
		else
			return empty;
	}

	template<typename StateType, typename TransitionType>
	static Path_t<StateType, TransitionType> Search(std::shared_ptr<Graph_t<StateType, TransitionType>> graph, uint64_t start_id, uint64_t goal_id)
	{
		// reset last search information
		graph->ResetGraphVertices();

		auto start = graph->GetVertexFromID(start_id);
		auto goal = graph->GetVertexFromID(goal_id);

		Path_t<StateType, TransitionType> empty;

		// start a new search and return result
		if(start != nullptr && goal != nullptr)
			return Search(start, goal);
		else
			return empty;
	}

	template<typename StateType, typename TransitionType>
	static Path_t<StateType, TransitionType> Search(Graph_t<StateType, TransitionType>* graph, uint64_t start_id, uint64_t goal_id)
	{
		// reset last search information
		graph->ResetGraphVertices();

		auto start = graph->GetVertexFromID(start_id);
		auto goal = graph->GetVertexFromID(goal_id);

		Path_t<StateType, TransitionType> empty;

		// start a new search and return result
		if(start != nullptr && goal != nullptr)
			return Search(start, goal);
		else
			return empty;
	}

private:
	template<typename StateType>
	static std::vector<Vertex_t<StateType, double>*> Search(Vertex_t<StateType, double> *start_vtx, Vertex_t<StateType, double> *goal_vtx)
	{
		using GraphVertexType = Vertex_t<StateType, double>;

		bool found_path = false;
		std::vector<GraphVertexType*> path;
		GraphVertexType* current_vertex;
		// open list - a list of vertices that need to be checked out
		PriorityQueue<GraphVertexType*> openlist;

		openlist.put(start_vtx, 0);
		start_vtx->is_in_openlist_ = true;

		//start->search_parent_ = start;
		start_vtx->g_astar_ = 0;

		while(!openlist.empty() && found_path != true)
		{
			current_vertex = openlist.get();
			if(current_vertex->is_checked_)
				continue;
			if(current_vertex == goal_vtx) {
				found_path = true;
				break;
			}

			current_vertex->is_in_openlist_ = false;
			current_vertex->is_checked_ = true;

			// check all adjacent vertices (successors of current vertex)
			for(auto& edge : current_vertex->edges_to_)
			{
				GraphVertexType* successor;
				successor = edge.dst_;

				// check if the vertex has been checked (in closed list)
				if(successor->is_checked_ == false)
				{
					// first set the parent of the adjacent vertex to be the current vertex
					double new_cost = current_vertex->g_astar_ + edge.cost_;

					// if the vertex is not in open list
					// or if the vertex is in open list but has a higher cost
					if(successor->is_in_openlist_ == false || new_cost < successor->g_astar_)
					{
						successor->search_parent_ = current_vertex;
						successor->g_astar_ = new_cost;

						openlist.put(successor, successor->g_astar_);
						successor->is_in_openlist_ = true;
					}
				}
			}
		}

		// reconstruct path from search
		if(found_path)
		{
			std::cout << "path found" << std::endl;
			GraphVertexType* waypoint = goal_vtx;
			while(waypoint != start_vtx)
			{
				path.push_back(waypoint);
				waypoint = waypoint->search_parent_;
			}
			// add the start node
			path.push_back(waypoint);
			std::reverse(path.begin(), path.end());

			auto traj_s = path.begin();
			auto traj_e = path.end() - 1;
#ifdef MINIMAL_PRINTOUT
			std::cout << "starting vertex id: " << (*traj_s)->vertex_id_ << std::endl;
			std::cout << "finishing vertex id: " << (*traj_e)->vertex_id_ << std::endl;
			std::cout << "path length: " << path.size() << std::endl;
			std::cout << "total cost: " << path.back()->g_astar_ << std::endl;
#endif
		}
		else
			std::cout << "failed to find a path" << std::endl;

		return path;
	};

};

}

#endif /* DIJKSTRA_HPP */
