/*
 * astar.h
 *
 *  Created on: Jan 18, 2016
 *      Author: rdu
 *  Code Reference:
 *  	1. http://www.redblobgames.com/pathfinding/a-star/implementation.html
 *  	2. https://oopscenities.net/2012/02/24/c11-stdfunction-and-stdbind/
 */

#ifndef SRC_GRAPH_ASTAR_H_
#define SRC_GRAPH_ASTAR_H_

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

#include "graph/graph.h"

#define MINIMAL_PRINTOUT 1

namespace librav {

template<typename StateType>
using CalcHeuristicFunc_t = std::function<double(StateType,StateType)>;

/// A simple priority queue structure used as A* open list.
// Source: http://www.redblobgames.com/pathfinding/a-star/implementation.html
template<typename T, typename Number=double>
struct PriorityQueue {
	typedef std::pair<Number, T> PQElement;

	std::priority_queue<PQElement, std::vector<PQElement>,
	std::greater<PQElement>> elements;

	inline bool empty() const { return elements.empty(); }

	inline void put(T item, Number priority) {
		elements.emplace(priority, item);
	}

	inline T get() {
		T best_item = elements.top().second;
		elements.pop();
		return best_item;
	}
};

/// A* search algorithm.
class AStar{

public:

	/// Search using vertices
	template<typename StateType, typename TransitionType>
	static Path_t<StateType, TransitionType> Search(Graph_t<StateType, TransitionType>& graph, Vertex_t<StateType, TransitionType> *start, Vertex_t<StateType, TransitionType> *goal, std::function<double(StateType, StateType)> calc_heuristic)
	{
		// reset last search information
		graph.ResetGraphVertices();

		// start a new search and return result
		return Search(start, goal, calc_heuristic);
	}

	template<typename StateType, typename TransitionType>
	static Path_t<StateType, TransitionType> Search(std::shared_ptr<Graph_t<StateType, TransitionType>> graph, Vertex_t<StateType, TransitionType> *start, Vertex_t<StateType, TransitionType> *goal, std::function<double(StateType, StateType)> calc_heuristic)
	{
		// reset last search information
		graph->ResetGraphVertices();

		// start a new search and return result
		return Search(start, goal, calc_heuristic);
	}

	template<typename StateType, typename TransitionType>
	static Path_t<StateType, TransitionType> Search(Graph_t<StateType, TransitionType>* graph, Vertex_t<StateType, TransitionType> *start, Vertex_t<StateType, TransitionType> *goal, std::function<double(StateType, StateType)> calc_heuristic)
	{
		// reset last search information
		graph->ResetGraphVertices();

		// start a new search and return result
		return Search(start, goal, calc_heuristic);
	}

	/// Search using vertex ids
	template<typename StateType, typename TransitionType>
	static Path_t<StateType, TransitionType> Search(Graph_t<StateType, TransitionType>& graph, uint64_t start_id, uint64_t goal_id, std::function<double(StateType, StateType)> calc_heuristic)
	{
		// reset last search information
		graph.ResetGraphVertices();

		auto start = graph.GetVertexFromID(start_id);
		auto goal = graph.GetVertexFromID(goal_id);

		Path_t<StateType, TransitionType> empty;

		// start a new search and return result
		if(start != nullptr && goal != nullptr)
			return Search(start, goal, calc_heuristic);
		else
			return empty;
	}

	template<typename StateType, typename TransitionType>
	static Path_t<StateType, TransitionType> Search(std::shared_ptr<Graph_t<StateType, TransitionType>> graph, uint64_t start_id, uint64_t goal_id, std::function<double(StateType, StateType)> calc_heuristic)
	{
		// reset last search information
		graph->ResetGraphVertices();

		auto start = graph->GetVertexFromID(start_id);
		auto goal = graph->GetVertexFromID(goal_id);

		Path_t<StateType, TransitionType> empty;

		// start a new search and return result
		if(start != nullptr && goal != nullptr)
			return Search(start, goal, calc_heuristic);
		else
			return empty;
	}

	template<typename StateType, typename TransitionType>
	static Path_t<StateType, TransitionType> Search(Graph_t<StateType, TransitionType>* graph, uint64_t start_id, uint64_t goal_id, std::function<double(StateType, StateType)> calc_heuristic)
	{
		// reset last search information
		graph->ResetGraphVertices();

		auto start = graph->GetVertexFromID(start_id);
		auto goal = graph->GetVertexFromID(goal_id);

		Path_t<StateType, TransitionType> empty;

		// start a new search and return result
		if(start != nullptr && goal != nullptr)
			return Search(start, goal, calc_heuristic);
		else
			return empty;
	}

private:
	template<typename StateType>
	static std::vector<Vertex_t<StateType, double>*> Search(Vertex_t<StateType, double> *start_vtx, Vertex_t<StateType, double> *goal_vtx, std::function<double(StateType,StateType)> CalcHeuristic)
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

						successor->h_astar_ = CalcHeuristic(successor->state_, goal_vtx->state_); 
						successor->f_astar_ = successor->g_astar_ + successor->h_astar_;

						openlist.put(successor, successor->f_astar_);
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

#endif /* SRC_GRAPH_ASTAR_H_ */
