/*
 * astar.h
 *
 *  Created on: Jan 18, 2016
 *      Author: rdu
 *  Code Reference:
 *  	1. http://www.redblobgames.com/pathfinding/a-star/implementation.html
 */

#ifndef SRC_GRAPH_ASTAR_H_
#define SRC_GRAPH_ASTAR_H_

#include <vector>
#include <queue>
#include <functional>
#include <utility>
#include <cmath>
#include <algorithm>
#include <type_traits>

#include "graph/vertex.h"

namespace srcl_ctrl {

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
namespace AStar{

template<typename GraphVertexType>
std::vector<GraphVertexType*> Search(GraphVertexType *start, GraphVertexType *goal){
	bool found_path = false;
	std::vector<GraphVertexType*> trajectory;
	GraphVertexType* current_vertex;
	// open list - a list of vertices that need to be checked out
	PriorityQueue<GraphVertexType*> openlist;

	openlist.put(start, 0);
	start->is_in_openlist_ = true;

	//start->search_parent_ = start;
	start->g_astar_ = 0;

	while(!openlist.empty() && found_path != true)
	{
		current_vertex = openlist.get();
		if(current_vertex->is_checked_)
			continue;

		current_vertex->is_in_openlist_ = false;
		current_vertex->is_checked_ = true;

		// check all adjacent vertices (successors of current vertex)
		for(auto ite = current_vertex->edges_.begin(); ite != current_vertex->edges_.end(); ite++)
		{
			GraphVertexType* successor;
			successor = (*ite).dst_;

			// check if the vertex has been checked (in closed list)
			if(successor->is_checked_ == false)
			{
				// first set the parent of the adjacent vertex to be the current vertex
				double new_cost = current_vertex->g_astar_ + (*ite).cost_;

				// if the vertex is not in open list
				// or if the vertex is in open list but has a higher cost
				if(successor->is_in_openlist_ == false || new_cost < successor->g_astar_)
				{
					successor->search_parent_ = current_vertex;
					successor->g_astar_ = new_cost;

					successor->h_astar_ = successor->CalcHeuristic(goal);
					successor->f_astar_ = successor->g_astar_ + successor->h_astar_;

					openlist.put(successor, successor->f_astar_);
					successor->is_in_openlist_ = true;

					if(successor == goal){
						found_path = true;
					}
				}
			}
		}
	}

	// reconstruct path from search
	if(found_path)
	{
		std::cout << "path found" << std::endl;
		GraphVertexType* waypoint = goal;
		while(waypoint != start)
		{
			trajectory.push_back(waypoint);
			waypoint = waypoint->search_parent_;
		}
		// add the start node
		trajectory.push_back(waypoint);
		std::reverse(trajectory.begin(), trajectory.end());

		auto traj_s = trajectory.begin();
		auto traj_e = trajectory.end() - 1;
		std::cout << "starting vertex id: " << (*traj_s)->vertex_id_ << std::endl;
		std::cout << "finishing vertex id: " << (*traj_e)->vertex_id_ << std::endl;
		std::cout << "path length: " << trajectory.size() << std::endl;
		std::cout << "total cost: " << trajectory.back()->g_astar_ << std::endl;
	}
	else
		std::cout << "failed to find a path" << std::endl;

	return trajectory;
};

};

}

#endif /* SRC_GRAPH_ASTAR_H_ */
