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

namespace librav
{

template <typename StateType, typename TransitionType = double>
using GetNeighbourFunc_t = std::function<std::vector<std::tuple<StateType, TransitionType>>(StateType)>;

/// Dijkstra search algorithm.
class Dijkstra
{
  public:
	/// Search using vertex ids
	template <typename StateType, typename TransitionType>
	static Path_t<StateType> Search(std::shared_ptr<Graph_t<StateType, TransitionType>> graph, uint64_t start_id, uint64_t goal_id)
	{
		// reset last search information
		graph->ResetGraphVertices();

		auto start = graph->GetVertexFromID(start_id);
		auto goal = graph->GetVertexFromID(goal_id);

		Path_t<StateType> empty;

		// start a new search and return result
		if (start != nullptr && goal != nullptr)
			return Search(start, goal);
		else
			return empty;
	}

	template <typename StateType, typename TransitionType>
	static Path_t<StateType> Search(Graph_t<StateType, TransitionType> *graph, uint64_t start_id, uint64_t goal_id)
	{
		// reset last search information
		graph->ResetGraphVertices();

		auto start = graph->GetVertexFromID(start_id);
		auto goal = graph->GetVertexFromID(goal_id);

		Path_t<StateType> empty;

		// start a new search and return result
		if (start != nullptr && goal != nullptr)
			return Search(start, goal);
		else
			return empty;
	}

	template <typename StateType, typename TransitionType>
	static Path_t<StateType> IncSearch(StateType start_state, StateType goal_state, GetNeighbourFunc_t<StateType, TransitionType> get_neighbours)
	{
		using GraphVertexType = Vertex_t<StateType, TransitionType>;

		// create a new graph with only start and goal vertices
		Graph_t<StateType, TransitionType> graph;
		graph.AddVertex(start_state);
		graph.AddVertex(goal_state);

		GraphVertexType *start_vtx = graph.GetVertex(start_state);
		GraphVertexType *goal_vtx = graph.GetVertex(goal_state);

		// open list - a list of vertices that need to be checked out
		PriorityQueue<GraphVertexType *> openlist;

		// begin with start vertex
		openlist.put(start_vtx, 0);
		start_vtx->is_in_openlist_ = true;
		start_vtx->g_astar_ = 0;

		// start search iterations
		bool found_path = false;
		GraphVertexType *current_vertex;
		while (!openlist.empty() && found_path != true)
		{
			current_vertex = openlist.get();
			if (current_vertex->is_checked_)
				continue;
			if (current_vertex == goal_vtx)
			{
				found_path = true;
				break;
			}

			current_vertex->is_in_openlist_ = false;
			current_vertex->is_checked_ = true;

			std::vector<std::tuple<StateType, TransitionType>> neighbours = get_neighbours(current_vertex->state_);
			for (auto &nb : neighbours)
				graph.AddEdge(current_vertex->state_, std::get<0>(nb), std::get<1>(nb));

			// check all adjacent vertices (successors of current vertex)
			for (auto &edge : current_vertex->edges_to_)
			{
				auto successor = edge.dst_;

				// check if the vertex has been checked (in closed list)
				if (successor->is_checked_ == false)
				{
					// first set the parent of the adjacent vertex to be the current vertex
					auto new_cost = current_vertex->g_astar_ + edge.cost_;

					// if the vertex is not in open list
					// or if the vertex is in open list but has a higher cost
					if (successor->is_in_openlist_ == false || new_cost < successor->g_astar_)
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
		Path_t<StateType> path;
		if (found_path)
		{
			std::cout << "path found with cost " << goal_vtx->g_astar_ << std::endl;
			auto path_vtx = ReconstructPath(start_vtx, goal_vtx);
			for (const auto &wp : path_vtx)
				path.push_back(wp->state_);
		}
		else
			std::cout << "failed to find a path" << std::endl;

		return path;
	};

  private:
	template <typename StateType, typename TransitionType>
	static Path_t<StateType> Search(Vertex_t<StateType, TransitionType> *start_vtx, Vertex_t<StateType, TransitionType> *goal_vtx)
	{
		using GraphVertexType = Vertex_t<StateType, TransitionType>;

		// open list - a list of vertices that need to be checked out
		PriorityQueue<GraphVertexType *> openlist;

		// begin with start vertex
		openlist.put(start_vtx, 0);
		start_vtx->is_in_openlist_ = true;
		start_vtx->g_astar_ = 0;

		// start search iterations
		bool found_path = false;
		GraphVertexType *current_vertex;
		while (!openlist.empty() && found_path != true)
		{
			current_vertex = openlist.get();
			if (current_vertex->is_checked_)
				continue;
			if (current_vertex == goal_vtx)
			{
				found_path = true;
				break;
			}

			current_vertex->is_in_openlist_ = false;
			current_vertex->is_checked_ = true;

			// check all adjacent vertices (successors of current vertex)
			for (auto &edge : current_vertex->edges_to_)
			{
				auto successor = edge.dst_;

				// check if the vertex has been checked (in closed list)
				if (successor->is_checked_ == false)
				{
					// first set the parent of the adjacent vertex to be the current vertex
					auto new_cost = current_vertex->g_astar_ + edge.cost_;

					// if the vertex is not in open list
					// or if the vertex is in open list but has a higher cost
					if (successor->is_in_openlist_ == false || new_cost < successor->g_astar_)
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
		Path_t<StateType> path;
		if (found_path)
		{
			std::cout << "path found with cost " << goal_vtx->g_astar_ << std::endl;
			auto path_vtx = ReconstructPath(start_vtx, goal_vtx);
			for (const auto &wp : path_vtx)
				path.push_back(wp->state_);
		}
		else
			std::cout << "failed to find a path" << std::endl;

		return path;
	};

	template <typename StateType, typename TransitionType>
	static std::vector<Vertex_t<StateType, TransitionType> *> ReconstructPath(Vertex_t<StateType, TransitionType> *start_vtx, Vertex_t<StateType, TransitionType> *goal_vtx)
	{
		std::vector<Vertex_t<StateType, TransitionType> *> path;
		Vertex_t<StateType, TransitionType> *waypoint = goal_vtx;
		while (waypoint != start_vtx)
		{
			path.push_back(waypoint);
			waypoint = waypoint->search_parent_;
		}
		// add the start node
		path.push_back(waypoint);
		std::reverse(path.begin(), path.end());

#ifndef MINIMAL_PRINTOUT
		auto traj_s = path.begin();
		auto traj_e = path.end() - 1;
		std::cout << "starting vertex id: " << (*traj_s)->vertex_id_ << std::endl;
		std::cout << "finishing vertex id: " << (*traj_e)->vertex_id_ << std::endl;
		std::cout << "path length: " << path.size() << std::endl;
		std::cout << "total cost: " << path.back()->g_astar_ << std::endl;
#endif
		return path;
	}
};
}

#endif /* DIJKSTRA_HPP */
