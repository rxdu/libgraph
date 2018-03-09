/* 
 * astar.hpp
 * 
 * Created on: Jan 18, 2016
 * Description: A* algorithm
 * Reference:
 *  	1. http://www.redblobgames.com/pathfinding/a-star/implementation.html
 *  	2. https://oopscenities.net/2012/02/24/c11-stdfunction-and-stdbind/
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef ASTAR_HPP
#define ASTAR_HPP

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

template <typename StateType>
using CalcHeuristicFunc_t = std::function<double(StateType, StateType)>;

/// A* search algorithm.
class AStar
{

  public:
	/// Search using vertices
	template <typename StateType, typename TransitionType>
	static Path_t<StateType, TransitionType> Search(Graph_t<StateType, TransitionType> &graph, Vertex_t<StateType, TransitionType> *start, Vertex_t<StateType, TransitionType> *goal, std::function<double(StateType, StateType)> calc_heuristic)
	{
		// reset last search information
		graph.ResetGraphVertices();

		// start a new search and return result
		return Search(start, goal, calc_heuristic);
	}

	template <typename StateType, typename TransitionType>
	static Path_t<StateType, TransitionType> Search(std::shared_ptr<Graph_t<StateType, TransitionType>> graph, Vertex_t<StateType, TransitionType> *start, Vertex_t<StateType, TransitionType> *goal, std::function<double(StateType, StateType)> calc_heuristic)
	{
		// reset last search information
		graph->ResetGraphVertices();

		// start a new search and return result
		return Search(start, goal, calc_heuristic);
	}

	template <typename StateType, typename TransitionType>
	static Path_t<StateType, TransitionType> Search(Graph_t<StateType, TransitionType> *graph, Vertex_t<StateType, TransitionType> *start, Vertex_t<StateType, TransitionType> *goal, std::function<double(StateType, StateType)> calc_heuristic)
	{
		// reset last search information
		graph->ResetGraphVertices();

		// start a new search and return result
		return Search(start, goal, calc_heuristic);
	}

	/// Search using vertex ids
	template <typename StateType, typename TransitionType>
	static Path_t<StateType, TransitionType> Search(Graph_t<StateType, TransitionType> &graph, uint64_t start_id, uint64_t goal_id, std::function<double(StateType, StateType)> calc_heuristic)
	{
		// reset last search information
		graph.ResetGraphVertices();

		auto start = graph.GetVertexFromID(start_id);
		auto goal = graph.GetVertexFromID(goal_id);

		Path_t<StateType, TransitionType> empty;

		// start a new search and return result
		if (start != nullptr && goal != nullptr)
			return Search(start, goal, calc_heuristic);
		else
			return empty;
	}

	template <typename StateType, typename TransitionType>
	static Path_t<StateType, TransitionType> Search(std::shared_ptr<Graph_t<StateType, TransitionType>> graph, uint64_t start_id, uint64_t goal_id, std::function<double(StateType, StateType)> calc_heuristic)
	{
		// reset last search information
		graph->ResetGraphVertices();

		auto start = graph->GetVertexFromID(start_id);
		auto goal = graph->GetVertexFromID(goal_id);

		Path_t<StateType, TransitionType> empty;

		// start a new search and return result
		if (start != nullptr && goal != nullptr)
			return Search(start, goal, calc_heuristic);
		else
			return empty;
	}

	template <typename StateType, typename TransitionType>
	static Path_t<StateType, TransitionType> Search(Graph_t<StateType, TransitionType> *graph, uint64_t start_id, uint64_t goal_id, std::function<double(StateType, StateType)> calc_heuristic)
	{
		// reset last search information
		graph->ResetGraphVertices();

		auto start = graph->GetVertexFromID(start_id);
		auto goal = graph->GetVertexFromID(goal_id);

		Path_t<StateType, TransitionType> empty;

		// start a new search and return result
		if (start != nullptr && goal != nullptr)
			return Search(start, goal, calc_heuristic);
		else
			return empty;
	}

	template <typename StateType, typename TransitionType>
	static std::vector<StateType> IncSearch(StateType start_state, StateType goal_state, std::function<std::vector<std::tuple<StateType, TransitionType>>(StateType)> get_neighbours, std::function<double(StateType, StateType)> CalcHeuristic)
	{
		using GraphVertexType = Vertex_t<StateType, double>;

		// create a new graph with only start and goal vertices
		Graph_t<StateType> graph;
		GraphVertexType *start_vtx = graph.AddVertex(start_state);
		GraphVertexType *goal_vtx = graph.AddVertex(goal_state);

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

			std::vector<std::tuple<StateType, double>> neighbours = get_neighbours(current_vertex->state_);
			for (auto &nb : neighbours)
				graph.AddEdge(current_vertex->state_, std::get<0>(nb), std::get<1>(nb));

			// check all adjacent vertices (successors of current vertex)
			for (auto &edge : current_vertex->edges_to_)
			{
				auto successor = edge.dst_;

				// check if the vertex has been checked (in closed list)
				if (successor->is_checked_ == false)
				{
					auto new_cost = current_vertex->g_astar_ + edge.cost_;

					// if the vertex is not in open list
					// or if the vertex is in open list but has a higher cost
					if (successor->is_in_openlist_ == false || new_cost < successor->g_astar_)
					{
						// first set the parent of the adjacent vertex to be the current vertex
						successor->search_parent_ = current_vertex;

						// update costs
						successor->g_astar_ = new_cost;
						successor->h_astar_ = CalcHeuristic(successor->state_, goal_vtx->state_);
						successor->f_astar_ = successor->g_astar_ + successor->h_astar_;

						// put vertex into open list
						openlist.put(successor, successor->f_astar_);
						successor->is_in_openlist_ = true;
					}
				}
			}
		}

		// reconstruct path from search
		std::vector<StateType> path;
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
	template <typename StateType>
	static std::vector<Vertex_t<StateType, double> *> Search(Vertex_t<StateType, double> *start_vtx, Vertex_t<StateType, double> *goal_vtx, std::function<double(StateType, StateType)> CalcHeuristic)
	{
		using GraphVertexType = Vertex_t<StateType, double>;

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
					auto new_cost = current_vertex->g_astar_ + edge.cost_;

					// if the vertex is not in open list
					// or if the vertex is in open list but has a higher cost
					if (successor->is_in_openlist_ == false || new_cost < successor->g_astar_)
					{
						// first set the parent of the adjacent vertex to be the current vertex
						successor->search_parent_ = current_vertex;

						// update costs
						successor->g_astar_ = new_cost;
						successor->h_astar_ = CalcHeuristic(successor->state_, goal_vtx->state_);
						successor->f_astar_ = successor->g_astar_ + successor->h_astar_;

						// put vertex into open list
						openlist.put(successor, successor->f_astar_);
						successor->is_in_openlist_ = true;
					}
				}
			}
		}

		// reconstruct path from search
		std::vector<GraphVertexType *> path;
		if (found_path)
		{
			std::cout << "path found with cost " << goal_vtx->g_astar_ << std::endl;
			path = ReconstructPath(start_vtx, goal_vtx);
		}
		else
			std::cout << "failed to find a path" << std::endl;

		return path;
	};

	template <typename StateType>
	static std::vector<Vertex_t<StateType, double> *> ReconstructPath(Vertex_t<StateType, double> *start_vtx, Vertex_t<StateType, double> *goal_vtx)
	{
		std::vector<Vertex_t<StateType, double> *> path;
		Vertex_t<StateType, double> *waypoint = goal_vtx;
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

#endif /* ASTAR_HPP */
