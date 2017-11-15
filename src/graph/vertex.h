/* 
 * vertex.h
 * 
 * Created on: Feb 1, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef VERTEX_H
#define VERTEX_H

#include <cstdint>
#include <algorithm>

#include "graph/edge.h"

namespace librav
{

/****************************************************************************/
/*								 Vertex_t										*/
/****************************************************************************/
/// A vertex data structure template.
template <typename StateType>
class Vertex_t
{
  public:
	template <class T = StateType, typename std::enable_if<std::is_pointer<T>::value>::type * = nullptr>
	Vertex_t(T state_node);

	template <class T = StateType, typename std::enable_if<!std::is_pointer<T>::value>::type * = nullptr>
	Vertex_t(T state_node);

	~Vertex_t() = default;

	// friends
	template <typename T>
	friend class Graph_t;
	friend class AStar;

	// generic attributes
	StateType state_;
	uint64_t vertex_id_;
	std::vector<Edge<Vertex_t<StateType> *>> edges_;

  public:
	/// == operator overloading. If two vertices have the same id, they're regarded as equal.
	bool operator==(const Vertex_t<StateType> &other) const;

	/// Get edge cost from current vertex to given vertex. -1 is returned if no edge between
	///		the two vertices exists.
	double GetEdgeCost(const Vertex_t<StateType> &dst_node) const;

	/// Get all neighbor vertices of this vertex.
	std::vector<Vertex_t<StateType> *> GetNeighbours();

	/// Check if a given vertex is the neighbor of current vertex.
	bool CheckNeighbour(Vertex_t<StateType> *dst_node);

  private:
	// vertices that contain edges connecting to current vertex
	std::vector<Vertex_t<StateType> *> associated_vertices_;

	// attributes for A* search
	bool is_checked_;
	bool is_in_openlist_;
	double f_astar_;
	double g_astar_;
	double h_astar_;
	Vertex_t<StateType> *search_parent_;

  private:
	/// Clear exiting search info before a new search
	void ClearVertexSearchInfo();

	// DEPRACATED
	/// Get heuristic using function provided by bundled data (pointer type)
	template <class T = StateType, typename std::enable_if<std::is_pointer<T>::value>::type * = nullptr>
	double CalcHeuristic(Vertex_t<StateType> *dst_vertex)
	{
		return this->state_->GetHeuristic(*(dst_vertex->state_));
	}

	// DEPRACATED
	/// Get heuristic using function provided by bundled data (non-pointer type)
	template <class T = StateType,
			  typename std::enable_if<!std::is_pointer<T>::value>::type * = nullptr>
	double CalcHeuristic(Vertex_t<StateType> *dst_vertex)
	{
		return this->state_.GetHeuristic(dst_vertex->state_);
	}
};
}

#include "graph/internal/vertex_impl.h"

#endif /* VERTEX_H */
