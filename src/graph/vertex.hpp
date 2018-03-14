/* 
 * vertex.hpp
 * 
 * Created on: Feb 1, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef VERTEX_HPP
#define VERTEX_HPP

#include <cstdint>
#include <algorithm>

#include "graph/edge.hpp"

namespace librav
{

/****************************************************************************/
/*								 Vertex_t										*/
/****************************************************************************/
/// A vertex data structure template.
template <typename StateType, typename TransitionType>
class Vertex_t
{
	using EdgeList = std::vector<Edge<Vertex_t<StateType, TransitionType> *, TransitionType>>;

  public:
	template <class T = StateType, typename std::enable_if<std::is_pointer<T>::value>::type * = nullptr>
	Vertex_t(T state_node);

	template <class T = StateType, typename std::enable_if<!std::is_pointer<T>::value>::type * = nullptr>
	Vertex_t(T state_node);

	~Vertex_t() = default;

	// friends
	template <typename T1, typename T2>
	friend class Graph_t;
	friend class AStar;
	friend class Dijkstra;

	// generic attributes
	StateType state_;
	const uint64_t vertex_id_;

	// edge iterator for easy access
	typedef typename EdgeList::iterator edge_iterator;
	typedef typename EdgeList::const_iterator const_edge_iterator;
	edge_iterator edge_begin() { return edges_to_.begin(); }
	edge_iterator edge_end() { return edges_to_.end(); }

  public:
	/// == operator overloading. If two vertices have the same id, they're regarded as equal.
	bool operator==(const Vertex_t<StateType, TransitionType> &other) const;

	/// Get edge cost from current vertex to given vertex. -1 is returned if no edge between
	///		the two vertices exists.
	TransitionType GetEdgeCost(const Vertex_t<StateType, TransitionType> &dst_node) const;

	/// Get all neighbor vertices of this vertex.
	std::vector<Vertex_t<StateType, TransitionType> *> GetNeighbours();

	/// Check if a given vertex is the neighbor of current vertex.
	bool CheckNeighbour(Vertex_t<StateType, TransitionType> *dst_node);

  private:
	// edges connecting to other vertices
	std::vector<Edge<Vertex_t<StateType, TransitionType> *, TransitionType>> edges_to_;

	// vertices that contain edges connecting to current vertex,
	//	used to cleanup edges in other vertices if current vertex is deleted
	std::vector<Vertex_t<StateType, TransitionType> *> vertices_from_;

	// attributes for A* search
	bool is_checked_ = false;
	bool is_in_openlist_ = false;
	double f_astar_ = 0.0;
	double g_astar_ = 0.0;
	double h_astar_ = 0.0;
	Vertex_t<StateType, TransitionType> *search_parent_ = nullptr;

	/// Clear exiting search info before a new search
	void ClearVertexSearchInfo();
};
}

#include "graph/details/vertex_impl.hpp"

#endif /* VERTEX_HPP */
