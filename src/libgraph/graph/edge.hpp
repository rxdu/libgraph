/* 
 * edge.hpp
 * 
 * Created on: Jul 14, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef EDGE_HPP
#define EDGE_HPP

#include <iostream>

namespace librav
{

// Forward declaration
template <typename StateType, typename TransitionType>
class Graph_t;

template <typename StateType, typename TransitionType>
class Vertex_t;

/****************************************************************************/
/*								 Edge_t  										*/
/****************************************************************************/
/// An edge data structure template.
template <typename StateType, typename TransitionType>
class Edge_t
{
	using VertexPtrType = Vertex_t<StateType, TransitionType> *;

  public:
	/**
	 * @param src a pointer to the source vertex of the edge
	 * @param dst a pointer to the destination vertex of the edge
	 * @param c cost associated with the edge
	 */
	Edge_t(VertexPtrType src, VertexPtrType dst, TransitionType c) : src_(src), dst_(dst), cost_(c){};
	~Edge_t() = default;

	VertexPtrType src_;
	VertexPtrType dst_;
	TransitionType cost_;

	/**
	 * == operator overloading. If two edges connect the same pair of vertices, they're
	 * regarded as equal.
	 */
	bool operator==(const Edge_t<StateType, TransitionType> &other);

	/**
	 * This operation checks if two edges connect the same vertex pair.
	 * If two edges connect the same pair of vertices, return true, otherwise false.
	 */
	bool operator-=(const Edge_t<StateType, TransitionType> &other);

	/**
	 * Print edge information: start vertex id, destination vertex id, edge cost.
	 */
	void PrintEdge() const;
};
}

#include "graph/details/edge_impl.hpp"

#endif /* EDGE_HPP */
