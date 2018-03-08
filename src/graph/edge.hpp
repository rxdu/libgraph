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

/****************************************************************************/
/*								 Edge  										*/
/****************************************************************************/
/// An edge data structure template.
template <typename VertexPtrType, typename TransitionType>
class Edge
{
  public:
	/**
	 * @param src a pointer to the source vertex of the edge
	 * @param dst a pointer to the destination vertex of the edge
	 * @param c cost associated with the edge
	 */
	Edge(VertexPtrType src, VertexPtrType dst, TransitionType c) : src_(src), dst_(dst), cost_(c){};
	~Edge() = default;

	VertexPtrType src_;
	VertexPtrType dst_;
	TransitionType cost_;

	/**
	 * == operator overloading. If two edges connect the same pair of vertices, they're
	 * regarded as equal.
	 */
	bool operator==(const Edge<VertexPtrType, TransitionType> &other)
	{
		if (src_->vertex_id_ == other.src_->vertex_id_ && dst_->vertex_id_ == other.dst_->vertex_id_)
			return true;
		else
			return false;
	};

	/**
	 * This operation checks if two edges connect the same vertex pair.
	 * If two edges connect the same pair of vertices, return true, otherwise false.
	 */
	bool operator-=(const Edge<VertexPtrType, TransitionType> &other)
	{
		if ((src_->vertex_id_ == other.src_->vertex_id_ && dst_->vertex_id_ == other.dst_->vertex_id_) || (src_->vertex_id_ == other.dst_->vertex_id_ && dst_->vertex_id_ == other.src_->vertex_id_))
			return true;
		else
			return false;
	};

	/**
	 * Print edge information: start vertex id, destination vertex id, edge cost.
	 */
	void PrintEdge() const;
};
}

#include "graph/details/edge_impl.hpp"

#endif /* EDGE_HPP */
