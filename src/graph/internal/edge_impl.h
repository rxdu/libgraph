/* 
 * edge_impl.h
 * 
 * Created on: Jul 14, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef EDGE_IMPL_H
#define EDGE_IMPL_H

#include <iostream>

namespace librav
{

/****************************************************************************/
/*								 Edge  										*/
/****************************************************************************/

/**
	 * == operator overloading. If two edges connect the same pair of vertices, they're
	 * regarded as equal.
	 */
template <typename VertexPtrType>
bool Edge<VertexPtrType>::operator==(const Edge<VertexPtrType> &other)
{
	if (src_->vertex_id_ == other.src_->vertex_id_ && dst_->vertex_id_ == other.dst_->vertex_id_)
		return true;
	else
		return false;
}

/**
	 * This operation checks if two edges connect the same vertex pair.
	 * If two edges connect the same pair of vertices, return true, otherwise false.
	 */
template <typename VertexPtrType>
bool Edge<VertexPtrType>::operator-=(const Edge<VertexPtrType> &other)
{
	if ((src_->vertex_id_ == other.src_->vertex_id_ && dst_.vertex_id_ == other.dst_->vertex_id_) || (src_->vertex_id_ == other.dst_->vertex_id_ && dst_.vertex_id_ == other.src_->vertex_id_))
		return true;
	else
		return false;
}

/**
	 * Print edge information: start vertex id, destination vertex id, edge cost.
	 */
template <typename VertexPtrType>
void Edge<VertexPtrType>::PrintEdge() const
{
	std::cout << "Edge: src - " << src_->vertex_id_ << " , dst - " << dst_->vertex_id_ << " , cost - " << cost_ << std::endl;
}
}

#endif /* EDGE_IMPL_H */
