/* 
 * edge_impl.hpp
 * 
 * Created on: Jul 14, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef EDGE_IMPL_HPP
#define EDGE_IMPL_HPP

#include <iostream>

namespace librav
{

/****************************************************************************/
/*								 Edge  										*/
/****************************************************************************/

/**
	 * Print edge information: start vertex id, destination vertex id, edge cost.
	 */
template <typename VertexPtrType, typename TransitionType>
void Edge<VertexPtrType, TransitionType>::PrintEdge() const
{
	std::cout << "Edge: src - " << src_->vertex_id_ << " , dst - " << dst_->vertex_id_ << " , cost - " << cost_ << std::endl;
}
}

#endif /* EDGE_IMPL_HPP */
