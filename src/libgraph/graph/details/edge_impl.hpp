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
/*								 Edge_t  										*/
/****************************************************************************/

template <typename StateType, typename TransitionType>
bool Edge_t<StateType, TransitionType>::operator==(const Edge_t<StateType, TransitionType> &other)
{
	if (src_->vertex_id_ == other.src_->vertex_id_ && dst_->vertex_id_ == other.dst_->vertex_id_)
		return true;
	else
		return false;
}

template <typename StateType, typename TransitionType>
bool Edge_t<StateType, TransitionType>::operator-=(const Edge_t<StateType, TransitionType> &other)
{
	if ((src_->vertex_id_ == other.src_->vertex_id_ && dst_->vertex_id_ == other.dst_->vertex_id_) || (src_->vertex_id_ == other.dst_->vertex_id_ && dst_->vertex_id_ == other.src_->vertex_id_))
		return true;
	else
		return false;
}

template <typename StateType, typename TransitionType>
void Edge_t<StateType, TransitionType>::PrintEdge() const
{
	std::cout << "Edge_t: src - " << src_->vertex_id_ << " , dst - " << dst_->vertex_id_ << " , cost - " << cost_ << std::endl;
}
}

#endif /* EDGE_IMPL_HPP */
