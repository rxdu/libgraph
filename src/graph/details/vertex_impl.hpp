/* 
 * vertex_impl.hpp
 * 
 * Created on: Feb 1, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef VERTEX_IMPL_HPP
#define VERTEX_IMPL_HPP

#include <cstdint>
#include <algorithm>

#include "graph/details/edge_impl.hpp"

namespace librav
{

/****************************************************************************/
/*								 Vertex_t										*/
/****************************************************************************/
/// A vertex data structure template.
template <typename StateType, typename TransitionType>
template <class T, typename std::enable_if<std::is_pointer<T>::value>::type *>
Vertex_t<StateType, TransitionType>::Vertex_t(T state) : state_(state), vertex_id_(state->GetUniqueID()){};

template <typename StateType, typename TransitionType>
template <class T, typename std::enable_if<!std::is_pointer<T>::value>::type *>
Vertex_t<StateType, TransitionType>::Vertex_t(T state) : state_(state), vertex_id_(state.GetUniqueID()){};

/// Clear exiting search info before a new search
template <typename StateType, typename TransitionType>
void Vertex_t<StateType, TransitionType>::ClearVertexSearchInfo()
{
	is_checked_ = false;
	is_in_openlist_ = false;
	search_parent_ = nullptr;

	f_astar_ = 0.0;
	g_astar_ = 0.0;
	h_astar_ = 0.0;
}

/// == operator overloading. If two vertices have the same id, they're regarded as equal.
template <typename StateType, typename TransitionType>
bool Vertex_t<StateType, TransitionType>::operator==(const Vertex_t<StateType, TransitionType> &other) const
{
	if (vertex_id_ == other.vertex_id_)
		return true;
	else
		return false;
}

/// Get edge cost from current vertex to given vertex. -1 is returned if no edge between
///		the two vertices exists.
template <typename StateType, typename TransitionType>
TransitionType Vertex_t<StateType, TransitionType>::GetEdgeCost(const Vertex_t<StateType, TransitionType> &dst_node) const
{
	TransitionType cost;

	for (const auto &it : edges_to_)
	{
		if (it.dst_.vertex_id_ == dst_node.vertex_id_)
		{
			cost = it.cost_;
			break;
		}
	}

	return cost;
}

/// Get all neighbor vertices of this vertex.
template <typename StateType, typename TransitionType>
std::vector<Vertex_t<StateType, TransitionType> *> Vertex_t<StateType, TransitionType>::GetNeighbours()
{
	std::vector<Vertex_t<StateType, TransitionType> *> neighbours;

	for (const auto &edge : edges_to_)
		neighbours.push_back(edge.dst_);

	return neighbours;
}

/// Check if a given vertex is the neighbor of current vertex.
template <typename StateType, typename TransitionType>
bool Vertex_t<StateType, TransitionType>::CheckNeighbour(Vertex_t<StateType, TransitionType> *dst_node)
{
	std::vector<Vertex_t<StateType, TransitionType> *> neighbours = GetNeighbours();

	auto it = find(neighbours.begin(), neighbours.end(), dst_node);

	if (it != neighbours.end())
		return true;
	else
		return false;
}
}

#endif /* VERTEX_IMPL_HPP */
