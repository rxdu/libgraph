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
bool Vertex_t<StateType, TransitionType>::operator==(const VertexType &other) const
{
	if (vertex_id_ == other.vertex_id_)
		return true;
	else
		return false;
}

template <typename StateType, typename TransitionType>
TransitionType Vertex_t<StateType, TransitionType>::GetEdgeCost(int64_t dst_id) const
{
	TransitionType cost = {-1};

	for (const auto &it : edges_to_)
		if (it.dst_->vertex_id_ == dst_id)
			return it.cost_;

	return cost;
}

/// Get edge cost from current vertex to given vertex. -1 is returned if no edge between
///		the two vertices exists.
template <typename StateType, typename TransitionType>
TransitionType Vertex_t<StateType, TransitionType>::GetEdgeCost(const VertexType *dst_node) const
{
	return GetEdgeCost(dst_node->vertex_id_);
}

/// Get all neighbor vertices of this vertex.
template <typename StateType, typename TransitionType>
std::vector<Vertex_t<StateType, TransitionType> *> Vertex_t<StateType, TransitionType>::GetNeighbours()
{
	std::vector<VertexType *> neighbours;

	for (const auto &edge : edges_to_)
		neighbours.push_back(edge.dst_);

	return neighbours;
}

template <typename StateType, typename TransitionType>
std::vector<int64_t> Vertex_t<StateType, TransitionType>::GetNeighbourIDs()
{
	std::vector<int64_t> neighbours;

	for (const auto &edge : edges_to_)
		neighbours.push_back(edge.dst_->vertex_id_);

	return neighbours;
}

/// Check if a given vertex is the neighbor of current vertex.
template <typename StateType, typename TransitionType>
bool Vertex_t<StateType, TransitionType>::CheckNeighbour(int64_t dst_id)
{
	std::vector<int64_t> neighbours = GetNeighbourIDs();

	auto it = find(neighbours.begin(), neighbours.end(), dst_id);

	return (it != neighbours.end() ? true : false);
}

/// Check if a given vertex is the neighbor of current vertex.
template <typename StateType, typename TransitionType>
bool Vertex_t<StateType, TransitionType>::CheckNeighbour(VertexType *dst_node)
{
	auto neighbours = GetNeighbours();

	auto it = find(neighbours.begin(), neighbours.end(), dst_node);

	return (it != neighbours.end() ? true : false);
}
}

#endif /* VERTEX_IMPL_HPP */
