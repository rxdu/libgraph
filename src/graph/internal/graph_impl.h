/* 
 * graph_impl.h
 * 
 * Created on: Nov 13, 2017 23:08
 *  Description:
 *  	1. A visualized illustration of the graph structure
 *
 *  	Graph "G":
 *  		Vertex "V1" - Edge "V1_E1", which connects "V1" to "Vx1"
 *  				    - Edge "V1_E2", which connects "V1" to "Vx2"
 *  				  			...
 *  		Vertex "V2" - Edge "V2_E1", which connects "V2" to "Vx3"
 *  				    - Edge "V2_E2", which connects "V2" to "Vx4"
 *  				  			...
 *  			...
 *
 *  		Vertex "Vx"			...
 *		2. Refer to documentation for instructions of how to use this library
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef GRAPH_IMPL_H
#define GRAPH_IMPL_H

#include <map>
#include <vector>
#include <cstdint>
#include <type_traits>

namespace librav
{

/****************************************************************************/
/*								 Graph_t										*/
/****************************************************************************/
template <typename StateType>
Graph_t<StateType>::~Graph_t()
{
	for (auto it = vertex_map_.begin(); it != vertex_map_.end(); it++)
		delete it->second;
};

/// This function is used to reset the vertices for a new search
template <typename StateType>
void Graph_t<StateType>::ResetGraphVertices()
{
	for (const auto &vertex_pair : vertex_map_)
		vertex_pair.second->ClearVertexSearchInfo();
};

/// This function removes all edges and vertices in the graph
template <typename StateType>
void Graph_t<StateType>::ClearGraph()
{
	for (auto it = vertex_map_.begin(); it != vertex_map_.end(); it++)
		delete it->second;
	vertex_map_.clear();
}

/// This function is used to create a graph by adding edges connecting two nodes
template <typename StateType>
void Graph_t<StateType>::AddEdge(StateType src_node, StateType dst_node, double cost)
{
	Vertex<StateType> *src_vertex = GetVertex(src_node);
	Vertex<StateType> *dst_vertex = GetVertex(dst_node);

	if (src_vertex->CheckNeighbour(dst_vertex))
		return;

	// store information for deleting vertex
	dst_vertex->associated_vertices_.push_back(src_vertex);

	Edge<Vertex<StateType> *> new_edge(src_vertex, dst_vertex, cost);
	src_vertex->edges_.push_back(new_edge);
};

/// This function is used to remove the edge from src_node to dst_node.
template <typename StateType>
bool Graph_t<StateType>::RemoveEdge(StateType src_node, StateType dst_node)
{
	Vertex<StateType> *src_vertex = SearchVertex(src_node);
	Vertex<StateType> *dst_vertex = SearchVertex(dst_node);

	if ((src_vertex != nullptr) && (dst_vertex != nullptr))
	{
		bool found_edge = false;
		auto idx = src_vertex->edges_.end();

		for (auto it = src_vertex->edges_.begin(); it != src_vertex->edges_.end(); it++)
		{
			if ((*it).dst_ == dst_vertex)
			{
				idx = it;
				found_edge = true;
			}
		}

		if (found_edge)
			src_vertex->edges_.erase(idx);

		return found_edge;
	}
	else
		return false;
};

template <typename StateType>
template <class T, typename std::enable_if<!std::is_pointer<T>::value>::type *>
void Graph_t<StateType>::RemoveVertex(T vertex_node)
{
	auto it = vertex_map_.find((uint64_t)(vertex_node.data_id_));

	// unknown vertex, no need to remove
	if (it == vertex_map_.end())
		return;

	for (auto &asv : it->second->associated_vertices_)
		for (auto eit = asv->edges_.begin(); eit != asv->edges_.end(); eit++)
		{
			if ((*eit).dst_ == it->second)
			{
				asv->edges_.erase(eit);
				break;
			}
		}

	auto vptr = it->second;
	vertex_map_.erase(it);
	delete vptr;
};

template <typename StateType>
template <class T, typename std::enable_if<std::is_pointer<T>::value>::type *>
void Graph_t<StateType>::RemoveVertex(T vertex_node)
{
	auto it = vertex_map_.find((uint64_t)(vertex_node->data_id_));

	// unknown vertex, no need to remove
	if (it == vertex_map_.end())
		return;

	for (auto &asv : it->second->associated_vertices_)
		for (auto eit = asv->edges_.begin(); eit != asv->edges_.end(); eit++)
		{
			if ((*eit).dst_ == it->second)
			{
				asv->edges_.erase(eit);
				break;
			}
		}

	auto vptr = it->second;
	vertex_map_.erase(it);
	delete vptr;
}

/// This functions is used to access all vertices of a graph
template <typename StateType>
std::vector<Vertex<StateType> *> Graph_t<StateType>::GetGraphVertices() const
{
	std::vector<Vertex<StateType> *> vertices;

	for (auto it = vertex_map_.begin(); it != vertex_map_.end(); it++)
	{
		vertices.push_back(it->second);
	}

	return vertices;
};

/// This functions is used to access all edges of a graph
template <typename StateType>
std::vector<Edge<Vertex<StateType> *>> Graph_t<StateType>::GetGraphEdges() const
{
	std::vector<Edge<Vertex<StateType> *>> edges;

	for (auto it = vertex_map_.begin(); it != vertex_map_.end(); it++)
	{
		Vertex<StateType> *vertex = it->second;
		for (auto ite = vertex->edges_.begin(); ite != vertex->edges_.end(); ite++)
		{
			edges.push_back(*ite);
		}
	}

	return edges;
};

/// This functions is used to access all edges of a graph
template <typename StateType>
std::vector<Edge<Vertex<StateType> *>> Graph_t<StateType>::GetGraphUndirectedEdges() const
{
	std::vector<Edge<Vertex<StateType> *>> edges;

	for (auto it = vertex_map_.begin(); it != vertex_map_.end(); it++)
	{
		Vertex<StateType> *vertex = it->second;

		for (auto ite = vertex->edges_.begin(); ite != vertex->edges_.end(); ite++)
		{
			bool edge_existed = false;

			for (auto &itedge : edges)
			{
				if (itedge -= (*ite))
				{
					edge_existed = true;
					break;
				}
			}

			if (!edge_existed)
				edges.push_back(*ite);
		}
	}

	return edges;
};

/// This function return the vertex with specified id
template <typename StateType>
Vertex<StateType> *Graph_t<StateType>::GetVertexFromID(uint64_t vertex_id)
{
	auto it = vertex_map_.find(vertex_id);

	if (it != vertex_map_.end())
		return (*it).second;
	else
		return nullptr;
};

template <typename StateType>
template <class T, typename std::enable_if<!std::is_pointer<T>::value>::type *>
Vertex<StateType> *Graph_t<StateType>::GetVertex(T vertex_node)
{
	auto it = vertex_map_.find((uint64_t)(vertex_node.data_id_));

	if (it == vertex_map_.end())
	{
		Vertex<StateType> *new_vertex = new Vertex<StateType>(vertex_node);
		//vertex_map_[vertex_node.data_id_] = new_vertex;
		vertex_map_.insert(std::make_pair(vertex_node.data_id_, new_vertex));
		return new_vertex;
	}

	return it->second;
}

template <typename StateType>
template <class T, typename std::enable_if<std::is_pointer<T>::value>::type *>
Vertex<StateType> *Graph_t<StateType>::GetVertex(T vertex_node)
{
	auto it = vertex_map_.find((uint64_t)(vertex_node->data_id_));

	if (it == vertex_map_.end())
	{
		Vertex<StateType> *new_vertex = new Vertex<StateType>(vertex_node);
		//vertex_map_[vertex_node->data_id_] = new_vertex;
		vertex_map_.insert(std::make_pair(vertex_node->data_id_, new_vertex));
		return new_vertex;
	}

	return it->second;
}

/// This function creates a vertex in the graph that associates with the given node.
/// The set of functions AddVertex() are only supposed to be used with incremental a* search.
template <typename StateType>
template <class T, typename std::enable_if<!std::is_pointer<T>::value>::type *>
Vertex<StateType> *Graph_t<StateType>::AddVertex(T vertex_node)
{
	Vertex<StateType> *new_vertex = new Vertex<StateType>(vertex_node);
	//vertex_map_[vertex_node.data_id_] = new_vertex;
	vertex_map_.insert(std::make_pair(vertex_node.data_id_, new_vertex));
	return new_vertex;
}

template <typename StateType>
template <class T, typename std::enable_if<std::is_pointer<T>::value>::type *>
Vertex<StateType> *Graph_t<StateType>::AddVertex(T vertex_node)
{
	Vertex<StateType> *new_vertex = new Vertex<StateType>(vertex_node);
	//vertex_map_[vertex_node->data_id_] = new_vertex;
	vertex_map_.insert(std::make_pair(vertex_node->data_id_, new_vertex));
	return new_vertex;
}

/// This function checks if a vertex exists in the graph.
///	If yes, the functions returns the pointer of the existing vertex,
///	otherwise it returns nullptr.
template <typename StateType>
template <class T, typename std::enable_if<!std::is_pointer<T>::value>::type *>
Vertex<StateType> *Graph_t<StateType>::SearchVertex(T vertex_node)
{
	auto it = vertex_map_.find((uint64_t)(vertex_node.data_id_));

	if (it == vertex_map_.end())
		return nullptr;
	else
		return it->second;
}

template <typename StateType>
template <class T, typename std::enable_if<std::is_pointer<T>::value>::type *>
Vertex<StateType> *Graph_t<StateType>::SearchVertex(T vertex_node)
{
	auto it = vertex_map_.find((uint64_t)(vertex_node->data_id_));

	if (it == vertex_map_.end())
		return nullptr;
	else
		return it->second;
}
}

#endif /* GRAPH_IMPL_H */
