/* 
 * graph_impl.hpp
 * 
 * Created on: Nov 13, 2017 23:08
 *  Description:
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef GRAPH_IMPL_HPP
#define GRAPH_IMPL_HPP

namespace librav
{

/****************************************************************************/
/*								 Graph_t									*/
/****************************************************************************/
template <typename StateType, typename TransitionType>
Graph_t<StateType, TransitionType>::~Graph_t()
{
	for (auto &vertex_pair : vertex_map_)
		delete vertex_pair.second;
};

/// This function is used to reset the vertices for a new search
template <typename StateType, typename TransitionType>
void Graph_t<StateType, TransitionType>::ResetGraphVertices()
{
	for (auto &vertex_pair : vertex_map_)
		vertex_pair.second->ClearVertexSearchInfo();
};

/// This function removes all edges and vertices in the graph
template <typename StateType, typename TransitionType>
void Graph_t<StateType, TransitionType>::ClearGraph()
{
	for (auto &vertex_pair : vertex_map_)
		delete vertex_pair.second;
	vertex_map_.clear();
}

/// This function return the vertex with specified id
template <typename StateType, typename TransitionType>
Vertex_t<StateType, TransitionType> *Graph_t<StateType, TransitionType>::GetVertexFromID(uint64_t vertex_id)
{
	auto it = vertex_map_.find(vertex_id);

	if (it != vertex_map_.end())
		return (*it).second;
	else
		return nullptr;
};

/// This function is used to create a graph by adding edges connecting two nodes
template <typename StateType, typename TransitionType>
void Graph_t<StateType, TransitionType>::AddEdge(StateType src_node, StateType dst_node, TransitionType cost)
{
	Vertex_t<StateType, TransitionType> *src_vertex = GetVertex(src_node);
	Vertex_t<StateType, TransitionType> *dst_vertex = GetVertex(dst_node);

	if (src_vertex->CheckNeighbour(dst_vertex))
		return;

	// store information for deleting vertex
	dst_vertex->vertices_from_.push_back(src_vertex);
	
	src_vertex->edges_to_.emplace_back(src_vertex, dst_vertex, cost);
};

/// This function is used to remove the edge from src_node to dst_node.
template <typename StateType, typename TransitionType>
bool Graph_t<StateType, TransitionType>::RemoveEdge(StateType src_node, StateType dst_node)
{
	Vertex_t<StateType, TransitionType> *src_vertex = SearchVertex(src_node);
	Vertex_t<StateType, TransitionType> *dst_vertex = SearchVertex(dst_node);

	if ((src_vertex != nullptr) && (dst_vertex != nullptr))
	{
		auto idx = src_vertex->edges_to_.end();
		bool found_edge = false;
		for (auto it = src_vertex->edges_to_.begin(); it != src_vertex->edges_to_.end(); it++)
		{
			if ((*it).dst_ == dst_vertex)
			{
				idx = it;
				found_edge = true;
			}
		}

		if (found_edge)
			src_vertex->edges_to_.erase(idx);

		return found_edge;
	}
	else
		return false;
};

/// This function is used to create a graph by adding edges connecting two nodes
template <typename StateType, typename TransitionType>
void Graph_t<StateType, TransitionType>::AddUndirectedEdge(StateType src_node, StateType dst_node, TransitionType cost)
{
	AddEdge(src_node, dst_node);
	AddEdge(dst_node, src_node);
}

/// This function is used to remove the edge from src_node to dst_node.
template <typename StateType, typename TransitionType>
bool Graph_t<StateType, TransitionType>::RemoveUndirectedEdge(StateType src_node, StateType dst_node)
{
	RemoveEdge(src_node, dst_node);
	RemoveEdge(dst_node, src_node);
}

/// This function creates a vertex in the graph that associates with the given node.
/// The set of functions AddVertex() are only supposed to be used with incremental a* search.
template <typename StateType, typename TransitionType>
template <class T, typename std::enable_if<!std::is_pointer<T>::value>::type *>
Vertex_t<StateType, TransitionType> *Graph_t<StateType, TransitionType>::AddVertex(T vertex_node)
{
	Vertex_t<StateType, TransitionType> *new_vertex = new Vertex_t<StateType, TransitionType>(vertex_node);
	//vertex_map_[vertex_node.GetUniqueID()] = new_vertex;
	vertex_map_.insert(std::make_pair(vertex_node.GetUniqueID(), new_vertex));
	return new_vertex;
}

template <typename StateType, typename TransitionType>
template <class T, typename std::enable_if<std::is_pointer<T>::value>::type *>
Vertex_t<StateType, TransitionType> *Graph_t<StateType, TransitionType>::AddVertex(T vertex_node)
{
	Vertex_t<StateType, TransitionType> *new_vertex = new Vertex_t<StateType, TransitionType>(vertex_node);
	//vertex_map_[vertex_node->GetUniqueID()] = new_vertex;
	vertex_map_.insert(std::make_pair(vertex_node->GetUniqueID(), new_vertex));
	return new_vertex;
}

template <typename StateType, typename TransitionType>
template <class T, typename std::enable_if<!std::is_pointer<T>::value>::type *>
void Graph_t<StateType, TransitionType>::RemoveVertex(T vertex_node)
{
	auto it = vertex_map_.find((uint64_t)(vertex_node.GetUniqueID()));

	// unknown vertex, no need to remove
	if (it == vertex_map_.end())
		return;

	for (auto &asv : it->second->vertices_from_)
		for (auto eit = asv->edges_to_.begin(); eit != asv->edges_to_.end(); eit++)
		{
			if ((*eit).dst_ == it->second)
			{
				asv->edges_to_.erase(eit);
				break;
			}
		}

	auto vptr = it->second;
	vertex_map_.erase(it);
	delete vptr;
};

template <typename StateType, typename TransitionType>
template <class T, typename std::enable_if<std::is_pointer<T>::value>::type *>
void Graph_t<StateType, TransitionType>::RemoveVertex(T vertex_node)
{
	auto it = vertex_map_.find((uint64_t)(vertex_node->GetUniqueID()));

	// unknown vertex, no need to remove
	if (it == vertex_map_.end())
		return;

	for (auto &asv : it->second->vertices_from_)
		for (auto eit = asv->edges_to_.begin(); eit != asv->edges_to_.end(); eit++)
		{
			if ((*eit).dst_ == it->second)
			{
				asv->edges_to_.erase(eit);
				break;
			}
		}

	auto vptr = it->second;
	vertex_map_.erase(it);
	delete vptr;
}

/// This functions is used to access all vertices of a graph
template <typename StateType, typename TransitionType>
std::vector<Vertex_t<StateType, TransitionType> *> Graph_t<StateType, TransitionType>::GetGraphVertices() const
{
	std::vector<Vertex_t<StateType, TransitionType> *> vertices;

	for (auto &vertex_pair : vertex_map_)
		vertices.push_back(vertex_pair.second);

	return vertices;
};

/// This functions is used to access all edges of a graph
template <typename StateType, typename TransitionType>
std::vector<Edge_t<StateType, TransitionType>> Graph_t<StateType, TransitionType>::GetGraphEdges() const
{
	std::vector<Edge_t<StateType, TransitionType>> edges;

	for (auto &vertex_pair : vertex_map_)
	{
		Vertex_t<StateType, TransitionType> *vertex = vertex_pair.second;
		for(auto edge:vertex->edges_to_)
			edges.push_back(edge);
	}

	return edges;
};

/// This functions is used to access all edges of a graph
template <typename StateType, typename TransitionType>
std::vector<Edge_t<StateType, TransitionType>> Graph_t<StateType, TransitionType>::GetGraphUndirectedEdges() const
{
	std::vector<Edge_t<StateType, TransitionType>> edges;

	for (auto &vertex_pair : vertex_map_)
	{
		Vertex_t<StateType, TransitionType> *vertex = vertex_pair.second;

		for (auto ite = vertex->edges_to_.begin(); ite != vertex->edges_to_.end(); ite++)
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

template <typename StateType, typename TransitionType>
template <class T, typename std::enable_if<!std::is_pointer<T>::value>::type *>
Vertex_t<StateType, TransitionType> *Graph_t<StateType, TransitionType>::GetVertex(T vertex_node)
{
	auto it = vertex_map_.find((uint64_t)(vertex_node.GetUniqueID()));

	if (it == vertex_map_.end())
	{
		Vertex_t<StateType, TransitionType> *new_vertex = new Vertex_t<StateType, TransitionType>(vertex_node);
		//vertex_map_[vertex_node.GetUniqueID()] = new_vertex;
		vertex_map_.insert(std::make_pair(vertex_node.GetUniqueID(), new_vertex));
		return new_vertex;
	}

	return it->second;
}

template <typename StateType, typename TransitionType>
template <class T, typename std::enable_if<std::is_pointer<T>::value>::type *>
Vertex_t<StateType, TransitionType> *Graph_t<StateType, TransitionType>::GetVertex(T vertex_node)
{
	auto it = vertex_map_.find((uint64_t)(vertex_node->GetUniqueID()));

	if (it == vertex_map_.end())
	{
		Vertex_t<StateType, TransitionType> *new_vertex = new Vertex_t<StateType, TransitionType>(vertex_node);
		//vertex_map_[vertex_node->GetUniqueID()] = new_vertex;
		vertex_map_.insert(std::make_pair(vertex_node->GetUniqueID(), new_vertex));
		return new_vertex;
	}

	return it->second;
}

/// This function checks if a vertex exists in the graph.
///	If yes, the functions returns the pointer of the existing vertex,
///	otherwise it returns nullptr.
template <typename StateType, typename TransitionType>
template <class T, typename std::enable_if<!std::is_pointer<T>::value>::type *>
Vertex_t<StateType, TransitionType> *Graph_t<StateType, TransitionType>::SearchVertex(T vertex_node)
{
	auto it = vertex_map_.find((uint64_t)(vertex_node.GetUniqueID()));

	if (it == vertex_map_.end())
		return nullptr;
	else
		return it->second;
}

template <typename StateType, typename TransitionType>
template <class T, typename std::enable_if<std::is_pointer<T>::value>::type *>
Vertex_t<StateType, TransitionType> *Graph_t<StateType, TransitionType>::SearchVertex(T vertex_node)
{
	auto it = vertex_map_.find((uint64_t)(vertex_node->GetUniqueID()));

	if (it == vertex_map_.end())
		return nullptr;
	else
		return it->second;
}
}

#endif /* GRAPH_IMPL_HPP */
