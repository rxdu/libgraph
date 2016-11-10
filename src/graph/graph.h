/*
 * graph.h
 *
 *  Created on: Dec 9, 2015
 *      Author: rdu
 *
 *  Description:
 *  	1. This file defines template classes of the graph structure.
 *  	The basic elements include:
 *  	 	a. Vertex
 *  	 	b. Edge
 *  	 	c. Graph
 *
 *  	Note that all above 3 concepts have a "type". This type is determined by "nodes" used
 *  	to construct the graph. For example, a graph is built to represent the connectivities
 *  	among the leaf nodes of a quad tree. Then we have the following three types of graph
 *  	elements:
 *  		a. Vertex<QuadTreeNode>
 *  		b. Edge<Vertex<QuadTreeNode>>
 *  		c. Graph<QuadTreeNode>
 *
 *  	A vertex must be associated with a node (such as the tree node); an edge connects two
 *  	vertices; a graph is a collection of vertices and the edges of each vertex.
 *
 *  	2. The graph uses unique IDs to index its vertices. So it's required that the
 *  	node used to construct the graph should have an unique ID with the name "node_id_".
 *  	Refer to the definition of "struct ExampleNode" defined below.
 *
 *  	3. A visualized illustration of the graph structure
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
 *
 */

#ifndef SRC_GRAPH_GRAPH_H_
#define SRC_GRAPH_GRAPH_H_

#include <map>
#include <vector>
#include <cstdint>
#include <type_traits>

#include "graph/vertex.h"
#include "graph/astar.h"
#include "graph/bds_base.h"

namespace srcl_ctrl {

template<typename T>
class Graph;

template<typename T>
using Graph_t = Graph<T>;

template<typename T>
using Vertex_t = Vertex<T>;

template<typename T>
using Edge_t = Edge<Vertex<T>*>;

template<typename T>
using Path_t = std::vector<Vertex<T>*>;

/****************************************************************************/
/*								 Graph										*/
/****************************************************************************/
/// A graph data structure template.
template<typename BundledStructType>
class Graph
{
public:
	/// Graph constructor.
	Graph(){};
	/// Graph destructor. Graph class is only responsible for the memory recycling of Vertex and Edge
	/// objects. The node, such as a quadtree node or a square cell, which each vertex is associated
	///  with needs to be recycled separately, for example by the quadtree/square_grid class.
	~Graph(){
		for(auto it = vertex_map_.begin(); it != vertex_map_.end(); it++)
			delete it->second;
	};

private:
	std::map<uint64_t, Vertex<BundledStructType>*> vertex_map_;
	AStar<Vertex<BundledStructType>> astar_;

private:
	/// This function checks if a vertex already exists in the graph.
	///	If yes, the functions returns the index of the existing vertex,
	///	otherwise it creates a new vertex.
	template<class T = BundledStructType, typename std::enable_if<!std::is_pointer<T>::value>::type* = nullptr>
	Vertex<BundledStructType>* GetVertex(BundledStructType vertex_node)
	{
		auto it = vertex_map_.find((uint64_t)(vertex_node.data_id_));

		if(it == vertex_map_.end())
		{
			Vertex<BundledStructType>* new_vertex = new Vertex<BundledStructType>(vertex_node);
			vertex_map_[vertex_node.data_id_] = new_vertex;
			return new_vertex;
		}

		return it->second;
	}

	template<class T = BundledStructType, typename std::enable_if<std::is_pointer<T>::value>::type* = nullptr>
	Vertex<BundledStructType>* GetVertex(BundledStructType vertex_node)
	{
		auto it = vertex_map_.find((uint64_t)(vertex_node->data_id_));

		if(it == vertex_map_.end())
		{
			Vertex<BundledStructType>* new_vertex = new Vertex<BundledStructType>(vertex_node);
			vertex_map_[vertex_node->data_id_] = new_vertex;
			return new_vertex;
		}

		return it->second;
	}

	/// This function is used to reset the vertices for a new search
	void ResetGraphVertices()
	{
		for(const auto& vertex_pair: vertex_map_)
			vertex_pair.second->ClearVertexSearchInfo();
	};

public:
	/// This function removes all edges and vertices in the graph
	void ClearGraph()
	{
		for(auto it = vertex_map_.begin(); it != vertex_map_.end(); it++)
			delete it->second;
		vertex_map_.clear();
	}

	/// This function is used to create a graph by adding edges connecting two nodes
	void AddEdge(BundledStructType src_node, BundledStructType dst_node, double cost)
	{
		Vertex<BundledStructType>* src_vertex = GetVertex(src_node);
		Vertex<BundledStructType>* dst_vertex = GetVertex(dst_node);

		Edge<Vertex<BundledStructType>*> new_edge(src_vertex, dst_vertex,cost);
		src_vertex->edges_.push_back(new_edge);
	};

	/// This functions is used to access all vertices of a graph
	std::vector<Vertex<BundledStructType>*> GetGraphVertices() const
	{
		std::vector<Vertex<BundledStructType>*> vertices;

		for(auto it = vertex_map_.begin(); it != vertex_map_.end(); it++)
		{
			vertices.push_back(it->second);
		}

		return vertices;
	};

	/// This functions is used to access all edges of a graph
	std::vector<Edge<Vertex<BundledStructType>*>> GetGraphEdges() const
	{
		std::vector<Edge<Vertex<BundledStructType>*>> edges;

		for(auto it = vertex_map_.begin(); it != vertex_map_.end(); it++)
		{
			Vertex<BundledStructType>* vertex = it->second;
			for(auto ite = vertex->edges_.begin(); ite != vertex->edges_.end(); ite++) {
				edges.push_back(*ite);
			}
		}

		return edges;
	};

	/// This functions is used to access all edges of a graph
	std::vector<Edge<Vertex<BundledStructType>*>> GetGraphUndirectedEdges() const
	{
		std::vector<Edge<Vertex<BundledStructType>*>> edges;

		for(auto it = vertex_map_.begin(); it != vertex_map_.end(); it++)
		{
			Vertex<BundledStructType>* vertex = it->second;

			for(auto ite = vertex->edges_.begin(); ite != vertex->edges_.end(); ite++) {
				bool edge_existed = false;

				for(auto& itedge : edges)
				{
					if(itedge -= (*ite)) {
						edge_existed = true;
						break;
					}
				}

				if(!edge_existed)
					edges.push_back(*ite);
			}
		}

		return edges;
	};

	/// This function return the vertex with specified id
	Vertex<BundledStructType>* GetVertexFromID(uint64_t vertex_id)
	{
		auto it = vertex_map_.find(vertex_id);

		if(it != vertex_map_.end())
			return (*it).second;
		else
			return nullptr;
	}

	/// Perform A* Search and return a path represented by a serious of vertices
	std::vector<Vertex<BundledStructType>*> AStarSearch(Vertex<BundledStructType>* start, Vertex<BundledStructType>* goal)
	{
		// clear previous search information before new search
		ResetGraphVertices();

		// do a* search and return search result
		return astar_.Search(start, goal);
	}

	std::vector<Vertex<BundledStructType>*> AStarSearch(uint64_t start_id, uint64_t goal_id)
	{
		std::vector<Vertex<BundledStructType>*> path;
		Vertex<BundledStructType> *start = this->GetVertexFromID(start_id);
		Vertex<BundledStructType> *goal = this->GetVertexFromID(goal_id);

		// do a* search and return search result
		if(start != nullptr && goal != nullptr)
			return this->AStarSearch(start, goal);
		else
			return path;
	}
};

}

#endif /* SRC_GRAPH_GRAPH_H_ */
