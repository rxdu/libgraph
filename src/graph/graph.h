/* 
 * graph.h
 * 
 * Created on: Nov 10, 2017 
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef GRAPH_H
#define GRAPH_H

#include <map>
#include <vector>
#include <cstdint>
#include <type_traits>

#include "graph/internal/bds_base.h"
#include "graph/vertex.h"

namespace librav
{

// Only types ended with "_t" should be used in user applications
template <typename T>
class Graph_t;

template <typename T>
class Vertex_t;

template <typename T>
using Edge_t = Edge<Vertex_t<T> *>;

template <typename T>
using Path_t = std::vector<Vertex_t<T> *>;

/// A graph data structure template.
template <typename StateType>
class Graph_t
{
  public:
    /// Graph_t constructor.
    Graph_t(){};
    /// Graph_t destructor. Graph_t class is only responsible for the memory recycling of Vertex_t and Edge_t
    /// objects. The node, such as a quadtree node or a square cell, which each vertex is associated
    ///  with needs to be recycled separately, for example by the quadtree/square_grid class.
    ~Graph_t();

  public:
    /// This function removes all edges and vertices in the graph
    void ClearGraph();

    /// This function is used to create a graph by adding edges connecting two nodes
    void AddEdge(StateType src_node, StateType dst_node, double cost);

    /// This function is used to remove the edge from src_node to dst_node.
    bool RemoveEdge(StateType src_node, StateType dst_node);

    /// This function checks if a vertex exists in the graph and remove it if presents.
    template <class T = StateType, typename std::enable_if<!std::is_pointer<T>::value>::type * = nullptr>
    void RemoveVertex(T vertex_node);

    /// This functions is used to access all vertices of a graph
    std::vector<Vertex_t<StateType> *> GetGraphVertices() const;

    /// This functions is used to access all edges of a graph
    std::vector<Edge<Vertex_t<StateType> *>> GetGraphEdges() const;

    /// This functions is used to access all edges of a graph
    std::vector<Edge<Vertex_t<StateType> *>> GetGraphUndirectedEdges() const;

    /// This function return the vertex with specified id
    Vertex_t<StateType> *GetVertexFromID(uint64_t vertex_id);

  private:
    std::map<uint64_t, Vertex_t<StateType> *> vertex_map_;
    friend class AStar;

    /// This function checks if a vertex already exists in the graph.
    ///	If yes, the functions returns the pointer of the existing vertex,
    ///	otherwise it creates a new vertex.
    template <class T = StateType, typename std::enable_if<!std::is_pointer<T>::value>::type * = nullptr>
    Vertex_t<StateType> *GetVertex(T vertex_node);

    /// This function creates a vertex in the graph that associates with the given node.
    /// The set of functions AddVertex() are only supposed to be used with incremental a* search.
    template <class T = StateType, typename std::enable_if<!std::is_pointer<T>::value>::type * = nullptr>
    Vertex_t<StateType> *AddVertex(T vertex_node);

    /// This function checks if a vertex exists in the graph.
    ///	If yes, the functions returns the pointer of the existing vertex,
    ///	otherwise it returns nullptr.
    template <class T = StateType, typename std::enable_if<!std::is_pointer<T>::value>::type * = nullptr>
    Vertex_t<StateType> *SearchVertex(T vertex_node);

    /// This function is used to reset the vertices for a new search
    void ResetGraphVertices();

    /* Same functions for pointer type State node */
    template <class T = StateType, typename std::enable_if<std::is_pointer<T>::value>::type * = nullptr>
    void RemoveVertex(T vertex_node);

    template <class T = StateType, typename std::enable_if<std::is_pointer<T>::value>::type * = nullptr>
    Vertex_t<StateType> *GetVertex(T vertex_node);

    template <class T = StateType, typename std::enable_if<std::is_pointer<T>::value>::type * = nullptr>
    Vertex_t<StateType> *AddVertex(T vertex_node);

    template <class T = StateType, typename std::enable_if<std::is_pointer<T>::value>::type * = nullptr>
    Vertex_t<StateType> *SearchVertex(T vertex_node);
};

}

#include "graph/internal/graph_impl.h"

#endif /* GRAPH_H */
