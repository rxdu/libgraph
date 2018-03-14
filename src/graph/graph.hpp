/* 
 * graph.hpp
 * 
 * Created on: Dec 9, 2015
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef GRAPH_HPP
#define GRAPH_HPP

#define USE_UNORDERED_MAP

#ifndef USE_UNORDERED_MAP
#include <map>
#else
#include <unordered_map>
#endif
#include <vector>
#include <cstdint>
#include <type_traits>

#include "graph/edge.hpp"
#include "graph/vertex.hpp"
#include "graph/details/helper_func.hpp"

namespace librav
{

// Only types ended with "_t" should be used in user applications
template <typename StateType, typename TransitionType = double>
class Graph_t;

template <typename StateType, typename TransitionType = double>
class Vertex_t;

template <typename StateType, typename TransitionType = double>
using Edge_t = Edge<Vertex_t<StateType, TransitionType> *, TransitionType>;

template <typename StateType, typename TransitionType = double>
using Path_t = std::vector<typename Graph_t<StateType, TransitionType>::vertex_iterator>;

/// A graph data structure template.
template <typename StateType, typename TransitionType>
class Graph_t
{
public:
  /// Graph_t constructor.
  /// StateType must provide a function with the signature: int64_t GetUniqueID() const
  Graph_t()
  {
    typedef typename std::remove_const<typename std::remove_reference<typename std::remove_pointer<StateType>::type>::type>::type TestType;
    static_assert(HasIDGenFunc<TestType>::value, "function required in StateType: int64_t GetUniqueID() const");
  }

  /// Graph_t destructor. Graph_t class is only responsible for the memory recycling of Vertex_t and Edge_t
  /// objects. The node, such as a quadtree node or a square cell, which each vertex is associated
  ///  with needs to be recycled separately, for example by the quadtree/square_grid class.
  ~Graph_t();

  typedef Vertex_t<StateType, TransitionType> VertexType;
  typedef Edge_t<StateType, TransitionType> EdgeType;
  typedef std::vector<StateType> PathType;

  // vertex_iterator can be used to access vertices in the graph
  // edge_iterator can be used to access edges in each vertex
  class vertex_iterator;
  typedef typename VertexType::edge_iterator edge_iterator;

  friend class AStar;
  friend class Dijkstra;

public:
  /// This function creates a vertex in the graph that associates with the given node.
  template <class T = StateType, typename std::enable_if<std::is_pointer<T>::value>::type * = nullptr>
  void AddVertex(T vertex_node);

  /// This function checks if a vertex exists in the graph and remove it if presents.
  template <class T = StateType, typename std::enable_if<!std::is_pointer<T>::value>::type * = nullptr>
  void RemoveVertex(T vertex_node);

  /* Directed Graph */
  /// This function is used to create a graph by adding edges connecting two nodes
  void AddEdge(StateType src_node, StateType dst_node, TransitionType cost);

  /// This function is used to remove the edge from src_node to dst_node.
  bool RemoveEdge(StateType src_node, StateType dst_node);

  /* Undirected Graph */
  /// This function is used to create a graph by adding edges connecting two nodes
  void AddUndirectedEdge(StateType src_node, StateType dst_node, TransitionType cost);

  /// This function is used to remove the edge from src_node to dst_node.
  bool RemoveUndirectedEdge(StateType src_node, StateType dst_node);

  /// This functions is used to access all edges of a graph
  std::vector<edge_iterator> GetAllEdges() const
  {
    std::vector<edge_iterator> edges;
    for (auto &vertex_pair : vertex_map_)
    {
      auto vertex = vertex_pair.second;
      for (auto it = vertex->edge_begin(); it != vertex->edge_end(); ++it)
        edges.push_back(it);
    }
    return edges;
  }

  /// Get total number of vertices in the graph
  int64_t GetGraphVertexNumber() const { return vertex_map_.size(); }

  /// Get total number of edges in the graph
  int64_t GetGraphEdgeNumber() const { return GetAllEdges().size(); }

  /* Utility functions */
  /// This function removes all edges and vertices in the graph
  void ClearGraph();

public:
  /* Same functions for pointer type State node */
  template <class T = StateType, typename std::enable_if<!std::is_pointer<T>::value>::type * = nullptr>
  void AddVertex(T vertex_node);

  template <class T = StateType, typename std::enable_if<std::is_pointer<T>::value>::type * = nullptr>
  void RemoveVertex(T vertex_node);

private:
#ifndef USE_UNORDERED_MAP
  typedef std::map<uint64_t, VertexType *> VertexMapType;
  typedef VertexMapType::iterator VertexMapTypeIterator;
#else
  typedef std::unordered_map<uint64_t, VertexType *> VertexMapType;
  typedef typename VertexMapType::iterator VertexMapTypeIterator;
#endif
  VertexMapType vertex_map_;

  /// This function is used to reset states of all vertice for a new search
  void ResetGraphVertices();

  /// This function return the vertex with specified id
  VertexType *GetVertexFromID(uint64_t vertex_id);

  /// This function checks if a vertex already exists in the graph.
  ///	If exists, the functions returns the pointer of the existing vertex,
  ///	otherwise it creates a new vertex.
  template <class T = StateType, typename std::enable_if<!std::is_pointer<T>::value>::type * = nullptr>
  VertexType *GetVertex(T vertex_node);

  template <class T = StateType, typename std::enable_if<std::is_pointer<T>::value>::type * = nullptr>
  VertexType *GetVertex(T vertex_node);

  /// This function checks if a vertex exists in the graph.
  ///	If exists, the functions returns the pointer of the existing vertex,
  ///	otherwise it returns nullptr.
  template <class T = StateType, typename std::enable_if<!std::is_pointer<T>::value && !std::is_integral<T>::value>::type * = nullptr>
  VertexType *FindVertex(T vertex_node);

  template <class T = StateType, typename std::enable_if<std::is_pointer<T>::value && !std::is_integral<T>::value>::type * = nullptr>
  VertexType *FindVertex(T vertex_node);

public:
  // Vertex iterator for easy access
  // Reference:
  //  [1] https://stackoverflow.com/a/16527081/2200873
  //  [2] https://stackoverflow.com/questions/1443793/iterate-keys-in-a-c-map/35262398#35262398
  class vertex_iterator : public VertexMapTypeIterator
  {
  public:
    vertex_iterator() : VertexMapTypeIterator(){};
    vertex_iterator(VertexMapTypeIterator s) : VertexMapTypeIterator(s){};
    vertex_iterator(const vertex_iterator &) = default;

    VertexType *operator->() { return (VertexType *const)(VertexMapTypeIterator::operator->()->second); }
    VertexType &operator*() { return *(VertexMapTypeIterator::operator*().second); }
  };

  vertex_iterator vertex_begin() { return vertex_iterator(vertex_map_.begin()); }
  vertex_iterator vertex_end() { return vertex_iterator(vertex_map_.end()); }

  /// This function return the vertex iterator with specified id
  vertex_iterator FindVertex(uint64_t vertex_id)
  {
    return vertex_iterator(vertex_map_.find(vertex_id));
  }
};
}

#include "graph/details/graph_impl.hpp"

#endif /* GRAPH_HPP */
