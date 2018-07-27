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

// Availabe class templates
template <typename StateType, typename TransitionType = double>
class Graph_t;

template <typename StateType, typename TransitionType = double>
class Vertex_t;

template <typename StateType, typename TransitionType = double>
class Edge_t;

template <typename StateType>
using Path_t = std::vector<StateType>;

/// Graph class template.
template <typename StateType, typename TransitionType>
class Graph_t
{
public:
  /// Default Graph_t constructor.
  /// StateType must provide a function with the signature: int64_t GetUniqueID() const
  Graph_t()
  {
    typedef typename std::remove_const<typename std::remove_reference<typename std::remove_pointer<StateType>::type>::type>::type TestType;
    static_assert(HasIDGenFunc<TestType>::value, "function required in StateType: int64_t GetUniqueID() const");
    static_assert(!std::is_reference<StateType>::value, "Graph_t with reference type template parameter is not allowed");
  }

  /// Default Graph_t destructor. Graph_t class is only responsible for the memory recycling of Vertex_t and Edge_t
  /// objects. The node, such as a quadtree node or a square cell, which each vertex is associated
  ///  with needs to be recycled separately, for example by the quadtree/square_grid class.
  ~Graph_t();

  typedef Vertex_t<StateType, TransitionType> VertexType;
  typedef Edge_t<StateType, TransitionType> EdgeType;
  typedef Path_t<StateType> PathType;

  // vertex_iterator can be used to access vertices in the graph
  // edge_iterator can be used to access edges in each vertex
  class vertex_iterator;
  typedef typename VertexType::edge_iterator edge_iterator;

public:
  /// This function creates a vertex in the graph that associates with the given node.
  void AddVertex(StateType state);

  /// This function checks if a vertex exists in the graph and remove it if presents.
  template <class T = StateType, typename std::enable_if<!std::is_integral<T>::value>::type * = nullptr>
  void RemoveVertex(StateType state);
  void RemoveVertex(int64_t state_id);

  /// This function return the vertex pointer with specified id
  inline VertexType *GetVertex(int64_t vertex_id)
  {
    return GetVertexFromID(vertex_id);
  }

    /// This function return the vertex pointer with specified state
  template <class T = StateType, typename std::enable_if<!std::is_integral<T>::value>::type * = nullptr>
  inline VertexType *GetVertex(T state)
  {
    return GetVertexFromState(state);
  }

  /* Directed Graph */
  /// This function is used to add a directed edge connecting two nodes
  void AddEdge(StateType src_node, StateType dst_node, TransitionType cost);

  /// This function is used to remove the directed edge from src_node to dst_node.
  bool RemoveEdge(StateType src_node, StateType dst_node);

  /* Undirected Graph */
  /// This function is used to add an undirected edge connecting two nodes
  void AddUndirectedEdge(StateType src_node, StateType dst_node, TransitionType cost);

  /// This function is used to remove the edge from src_node to dst_node.
  bool RemoveUndirectedEdge(StateType src_node, StateType dst_node);

  /// This functions is used to access all edges of a graph
  inline std::vector<edge_iterator> GetAllEdges() const
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
  /// This function is used to reset states of all vertice for a new search
  void ResetGraphVertices();

  /// This function removes all edges and vertices in the graph
  void ClearGraph();

private:
#ifndef USE_UNORDERED_MAP
  typedef std::map<int64_t, VertexType *> VertexMapType;
  typedef VertexMapType::iterator VertexMapTypeIterator;
#else
  typedef std::unordered_map<int64_t, VertexType *> VertexMapType;
  typedef typename VertexMapType::iterator VertexMapTypeIterator;
#endif
  VertexMapType vertex_map_;

  /// This function returns the vertex associated with the given state.
  ///	If a vertex has already been created for this state, the functions
  ///   returns the pointer of the existing vertex.
  /// Otherwise, it creates a new vertex.
  VertexType *FindOrCreateVertex(StateType state);

  /// This function finds and returns the vertex with specified id.
  ///	If exists, the functions returns the pointer of the existing vertex.
  ///	Otherwise, it returns nullptr.
  VertexType *GetVertexFromID(int64_t vertex_id);

  /// This function finds and returns the vertex with specified state.
  ///	If exists, the functions returns the pointer of the existing vertex.
  ///	Otherwise, it returns nullptr.
  VertexType *GetVertexFromState(StateType state);

  /// This function returns the unique ID of given state.
  template <class T = StateType, typename std::enable_if<!std::is_pointer<T>::value>::type * = nullptr>
  inline int64_t GetStateID(T state)
  {
    return state.GetUniqueID();
  }

  template <class T = StateType, typename std::enable_if<std::is_pointer<T>::value>::type * = nullptr>
  inline int64_t GetStateID(T state)
  {
    return state->GetUniqueID();
  }

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

    VertexType *operator->() { return (VertexType *const)(VertexMapTypeIterator::operator->()->second); }
    VertexType &operator*() { return *(VertexMapTypeIterator::operator*().second); }
  };

  vertex_iterator vertex_begin() { return vertex_iterator(vertex_map_.begin()); }
  vertex_iterator vertex_end() { return vertex_iterator(vertex_map_.end()); }

  /// This function return the vertex iterator with specified id
  inline vertex_iterator find(int64_t vertex_id)
  {
    return vertex_iterator(vertex_map_.find(vertex_id));
  }

  template <class T = StateType, typename std::enable_if<!std::is_integral<T>::value>::type * = nullptr>
  inline vertex_iterator find(T state)
  {
    return vertex_iterator(vertex_map_.find(GetStateID(state)));
  }
};
} // namespace librav

#include "graph/details/graph_impl.hpp"

#endif /* GRAPH_HPP */
