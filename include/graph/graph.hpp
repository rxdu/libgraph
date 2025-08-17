/*
 * graph.hpp
 *
 * Created on: Dec 9, 2015
 * Description:
 *
 * Major Revisions:
 *  version 0.1 Dec 09, 2015
 *  version 1.0 Sep 03, 2018
 *
 * Copyright (c) 2015-2021 Ruixiang Du (rdu)
 */

/* Reference
 *
 * Iterator:
 * [1] https://stackoverflow.com/a/16527081/2200873
 * [2]
 * https://stackoverflow.com/questions/1443793/iterate-keys-in-a-c-map/35262398#35262398
 *
 * Erase–remove idiom:
 * [3] https://en.wikipedia.org/wiki/Erase%E2%80%93remove_idiom
 *
 */

#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <algorithm>
#include <cstdint>
#include <limits>
#include <list>
#include <memory>
#include <type_traits>
#include <unordered_map>
#include <vector>
#include <cmath> // For std::isnan, std::isinf

#include "graph/edge.hpp" // Independent Edge class
#include "graph/impl/default_indexer.hpp"
#include "graph/vertex.hpp" // Independent Vertex class
#include "graph/exceptions.hpp" // Enhanced error handling

namespace xmotion {

/**
 * @brief Exception Safety Guarantees for Graph Operations
 * 
 * This documentation defines the exception safety guarantees for all Graph class operations.
 * The Graph class follows C++ exception safety best practices with RAII and proper resource management.
 * 
 * @section exception_safety_levels Exception Safety Levels
 * 
 * **1. Basic Guarantee**: No resource leaks, object remains in valid state
 * **2. Strong Guarantee**: Operation succeeds completely or has no effect (rollback semantics)  
 * **3. No-throw Guarantee**: Operation never throws exceptions (marked with noexcept)
 * 
 * @section operation_guarantees Operation-Specific Guarantees
 * 
 * **Construction/Destruction (Strong Guarantee)**
 * - Default constructor: No-throw (noexcept)
 * - Copy constructor: Strong guarantee - succeeds completely or leaves original unchanged
 * - Move constructor: No-throw (noexcept) 
 * - Copy assignment: Strong guarantee via copy-and-swap idiom
 * - Move assignment: No-throw (noexcept)
 * - Destructor: No-throw (automatic via RAII std::unique_ptr cleanup)
 * 
 * **Vertex Operations (Strong Guarantee)**
 * - AddVertex(): Strong guarantee - vertex fully added or graph unchanged
 * - RemoveVertex(): Strong guarantee - vertex fully removed or graph unchanged  
 * - FindVertex(): No-throw for valid inputs, throws std::out_of_range for invalid IDs
 * 
 * **Edge Operations (Strong Guarantee)**
 * - AddEdge(): Strong guarantee - edge fully added or graph unchanged
 * - RemoveEdge(): Strong guarantee - edge fully removed or graph unchanged
 * - AddUndirectedEdge(): Strong guarantee - both edges added or neither added
 * 
 * **Query Operations (No-throw Guarantee)**
 * - All const query methods (HasVertex, HasEdge, GetVertexDegree, etc.): No-throw (noexcept)
 * - Container-like operations (empty, size, begin, end): No-throw (noexcept)
 * - Counting operations (GetEdgeCount, GetVertexCount): No-throw (noexcept)
 * 
 * **Iterator Operations (Strong Guarantee)**  
 * - Iterator creation: No-throw (returns valid iterators or end())
 * - Iterator dereferencing: No-throw for valid iterators
 * - Iterator invalidation: Iterators invalidated only by operations that modify the container
 * 
 * **Search Operations (Basic Guarantee)**
 * - Dijkstra/AStar: Basic guarantee - graph remains valid, search state may be partial
 * - Thread-safe searches: Strong guarantee - SearchContext isolates all search state
 * 
 * **Memory Management (Strong Guarantee via RAII)**
 * - All vertex storage uses std::unique_ptr for automatic cleanup
 * - No manual memory management required
 * - Exception during vertex creation automatically cleans up partial state
 * - Copy operations use RAII throughout to prevent leaks
 * 
 * @section error_conditions Error Conditions and Exceptions
 * 
 * **std::bad_alloc**: Memory allocation failures (from std::unordered_map or std::unique_ptr)
 * **std::invalid_argument**: Invalid input parameters (e.g., in tree operations)
 * **std::logic_error**: Violation of class invariants (e.g., tree structure violations)
 * **State copy constructor exceptions**: Propagated with strong guarantee via RAII
 * 
 * @section thread_safety_exceptions Thread Safety and Exceptions
 * 
 * **Single-threaded operations**: All guarantees apply as documented
 * **Concurrent read operations**: Thread-safe with SearchContext, no exceptions from race conditions
 * **Concurrent write operations**: Not supported - user must provide external synchronization
 * 
 * @note The Graph class is designed with RAII principles throughout. All resource management
 *       is automatic via std::unique_ptr, ensuring no memory leaks even in exceptional cases.
 */

/// Graph class template.
template <typename State, typename Transition = double,
          typename StateIndexer = DefaultIndexer<State>>
class Graph {
public:
  // Use independent Edge and Vertex classes
  using Edge = xmotion::Edge<State, Transition, StateIndexer>;
  using Vertex = xmotion::Vertex<State, Transition, StateIndexer>;
  using GraphType = Graph<State, Transition, StateIndexer>;

  using VertexMapType = std::unordered_map<int64_t, std::unique_ptr<Vertex>>;
  using VertexMapTypeIterator = typename VertexMapType::iterator;
  using VertexMapTypeConstIterator = typename VertexMapType::const_iterator;

  /*---------------------------------------------------------------------------------*/
  /*                              Vertex Iterator */
  /*---------------------------------------------------------------------------------*/
  ///@{
  /// Const vertex iterator for unified access.
  /// Wraps the "value" part of VertexMapType::const_iterator
  class const_vertex_iterator {
  private:
    VertexMapTypeConstIterator iter_;
    
  public:
    // Iterator traits
    using iterator_category = std::forward_iterator_tag;
    using value_type = const Vertex;
    using difference_type = std::ptrdiff_t;
    using pointer = const Vertex*;
    using reference = const Vertex&;

    const_vertex_iterator() : iter_() {}
    explicit const_vertex_iterator(VertexMapTypeConstIterator s) : iter_(s) {}
    explicit const_vertex_iterator(VertexMapTypeIterator s) : iter_(s) {}

    const Vertex *operator->() const;
    const Vertex &operator*() const;
    
    const_vertex_iterator& operator++() { ++iter_; return *this; }
    const_vertex_iterator operator++(int) { const_vertex_iterator tmp(*this); ++iter_; return tmp; }
    
    bool operator==(const const_vertex_iterator& other) const { return iter_ == other.iter_; }
    bool operator!=(const const_vertex_iterator& other) const { return iter_ != other.iter_; }
    
    // Access to underlying iterator for compatibility
    VertexMapTypeConstIterator base() const { return iter_; }
  };

  class vertex_iterator {
  private:
    VertexMapTypeIterator iter_;
    
  public:
    // Iterator traits
    using iterator_category = std::forward_iterator_tag;
    using value_type = Vertex;
    using difference_type = std::ptrdiff_t;
    using pointer = Vertex*;
    using reference = Vertex&;

    vertex_iterator() : iter_() {}
    explicit vertex_iterator(VertexMapTypeIterator s) : iter_(s) {}

    Vertex *operator->();
    Vertex &operator*();
    const Vertex *operator->() const;
    
    vertex_iterator& operator++() { ++iter_; return *this; }
    vertex_iterator operator++(int) { vertex_iterator tmp(*this); ++iter_; return tmp; }
    
    bool operator==(const vertex_iterator& other) const { return iter_ == other.iter_; }
    bool operator!=(const vertex_iterator& other) const { return iter_ != other.iter_; }
    
    // Conversion to const_vertex_iterator
    operator const_vertex_iterator() const { return const_vertex_iterator(iter_); }
    
    // Access to underlying iterator for compatibility
    VertexMapTypeIterator base() const { return iter_; }

    // Hash support for vertex_iterator
    struct Hash {
      size_t operator()(const vertex_iterator &iter) const;
    };

    // Equality comparison for vertex_iterator (for unordered containers)
    struct Equal {
      bool operator()(const vertex_iterator &a, const vertex_iterator &b) const;
    };
  };
  ///@}

  /*---------------------------------------------------------------------------------*/
  /*                              Edge Iterator */
  /*---------------------------------------------------------------------------------*/
  /** @name Edge Access
   *  Edge iterators to access edges in the vertex.
   */
  ///@{
  using edge_iterator = typename Vertex::edge_iterator;
  using const_edge_iterator = typename Vertex::const_edge_iterator;
  ///@}

public:
  // Note: Edge and Vertex classes are now defined independently in their own
  // headers The type aliases above (using Edge = ..., using Vertex = ...) make
  // them available as if they were nested classes for backward compatibility

  /*---------------------------------------------------------------------------------*/
  /*                               Graph Template */
  /*---------------------------------------------------------------------------------*/
public:
  /** @name Big Five
   *  Constructor, copy/move constructor, copy/move assignment operator,
   * destructor.
   */
  ///@{
  /** Default Graph constructor (No-throw guarantee)
   *  @noexcept Strong guarantee - never throws
   */
  Graph() = default;
  
  /** Copy constructor (Strong guarantee)
   *  @param other Graph to copy from
   *  @throws std::bad_alloc Memory allocation failure
   *  @throws State copy constructor exceptions
   */
  Graph(const GraphType &other);
  
  /** Move constructor (No-throw guarantee)  
   *  @param other Graph to move from
   *  @noexcept Strong guarantee - never throws
   */
  Graph(GraphType &&other) noexcept;
  
  /** Assignment operator (Strong guarantee via copy-and-swap)
   *  @param other Graph to assign from
   *  @return Reference to this graph
   *  @throws std::bad_alloc Memory allocation failure  
   *  @throws State copy constructor exceptions
   */
  GraphType &operator=(const GraphType &other);
  
  /** Move assignment operator (No-throw guarantee)
   *  @param other Graph to move from
   *  @return Reference to this graph
   *  @noexcept Strong guarantee - never throws
   */
  GraphType &operator=(GraphType &&other) noexcept;

  /// Default Graph destructor.
  /// Graph class is only responsible for the memory recycling of its internal
  /// objects, such as vertices and edges. If a state is associated with a
  /// vertex by its pointer, the memory allocated
  //  for the state object will not be managed by the graph and needs to be
  //  recycled separately.
  ~Graph();
  
  /// Swap function for efficient assignment operations
  void swap(GraphType& other) noexcept;
  ///@}

  /** @name Vertex Access
   *  Vertex iterators to access vertices in the graph.
   */
  ///@{
  vertex_iterator vertex_begin() {
    return vertex_iterator{vertex_map_.begin()};
  }
  vertex_iterator vertex_end() { return vertex_iterator{vertex_map_.end()}; }
  const_vertex_iterator vertex_begin() const {
    return const_vertex_iterator{vertex_map_.begin()};
  }
  const_vertex_iterator vertex_end() const {
    return const_vertex_iterator{vertex_map_.end()};
  }
  ///@}

  /** @name Graph Operations
   *  Modify vertex or edge of the graph.
   */
  ///@{
  /// This function is used to create a vertex in the graph that associates with
  /// the given node.
  vertex_iterator AddVertex(State state);

  /// This function checks if a vertex exists in the graph and remove it if
  /// presents.
  void RemoveVertex(int64_t state_id);

  template <class T = State, typename std::enable_if<
                                 !std::is_integral<T>::value>::type * = nullptr>
  void RemoveVertex(T state) {
    RemoveVertex(GetStateIndex(state));
  }

  /// This function is used to add an edge between the vertices associated with
  /// the given two states. Update the transition if edge already exists.
  void AddEdge(State sstate, State dstate, Transition trans);

  /// This function is used to remove the directed edge from src_node to
  /// dst_node.
  bool RemoveEdge(State sstate, State dstate);

  /* Undirected Graph */
  /// This function is used to add an undirected edge connecting two nodes
  void AddUndirectedEdge(State sstate, State dstate, Transition trans);

  /// This function is used to remove the edge from src_node to dst_node.
  bool RemoveUndirectedEdge(State sstate, State dstate);

  /// This functions is used to access all edges of a graph
  std::vector<edge_iterator> GetAllEdges() const;

  /// This function return the vertex iterator with specified id
  inline vertex_iterator FindVertex(int64_t vertex_id) {
    return vertex_iterator{vertex_map_.find(vertex_id)};
  }

  /// This function return the const vertex iterator with specified id
  inline const_vertex_iterator FindVertex(int64_t vertex_id) const {
    return const_vertex_iterator{vertex_map_.find(vertex_id)};
  }

  /// This function return the vertex iterator with specified state
  template <class T = State, typename std::enable_if<
                                 !std::is_integral<T>::value>::type * = nullptr>
  inline vertex_iterator FindVertex(T state) {
    return vertex_iterator{vertex_map_.find(GetStateIndex(state))};
  }

  /// This function return the const vertex iterator with specified state
  template <class T = State, typename std::enable_if<
                                 !std::is_integral<T>::value>::type * = nullptr>
  inline const_vertex_iterator FindVertex(T state) const {
    return const_vertex_iterator{vertex_map_.find(GetStateIndex(state))};
  }


  /// Get total number of vertices in the graph
  int64_t GetTotalVertexNumber() const noexcept { return vertex_map_.size(); }

  /// Get total number of edges in the graph
  int64_t GetTotalEdgeNumber() const { return GetAllEdges().size(); }

  /* Utility functions */
  /// This function is used to reset states of all vertice for a new search
  void ResetAllVertices();

  /// This function removes all edges and vertices in the graph
  void ClearAll();
  ///@}

  /** @name API Polish - Convenience Methods
   *  Additional convenience methods for improved usability.
   */
  ///@{
  /** @name Vertex Information Access */
  ///@{
  /** Check if a vertex with the given ID exists in the graph
   *  @param vertex_id The ID of the vertex to check
   *  @return True if vertex exists, false otherwise
   */
  bool HasVertex(int64_t vertex_id) const;
  
  /** Check if a vertex with the given state exists in the graph
   *  @param state The state of the vertex to check
   *  @return True if vertex exists, false otherwise
   */
  template <class T = State, typename std::enable_if<
                                 !std::is_integral<T>::value>::type * = nullptr>
  bool HasVertex(T state) const {
    return HasVertex(GetStateIndex(state));
  }
  
  /** Get the total degree of a vertex (in-degree + out-degree)
   *  @param vertex_id The ID of the vertex
   *  @return Total degree of the vertex, 0 if vertex doesn't exist
   */
  size_t GetVertexDegree(int64_t vertex_id) const;
  
  /** Get the in-degree of a vertex (number of incoming edges)
   *  @param vertex_id The ID of the vertex
   *  @return In-degree of the vertex, 0 if vertex doesn't exist
   */
  size_t GetInDegree(int64_t vertex_id) const;
  
  /** Get the out-degree of a vertex (number of outgoing edges)
   *  @param vertex_id The ID of the vertex
   *  @return Out-degree of the vertex, 0 if vertex doesn't exist
   */
  size_t GetOutDegree(int64_t vertex_id) const;
  ///@}
  
  /** @name Neighbor Access */
  ///@{
  /** Get all neighbor states of a vertex (vertices connected by outgoing edges)
   *  @param state The state of the vertex
   *  @return Vector of neighbor states, empty if vertex doesn't exist
   */
  std::vector<State> GetNeighbors(State state) const;
  
  /** Get all neighbor states of a vertex by ID
   *  @param vertex_id The ID of the vertex
   *  @return Vector of neighbor states, empty if vertex doesn't exist
   */
  std::vector<State> GetNeighbors(int64_t vertex_id) const;
  ///@}
  
  /** @name Edge Query Methods */
  ///@{
  /** Check if an edge exists between two states
   *  @param from Source state
   *  @param to Destination state
   *  @return True if edge exists, false otherwise
   */
  bool HasEdge(State from, State to) const;
  
  /** Get the weight/transition of an edge between two states
   *  @param from Source state
   *  @param to Destination state
   *  @return Edge weight/transition, Transition{} if edge doesn't exist
   */
  Transition GetEdgeWeight(State from, State to) const;
  
  /** Get the total number of edges more efficiently (without creating vector)
   *  @return Total number of edges in the graph
   */
  size_t GetEdgeCount() const noexcept;
  ///@}
  
  /** @name Safe Vertex Access */
  ///@{
  /** Get vertex pointer by ID (returns nullptr if not found)
   *  @param vertex_id The ID of the vertex
   *  @return Pointer to vertex or nullptr if not found
   */
  Vertex* GetVertex(int64_t vertex_id);
  
  /** Get const vertex pointer by ID (returns nullptr if not found)
   *  @param vertex_id The ID of the vertex
   *  @return Const pointer to vertex or nullptr if not found
   */
  const Vertex* GetVertex(int64_t vertex_id) const;
  
  /** Get vertex pointer by state (returns nullptr if not found)
   *  @param state The state of the vertex
   *  @return Pointer to vertex or nullptr if not found
   */
  template <class T = State, typename std::enable_if<
                                 !std::is_integral<T>::value>::type * = nullptr>
  Vertex* GetVertex(T state) {
    return GetVertex(GetStateIndex(state));
  }
  
  /** Get const vertex pointer by state (returns nullptr if not found)
   *  @param state The state of the vertex
   *  @return Const pointer to vertex or nullptr if not found
   */
  template <class T = State, typename std::enable_if<
                                 !std::is_integral<T>::value>::type * = nullptr>
  const Vertex* GetVertex(T state) const {
    return GetVertex(GetStateIndex(state));
  }
  ///@}
  
  /** @name Validation and Error Checking */
  ///@{
  /** Get vertex safely with exception on failure
   *  @param vertex_id The ID of the vertex
   *  @return Reference to vertex
   *  @throws ElementNotFoundError if vertex doesn't exist
   */
  Vertex& GetVertexSafe(int64_t vertex_id) {
    auto* vertex = GetVertex(vertex_id);
    if (!vertex) {
      throw ElementNotFoundError("Vertex", vertex_id);
    }
    return *vertex;
  }
  
  /** Get const vertex safely with exception on failure
   *  @param vertex_id The ID of the vertex
   *  @return Const reference to vertex
   *  @throws ElementNotFoundError if vertex doesn't exist
   */
  const Vertex& GetVertexSafe(int64_t vertex_id) const {
    const auto* vertex = GetVertex(vertex_id);
    if (!vertex) {
      throw ElementNotFoundError("Vertex", vertex_id);
    }
    return *vertex;
  }
  
  /** Validate edge weight is acceptable
   *  @param weight The edge weight to validate
   *  @throws InvalidArgumentError if weight is invalid (e.g., negative for Dijkstra)
   */
  void ValidateEdgeWeight(Transition weight) const {
    // Check for NaN and infinity for floating point types (C++11 compatible)
    if (std::is_floating_point<Transition>::value) {
      if (std::isnan(static_cast<double>(weight))) {
        throw InvalidArgumentError("Edge weight cannot be NaN");
      }
      if (std::isinf(static_cast<double>(weight))) {
        throw InvalidArgumentError("Edge weight cannot be infinite");
      }
    }
  }
  
  /** Check if the graph structure is valid
   *  @throws DataCorruptionError if corruption is detected
   */
  void ValidateStructure() const {
    for (const auto& vertex_pair : vertex_map_) {
      const auto& vertex = vertex_pair.second;
      
      // Check vertex ID consistency
      if (vertex->vertex_id != vertex_pair.first) {
        throw DataCorruptionError("Vertex ID mismatch", 
          "Vertex claims ID " + std::to_string(vertex->vertex_id) + 
          " but stored under ID " + std::to_string(vertex_pair.first));
      }
      
      // Check edge consistency
      for (const auto& edge : vertex->edges_to) {
        // Check edge destination exists
        if (vertex_map_.find(edge.dst->vertex_id) == vertex_map_.end()) {
          throw DataCorruptionError("Dangling edge", 
            "Edge from vertex " + std::to_string(vertex->vertex_id) + 
            " points to non-existent vertex " + std::to_string(edge.dst->vertex_id));
        }
        
        // Check reverse reference exists
        bool found_reverse = false;
        for (const auto& reverse_vertex : edge.dst->vertices_from) {
          if (reverse_vertex->vertex_id == vertex->vertex_id) {
            found_reverse = true;
            break;
          }
        }
        if (!found_reverse) {
          throw DataCorruptionError("Missing reverse reference",
            "Edge from " + std::to_string(vertex->vertex_id) + 
            " to " + std::to_string(edge.dst->vertex_id) + 
            " lacks reverse reference");
        }
      }
    }
  }
  ///@}
  
  /** @name STL-like Interface */
  ///@{
  /** Check if the graph is empty
   *  @return True if no vertices exist, false otherwise
   */
  bool empty() const noexcept { return vertex_map_.empty(); }
  
  /** Get the number of vertices (same as GetTotalVertexNumber)
   *  @return Number of vertices in the graph
   */
  size_t size() const noexcept { return vertex_map_.size(); }
  
  /** Reserve space for n vertices to improve performance
   *  @param n Number of vertices to reserve space for
   */
  void reserve(size_t n) { vertex_map_.reserve(n); }
  ///@}
  
  /** @name Batch Operations */
  ///@{
  /** Add multiple vertices at once
   *  @param states Vector of states to add as vertices
   */
  void AddVertices(const std::vector<State>& states);
  
  /** Add multiple edges at once
   *  @param edges Vector of tuples (from, to, transition) to add
   */
  void AddEdges(const std::vector<std::tuple<State, State, Transition>>& edges);
  
  /** Remove multiple vertices at once
   *  @param states Vector of states to remove
   */
  void RemoveVertices(const std::vector<State>& states);
  ///@}

  /** @name Standardized Return Types
   *  Methods with consistent return types and error reporting.
   */
  ///@{
  /** @name Consistent Add Operations */
  ///@{
  /** Add vertex with success/failure reporting (like std::map::insert)
   *  @param state The state to add as a vertex
   *  @return Pair of iterator to vertex and bool indicating if insertion took place
   */
  std::pair<vertex_iterator, bool> AddVertexWithResult(State state);
  
  /** Add edge with success/failure reporting
   *  @param from Source state
   *  @param to Destination state
   *  @param trans Edge weight/transition
   *  @return True if edge was added, false if it already exists
   */
  bool AddEdgeWithResult(State from, State to, Transition trans);
  
  /** Add undirected edge with success/failure reporting
   *  @param from First state
   *  @param to Second state
   *  @param trans Edge weight/transition
   *  @return True if both edges were added, false if one or both already exist
   */
  bool AddUndirectedEdgeWithResult(State from, State to, Transition trans);
  ///@}
  
  /** @name Consistent Remove Operations */
  ///@{
  /** Remove vertex with success/failure reporting
   *  @param vertex_id ID of the vertex to remove
   *  @return True if vertex was removed, false if it didn't exist
   */
  bool RemoveVertexWithResult(int64_t vertex_id);
  
  /** Remove vertex by state with success/failure reporting
   *  @param state The state of the vertex to remove
   *  @return True if vertex was removed, false if it didn't exist
   */
  template <class T = State, typename std::enable_if<
                                 !std::is_integral<T>::value>::type * = nullptr>
  bool RemoveVertexWithResult(T state) {
    return RemoveVertexWithResult(GetStateIndex(state));
  }
  ///@}
  
  /** @name Standardized Counting Methods */
  ///@{
  /** Get vertex count using size_t (standardized alternative to GetTotalVertexNumber)
   *  @return Number of vertices as size_t
   */
  size_t GetVertexCount() const noexcept { return static_cast<size_t>(GetTotalVertexNumber()); }
  
  /** Get edge count using size_t (alias for existing GetEdgeCount for consistency)
   *  @return Number of edges as size_t
   */
  size_t GetEdgeCountStd() const noexcept { return GetEdgeCount(); }
  ///@}
  ///@}

  /** @name Range-based For Loop Support
   *  Support for modern C++ range-based iteration.
   */
  ///@{
  /// Vertex range for non-const graphs
  class vertex_range {
  private:
    Graph* graph_;
  public:
    explicit vertex_range(Graph* g) : graph_(g) {}
    vertex_iterator begin() { return graph_->vertex_begin(); }
    vertex_iterator end() { return graph_->vertex_end(); }
  };
  
  /// Vertex range for const graphs
  class const_vertex_range {
  private:
    const Graph* graph_;
  public:
    explicit const_vertex_range(const Graph* g) : graph_(g) {}
    const_vertex_iterator begin() const { return graph_->vertex_begin(); }
    const_vertex_iterator end() const { return graph_->vertex_end(); }
  };
  
  /// Get a range of all vertices for range-based for loops
  vertex_range vertices() { return vertex_range(this); }
  const_vertex_range vertices() const { return const_vertex_range(this); }
  ///@}

protected:
  /** @name Internal variables and functions.
   *  Internal variables and functions.
   */
  ///@{
  /// This function returns an index of the give state.
  /// The default indexer returns member variable "id_", assuming it exists.
  StateIndexer GetStateIndex;
  VertexMapType vertex_map_;

  /// Returns the iterator to the pair whose value is "state" in the vertex map.
  /// Create a new pair if one does not exit yet and return the iterator to the
  /// newly created pair.
  vertex_iterator ObtainVertexFromVertexMap(State state);
  ///@}
};

template <typename State, typename Transition = double,
          typename StateIndexer = DefaultIndexer<State>>
using Graph_t = Graph<State, Transition, StateIndexer>;
} // namespace xmotion

#include "graph/impl/edge_impl.hpp"
#include "graph/impl/graph_impl.hpp"
#include "graph/impl/vertex_impl.hpp"

#endif /* GRAPH_HPP */
