/*
 * search_context.hpp
 *
 * Created on: 2025
 * Description: Thread-safe search context for externalizing search state
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 *
 * THREAD SAFETY MODEL:
 * ====================
 * SearchContext enables thread-safe concurrent searches on a shared graph.
 *
 * Safe patterns:
 *   - Multiple threads can search the SAME graph concurrently
 *   - Each thread MUST have its own SearchContext instance
 *   - Search strategies CAN be shared across threads (they are stateless)
 *   - The graph MUST be read-only during concurrent searches
 *
 * Unsafe patterns:
 *   - Sharing a SearchContext across threads (data races)
 *   - Modifying the graph while searches are in progress (undefined behavior)
 *
 * Example (correct multi-threaded usage):
 *   const auto* graph = build_graph();  // Graph is immutable after this
 *   std::vector<std::thread> threads;
 *   for (int i = 0; i < NUM_THREADS; ++i) {
 *       threads.emplace_back([&graph, start, goal]() {
 *           SearchContext<...> context;  // Each thread has its own context
 *           auto path = Dijkstra::Search(graph, context, start, goal);
 *       });
 *   }
 *   for (auto& t : threads) t.join();
 *
 * For real-time applications, pre-allocate context:
 *   SearchContext<...> context(expected_graph_size);  // Avoid reallocations
 */

#ifndef SEARCH_CONTEXT_HPP
#define SEARCH_CONTEXT_HPP

#include <unordered_map>
#include <limits>
#include <vector>
#include <memory>
#include <string>
#include <type_traits>
#include "graph/exceptions.hpp"
#include "graph/attributes.hpp"

namespace xmotion {

/**
 * @brief Traits for providing default cost values for different types
 * 
 * Default implementation for arithmetic types uses std::numeric_limits::max().
 * For custom cost types, specialize this template explicitly.
 * 
 * Example specialization:
 * template<>
 * struct CostTraits<MyCustomCost> {
 *     static MyCustomCost infinity() { return MyCustomCost::max(); }
 * };
 */
template<typename T>
struct CostTraits {
    static T infinity() {
        // For arithmetic types, use numeric_limits
        static_assert(std::is_arithmetic<T>::value, 
            "CostTraits<T>::infinity() must be specialized for non-arithmetic cost types. "
            "Either specialize CostTraits<YourType> or add a static max() method to your type.");
        return std::numeric_limits<T>::max();
    }
};

/**
 * @brief Type alias for search result paths
 * @tparam State The state type stored in the path
 */
template <typename State>
using Path = std::vector<State>;

/**
 * @brief Rich search result with path, cost, and diagnostics
 *
 * Provides more information than just the path for robotics applications:
 * - Total path cost for comparing alternatives
 * - Nodes expanded for tuning heuristics
 * - Success/failure status
 *
 * @tparam State The state type stored in the path
 * @tparam Cost The cost type (defaults to double)
 */
template <typename State, typename Cost = double>
struct PathResult {
    Path<State> path;           ///< Sequence of states from start to goal
    Cost total_cost{};          ///< Total accumulated cost of the path
    size_t nodes_expanded{0};   ///< Number of vertices expanded during search
    bool found{false};          ///< True if a valid path was found

    /// Implicit conversion to bool for convenient checking
    explicit operator bool() const noexcept { return found; }

    /// Check if path is empty (no path found or trivial path)
    bool empty() const noexcept { return path.empty(); }

    /// Get path length (number of waypoints)
    size_t size() const noexcept { return path.size(); }
};

/**
 * @brief Multi-goal search result with path, cost, and goal identification
 *
 * Extends PathResult to identify which goal was reached when searching
 * for the nearest of multiple possible goals.
 *
 * @tparam State The state type stored in the path
 * @tparam Cost The cost type (defaults to double)
 */
template <typename State, typename Cost = double>
struct MultiGoalResult {
    Path<State> path;           ///< Sequence of states from start to reached goal
    Cost total_cost{};          ///< Total accumulated cost of the path
    size_t nodes_expanded{0};   ///< Number of vertices expanded during search
    bool found{false};          ///< True if any goal was reached
    int64_t goal_vertex_id{-1}; ///< Vertex ID of the reached goal (-1 if not found)
    size_t goal_index{0};       ///< Index in the goals vector (valid only if found)

    /// Implicit conversion to bool for convenient checking
    explicit operator bool() const noexcept { return found; }

    /// Check if path is empty
    bool empty() const noexcept { return path.empty(); }

    /// Get path length
    size_t size() const noexcept { return path.size(); }
};

/**
 * @brief Search termination limits for real-time systems
 *
 * Allows bounding search execution time or effort for predictable latency.
 * When a limit is reached, the search returns the best path found so far
 * (if any) or an empty result.
 */
struct SearchLimits {
    /// Maximum number of vertex expansions (0 = unlimited)
    size_t max_expansions;

    /// Maximum search duration in milliseconds (0 = unlimited)
    size_t timeout_ms;

    /// Default constructor - unlimited search
    SearchLimits() : max_expansions(0), timeout_ms(0) {}

    /// Constructor with explicit limits
    SearchLimits(size_t max_exp, size_t timeout)
        : max_expansions(max_exp), timeout_ms(timeout) {}

    /// Check if any limits are set
    bool HasLimits() const noexcept {
        return max_expansions > 0 || timeout_ms > 0;
    }

    /// Factory for unlimited search
    static SearchLimits Unlimited() noexcept {
        return SearchLimits();
    }

    /// Factory for expansion-limited search
    static SearchLimits MaxExpansions(size_t n) noexcept {
        return SearchLimits(n, 0);
    }

    /// Factory for time-limited search
    static SearchLimits Timeout(size_t ms) noexcept {
        return SearchLimits(0, ms);
    }
};

/// Forward declarations
template <typename State, typename Transition, typename StateIndexer>
class Graph;

/**
 * @brief Thread-local search context that externalizes search state from vertices
 * 
 * This class manages search algorithm state (costs, flags, parent pointers) 
 * separately from the graph structure, enabling thread-safe concurrent searches
 * on the same graph.
 * 
 * @tparam State The state type used in the graph
 * @tparam Transition The transition/cost type used in edges
 * @tparam StateIndexer The indexer functor for state types
 */
template <typename State, typename Transition, typename StateIndexer>
class SearchContext {
public:
  using GraphType = Graph<State, Transition, StateIndexer>;
  using vertex_iterator = typename GraphType::vertex_iterator;
  using const_vertex_iterator = typename GraphType::const_vertex_iterator;
  using VertexId = int64_t;

  /**
   * @brief Search information for a single vertex
   * 
   * Contains all the temporary data needed during search algorithms,
   * using flexible attributes for all algorithm-specific data.
   * This provides maximum flexibility and extensibility for future algorithms.
   */
  struct SearchVertexInfo {
    // Flexible attributes for all algorithm data
    std::unique_ptr<AttributeMap> attributes;

    // Default constructor
    SearchVertexInfo() = default;

    // Copy constructor - deep copy attributes if present
    SearchVertexInfo(const SearchVertexInfo& other) {
      if (other.attributes) {
        attributes.reset(new AttributeMap(*other.attributes));
      }
    }

    // Copy assignment - deep copy attributes if present
    SearchVertexInfo& operator=(const SearchVertexInfo& other) {
      if (this != &other) {
        if (other.attributes) {
          attributes.reset(new AttributeMap(*other.attributes));
        } else {
          attributes.reset();
        }
      }
      return *this;
    }

    // Move constructor
    SearchVertexInfo(SearchVertexInfo&&) = default;
    
    // Move assignment
    SearchVertexInfo& operator=(SearchVertexInfo&&) = default;

    /// Reset all search information to initial state
    void Reset() {
      // Clear attributes but keep the allocated AttributeMap for reuse
      if (attributes) {
        attributes->ClearAttributes();
      }
    }

    // === CONVENIENCE METHODS FOR COMMON ALGORITHM DATA ===

    // Boolean flags
    bool GetChecked() const {
      return GetAttributeOr<bool>("is_checked", false);
    }
    
    void SetChecked(bool checked) {
      SetAttribute("is_checked", checked);
    }
    
    bool GetInOpenList() const {
      return GetAttributeOr<bool>("is_in_openlist", false);
    }
    
    void SetInOpenList(bool in_list) {
      SetAttribute("is_in_openlist", in_list);
    }

    // Cost values (flexible types via attributes)
    template<typename T = double>
    T GetGCost() const {
      return GetAttributeOr<T>("g_cost", CostTraits<T>::infinity());
    }
    
    template<typename T>
    void SetGCost(const T& cost) {
      SetAttribute("g_cost", cost);
    }
    
    template<typename T = double>
    T GetHCost() const {
      return GetAttributeOr<T>("h_cost", CostTraits<T>::infinity());
    }
    
    template<typename T>
    void SetHCost(const T& cost) {
      SetAttribute("h_cost", cost);
    }
    
    template<typename T = double>
    T GetFCost() const {
      return GetAttributeOr<T>("f_cost", CostTraits<T>::infinity());
    }
    
    template<typename T>
    void SetFCost(const T& cost) {
      SetAttribute("f_cost", cost);
    }

    // Parent tracking
    VertexId GetParent() const {
      return GetAttributeOr<VertexId>("parent_id", -1);
    }
    
    void SetParent(VertexId parent) {
      SetAttribute("parent_id", parent);
    }

    // === LEGACY COMPATIBILITY PROPERTIES ===
    // These provide backward compatibility for existing code that accesses fields directly
    
    // Property-like accessors that can be used as lvalues for assignment
    struct BoolProperty {
      SearchVertexInfo* info;
      const char* key;
      operator bool() const { return info->GetAttributeOr<bool>(key, false); }
      BoolProperty& operator=(bool value) { info->SetAttribute(key, value); return *this; }
    };
    
    template<typename T = double>
    struct CostProperty {
      SearchVertexInfo* info;
      const char* key;
      operator T() const { return info->GetAttributeOr<T>(key, CostTraits<T>::infinity()); }
      CostProperty& operator=(const T& value) { info->SetAttribute(key, value); return *this; }
    };
    
    struct ParentProperty {
      SearchVertexInfo* info;
      const char* key;
      operator VertexId() const { return info->GetAttributeOr<VertexId>(key, -1); }
      ParentProperty& operator=(VertexId value) { info->SetAttribute(key, value); return *this; }
    };

    // Legacy field accessors that behave like the old direct field access
    BoolProperty is_checked{this, "is_checked"};
    BoolProperty is_in_openlist{this, "is_in_openlist"};
    ParentProperty parent_id{this, "parent_id"};
    
    // Cost properties for backward compatibility (default to double for legacy code)
    CostProperty<double> g_cost{this, "g_cost"};
    CostProperty<double> h_cost{this, "h_cost"};
    CostProperty<double> f_cost{this, "f_cost"};

    // Flexible attribute methods
    template<typename T>
    void SetAttribute(const std::string& key, const T& value) {
      if (!attributes) {
        attributes.reset(new AttributeMap());
      }
      attributes->SetAttribute(key, value);
    }

    template<typename T>
    const T& GetAttribute(const std::string& key) const {
      if (!attributes) {
        throw std::out_of_range("No attributes set on this vertex");
      }
      return attributes->GetAttribute<T>(key);
    }

    template<typename T>
    T GetAttributeOr(const std::string& key, const T& default_value) const {
      if (!attributes) {
        return default_value;
      }
      return attributes->GetAttributeOr(key, default_value);
    }

    bool HasAttribute(const std::string& key) const {
      return attributes && attributes->HasAttribute(key);
    }

    bool RemoveAttribute(const std::string& key) {
      return attributes && attributes->RemoveAttribute(key);
    }

    std::vector<std::string> GetAttributeKeys() const {
      if (!attributes) {
        return std::vector<std::string>();
      }
      return attributes->GetAttributeKeys();
    }
  };

private:
  /// Map from vertex ID to search information - optimized for reuse
  std::unordered_map<VertexId, SearchVertexInfo> search_data_;

  /// Context-level attributes for algorithm-specific state (e.g., DFS timestamp counter)
  /// This provides thread-safe storage for data that needs to be per-search, not per-vertex
  std::unique_ptr<AttributeMap> context_attributes_;

  /// Explicitly tracked start vertex for reliable path reconstruction
  /// Set by search algorithms at initialization, -1 means not set
  VertexId start_vertex_id_ = -1;

  /// Number of vertices expanded during the current/last search
  /// Useful for diagnostics and heuristic tuning
  size_t nodes_expanded_ = 0;

  /// Default reserve size for moderate-sized graphs
  static constexpr size_t DEFAULT_RESERVE_SIZE = 1000;

public:
  /**
   * @brief Default constructor with default memory optimization
   *
   * Pre-allocates space for 1000 vertices. For larger graphs in real-time
   * applications, use the constructor with explicit reserve_size.
   */
  SearchContext() {
    // Pre-allocate space to avoid frequent reallocations during search
    search_data_.reserve(DEFAULT_RESERVE_SIZE);
  }

  /**
   * @brief Constructor with configurable pre-allocation
   *
   * For real-time robotics applications, reserve >= expected graph size
   * to prevent memory reallocations during search operations.
   *
   * @param reserve_size Number of vertices to pre-allocate space for
   */
  explicit SearchContext(size_t reserve_size) {
    search_data_.reserve(reserve_size);
  }

  /**
   * @brief Reserve space for expected number of vertices
   *
   * Call this before search if the graph size is known and larger than
   * the default reserve size. Prevents reallocations during search.
   *
   * @param n Number of vertices to reserve space for
   */
  void Reserve(size_t n) {
    search_data_.reserve(n);
  }

  /**
   * @brief Get the current capacity of the search context
   * @return Number of vertices that can be stored without reallocation
   */
  size_t Capacity() const noexcept {
    return search_data_.bucket_count();
  }

  /**
   * @brief Get the number of nodes expanded in the current/last search
   *
   * Useful for:
   * - Comparing heuristic effectiveness (fewer expansions = better heuristic)
   * - Performance profiling and optimization
   * - Debugging search behavior
   *
   * @return Number of vertices that were expanded (removed from open list)
   */
  size_t GetNodesExpanded() const noexcept {
    return nodes_expanded_;
  }

  /**
   * @brief Increment the nodes expanded counter
   *
   * Called by search algorithms when a vertex is expanded (popped from open list).
   */
  void IncrementNodesExpanded() noexcept {
    ++nodes_expanded_;
  }

  /**
   * @brief Reset the nodes expanded counter to zero
   *
   * Called at the start of a new search.
   */
  void ResetNodesExpanded() noexcept {
    nodes_expanded_ = 0;
  }

  /**
   * @brief Get search information for a vertex
   * @param vertex_id The ID of the vertex
   * @return Reference to search information (creates if doesn't exist)
   */
  SearchVertexInfo& GetSearchInfo(VertexId vertex_id) {
    return search_data_[vertex_id];
  }

  /**
   * @brief Get search information for a vertex (const version)
   * @param vertex_id The ID of the vertex
   * @return Const reference to search information
   * @throws ElementNotFoundError if vertex not found
   */
  const SearchVertexInfo& GetSearchInfo(VertexId vertex_id) const {
    auto it = search_data_.find(vertex_id);
    if (it == search_data_.end()) {
      throw ElementNotFoundError("Vertex", vertex_id);
    }
    return it->second;
  }

  /**
   * @brief Check if vertex has search information
   * @param vertex_id The ID of the vertex
   * @return True if vertex has search info, false otherwise
   */
  bool HasSearchInfo(VertexId vertex_id) const {
    return search_data_.find(vertex_id) != search_data_.end();
  }

  /**
   * @brief Get search information for a vertex iterator
   * @param vertex_it Iterator to the vertex
   * @return Reference to search information
   */
  SearchVertexInfo& GetSearchInfo(vertex_iterator vertex_it) {
    return GetSearchInfo(vertex_it->vertex_id);
  }

  /**
   * @brief Get search information for a const vertex iterator
   * @param vertex_it Const iterator to the vertex
   * @return Reference to search information
   */
  SearchVertexInfo& GetSearchInfo(const_vertex_iterator vertex_it) {
    return GetSearchInfo(vertex_it->vertex_id);
  }

  /**
   * @brief Get search information for a vertex iterator (const version)  
   * @param vertex_it Iterator to the vertex
   * @return Const reference to search information
   */
  const SearchVertexInfo& GetSearchInfo(vertex_iterator vertex_it) const {
    return GetSearchInfo(vertex_it->vertex_id);
  }

  /**
   * @brief Get search information for a const vertex iterator (const version)  
   * @param vertex_it Const iterator to the vertex
   * @return Const reference to search information
   */
  const SearchVertexInfo& GetSearchInfo(const_vertex_iterator vertex_it) const {
    return GetSearchInfo(vertex_it->vertex_id);
  }

  /**
   * @brief Clear all search information
   */
  void Clear() {
    search_data_.clear();
  }

  /**
   * @brief Reset all search information to initial state
   *
   * This keeps the allocated memory but resets values,
   * which can be more efficient for repeated searches.
   * This is the key optimization for 36% improvement shown in benchmarks.
   * For complete clearing that removes all entries, use Clear() instead.
   */
  void Reset() {
    for (auto& pair : search_data_) {
      pair.second.Reset();
    }
    // Clear context-level attributes (e.g., DFS timestamp counter)
    if (context_attributes_) {
      context_attributes_->ClearAttributes();
    }
    // Reset start vertex tracking
    start_vertex_id_ = -1;
    // Reset nodes expanded counter
    nodes_expanded_ = 0;
    // Keep allocated memory in the map for next search
    // This avoids reallocating hash table buckets
  }

  /**
   * @brief Get the number of vertices with search information
   * @return Number of vertices in the search context
   */
  size_t Size() const {
    return search_data_.size();
  }

  /**
   * @brief Check if the context is empty
   * @return True if no vertices have search information
   */
  bool Empty() const {
    return search_data_.empty();
  }

  // =========================================================================
  // START VERTEX TRACKING (for reliable path reconstruction)
  // =========================================================================

  /**
   * @brief Set the start vertex ID for this search
   *
   * Called by search algorithms during initialization to enable reliable
   * path reconstruction. This avoids the need to iterate through all vertices
   * to find which one has parent_id == -1.
   *
   * @param vertex_id The ID of the start vertex
   */
  void SetStartVertexId(VertexId vertex_id) {
    start_vertex_id_ = vertex_id;
  }

  /**
   * @brief Get the start vertex ID
   * @return The start vertex ID, or -1 if not set
   */
  VertexId GetStartVertexId() const noexcept {
    return start_vertex_id_;
  }

  /**
   * @brief Check if start vertex has been set
   * @return True if start vertex ID is valid (not -1)
   */
  bool HasStartVertex() const noexcept {
    return start_vertex_id_ != -1;
  }

  // =========================================================================
  // CONTEXT-LEVEL ATTRIBUTES (for algorithm state like DFS counters)
  // =========================================================================

  /**
   * @brief Set a context-level attribute
   *
   * Context-level attributes store algorithm state that is per-search rather
   * than per-vertex. For example, DFS uses this for its timestamp counter.
   * This ensures thread-safety when multiple searches share a strategy instance.
   *
   * @tparam T Type of the attribute value
   * @param key Attribute name
   * @param value Attribute value
   */
  template<typename T>
  void SetContextAttribute(const std::string& key, const T& value) {
    if (!context_attributes_) {
      context_attributes_.reset(new AttributeMap());
    }
    context_attributes_->SetAttribute(key, value);
  }

  /**
   * @brief Get a context-level attribute
   * @tparam T Expected type of the attribute
   * @param key Attribute name
   * @return Reference to the attribute value
   * @throws std::out_of_range if attribute doesn't exist
   */
  template<typename T>
  const T& GetContextAttribute(const std::string& key) const {
    if (!context_attributes_) {
      throw std::out_of_range("No context attributes set");
    }
    return context_attributes_->GetAttribute<T>(key);
  }

  /**
   * @brief Get a context-level attribute with default value
   * @tparam T Expected type of the attribute
   * @param key Attribute name
   * @param default_value Default value if attribute doesn't exist
   * @return Attribute value or default
   */
  template<typename T>
  T GetContextAttributeOr(const std::string& key, const T& default_value) const {
    if (!context_attributes_) {
      return default_value;
    }
    return context_attributes_->GetAttributeOr(key, default_value);
  }

  /**
   * @brief Check if a context-level attribute exists
   * @param key Attribute name
   * @return true if attribute exists
   */
  bool HasContextAttribute(const std::string& key) const {
    return context_attributes_ && context_attributes_->HasAttribute(key);
  }

  /**
   * @brief Increment a numeric context attribute and return the new value
   *
   * This is a convenience method primarily for DFS timestamp counters.
   * If the attribute doesn't exist, initializes it to 1.
   *
   * @param key Attribute name
   * @return The incremented value
   */
  int64_t IncrementContextCounter(const std::string& key) {
    int64_t current = GetContextAttributeOr<int64_t>(key, 0);
    int64_t next = current + 1;
    SetContextAttribute(key, next);
    return next;
  }

  // =========================================================================
  // FLEXIBLE ATTRIBUTE INTERFACE (for new algorithms)
  // =========================================================================

  /**
   * @brief Set a custom attribute for a vertex in the search context
   * @tparam T Type of the attribute value
   * @param vertex_id Vertex identifier
   * @param key Attribute name
   * @param value Attribute value
   */
  template<typename T>
  void SetVertexAttribute(VertexId vertex_id, const std::string& key, const T& value) {
    auto& info = GetSearchInfo(vertex_id);
    info.SetAttribute(key, value);
  }

  /**
   * @brief Get a custom attribute for a vertex in the search context
   * @tparam T Expected type of the attribute
   * @param vertex_id Vertex identifier
   * @param key Attribute name
   * @return Reference to the attribute value
   */
  template<typename T>
  const T& GetVertexAttribute(VertexId vertex_id, const std::string& key) const {
    const auto& info = GetSearchInfo(vertex_id);
    return info.template GetAttribute<T>(key);
  }

  /**
   * @brief Get a custom attribute with default value
   * @tparam T Expected type of the attribute
   * @param vertex_id Vertex identifier
   * @param key Attribute name
   * @param default_value Default value if attribute doesn't exist
   * @return Attribute value or default
   */
  template<typename T>
  T GetVertexAttributeOr(VertexId vertex_id, const std::string& key, const T& default_value) const {
    if (!HasSearchInfo(vertex_id)) {
      return default_value;
    }
    const auto& info = GetSearchInfo(vertex_id);
    return info.template GetAttributeOr<T>(key, default_value);
  }

  /**
   * @brief Check if a vertex has a custom attribute
   * @param vertex_id Vertex identifier
   * @param key Attribute name
   * @return true if attribute exists
   */
  bool HasVertexAttribute(VertexId vertex_id, const std::string& key) const {
    if (!HasSearchInfo(vertex_id)) {
      return false;
    }
    const auto& info = GetSearchInfo(vertex_id);
    return info.HasAttribute(key);
  }

  /**
   * @brief Get all custom attribute keys for a vertex
   * @param vertex_id Vertex identifier
   * @return Vector of attribute keys
   */
  std::vector<std::string> GetVertexAttributeKeys(VertexId vertex_id) const {
    if (!HasSearchInfo(vertex_id)) {
      return std::vector<std::string>();
    }
    const auto& info = GetSearchInfo(vertex_id);
    return info.GetAttributeKeys();
  }

  // =========================================================================
  // CONVENIENCE METHODS (bridge legacy and flexible approaches)
  // =========================================================================

  /**
   * @brief Set g-cost with flexible type support
   * @param vertex_id Vertex identifier
   * @param cost The cost value
   */
  template<typename T>
  void SetGCost(VertexId vertex_id, const T& cost) {
    GetSearchInfo(vertex_id).SetGCost(cost);
  }

  /**
   * @brief Get g-cost with flexible type support
   * @param vertex_id Vertex identifier
   */
  template<typename T = double>
  T GetGCost(VertexId vertex_id) const {
    if (HasSearchInfo(vertex_id)) {
      return GetSearchInfo(vertex_id).template GetGCost<T>();
    }
    return std::numeric_limits<T>::max();
  }

  /**
   * @brief Set parent using either legacy field or flexible attribute
   */
  void SetParent(VertexId vertex_id, VertexId parent_id, bool use_legacy = true) {
    if (use_legacy) {
      GetSearchInfo(vertex_id).parent_id = parent_id;
    } else {
      SetVertexAttribute(vertex_id, "parent", parent_id);
    }
  }

  /**
   * @brief Get parent from either legacy field or flexible attribute
   */
  VertexId GetParent(VertexId vertex_id, bool use_legacy = true) const {
    if (use_legacy) {
      return HasSearchInfo(vertex_id) ? GetSearchInfo(vertex_id).parent_id : -1;
    } else {
      return GetVertexAttributeOr<VertexId>(vertex_id, "parent", -1);
    }
  }

  /**
   * @brief Reconstruct path from search results
   * @param graph Pointer to the graph
   * @param goal_id ID of the goal vertex
   * @return Vector of states representing the path from start to goal
   */
  template<typename PathState = State>
  std::vector<PathState> ReconstructPath(const GraphType* graph, VertexId goal_id) const {
    std::vector<PathState> path;

    if (!HasSearchInfo(goal_id)) {
      throw ElementNotFoundError("Goal vertex", goal_id);
    }

    // Check if goal was reached
    const auto& goal_info = GetSearchInfo(goal_id);
    if (goal_info.parent_id == -1) {
      // Goal has no parent - check if it's the start vertex (single node path)
      // Use explicit start vertex tracking if available (more reliable)
      if (HasStartVertex()) {
        if (goal_id != start_vertex_id_) {
          return path; // No path found - goal is unreachable
        }
        // goal_id == start_vertex_id_ means single-vertex path
      } else {
        // Fallback: check if there's another vertex with parent_id == -1
        // This is the legacy behavior for backward compatibility
        for (const auto& pair : search_data_) {
          if (pair.second.parent_id == -1 && pair.first != goal_id) {
            return path; // No path found - there's a different start vertex
          }
        }
      }
    }

    // Build path backwards from goal to start
    std::vector<VertexId> vertex_path;
    VertexId current_id = goal_id;

    // Use explicit start vertex for termination if available
    VertexId termination_id = HasStartVertex() ? start_vertex_id_ : -1;

    while (current_id != -1) {
      vertex_path.push_back(current_id);

      // Stop if we've reached the start vertex (explicit termination)
      if (HasStartVertex() && current_id == termination_id) {
        break;
      }

      if (!HasSearchInfo(current_id)) {
        break; // Safety check
      }
      const auto& info = GetSearchInfo(current_id);
      current_id = info.parent_id;
    }

    // Convert vertex IDs to states and reverse
    path.reserve(vertex_path.size());
    for (auto it = vertex_path.rbegin(); it != vertex_path.rend(); ++it) {
      // Find vertex by ID in the graph using FindVertex method
      auto vertex_it = graph->FindVertex(*it);
      if (vertex_it != graph->vertex_end()) {
        path.push_back(vertex_it->state);
      } else {
        // If vertex not found, path reconstruction failed
        return std::vector<PathState>(); // Return empty path on failure
      }
    }

    return path;
  }
};

} // namespace xmotion

#endif /* SEARCH_CONTEXT_HPP */