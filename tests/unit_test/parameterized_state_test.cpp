/*
 * parameterized_state_test.cpp
 *
 * Created on: 2025
 * Description: Parameterized tests for different state types (value, pointer, shared_ptr)
 *              Uses Google Test typed test framework for comprehensive coverage
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <memory>
#include <vector>
#include <functional>
#include <type_traits>

#include "gtest/gtest.h"

#include "graph/graph.hpp"
#include "graph/tree.hpp"
#include "graph/search/astar.hpp"
#include "graph/search/dijkstra.hpp"

using namespace xmotion;

// Test state class for parameterized tests
struct ParameterizedTestState {
  ParameterizedTestState(int64_t id) : id_(id) {}
  
  int64_t id_;
  
  int64_t GetUniqueID() const { return id_; }
  
  bool operator==(const ParameterizedTestState& other) const {
    return id_ == other.id_;
  }
};

// Custom indexer for shared_ptr<ParameterizedTestState>
struct SharedPtrStateIndexer {
  int64_t operator()(std::shared_ptr<ParameterizedTestState> state) const {
    return state->id_;
  }
};

// Custom indexer for ParameterizedTestState*
struct PtrStateIndexer {
  int64_t operator()(ParameterizedTestState* state) const {
    return state->id_;
  }
};

// State type traits and utilities
template<typename StateType>
struct StateTraits;

// Specialization for value types
template<>
struct StateTraits<ParameterizedTestState> {
  using StateType = ParameterizedTestState;
  using GraphType = Graph<StateType>;
  using TreeType = Tree<StateType>;
  using IndexerType = DefaultIndexer<StateType>;
  
  static StateType CreateState(int64_t id) {
    return ParameterizedTestState(id);
  }
  
  static int64_t GetId(const StateType& state) {
    return state.id_;
  }
  
  static void CleanupStates(const std::vector<StateType>&) {
    // No cleanup needed for value types
  }
  
  static const char* GetTypeName() { return "ValueType"; }
};

// Specialization for pointer types  
template<>
struct StateTraits<ParameterizedTestState*> {
  using StateType = ParameterizedTestState*;
  using GraphType = Graph<StateType, double, PtrStateIndexer>;
  using TreeType = Tree<StateType, double, PtrStateIndexer>;
  using IndexerType = PtrStateIndexer;
  
  static StateType CreateState(int64_t id) {
    return new ParameterizedTestState(id);
  }
  
  static int64_t GetId(const StateType& state) {
    return state->id_;
  }
  
  static void CleanupStates(const std::vector<StateType>& states) {
    for (auto state : states) {
      delete state;
    }
  }
  
  static const char* GetTypeName() { return "PointerType"; }
};

// Specialization for shared_ptr types
template<>
struct StateTraits<std::shared_ptr<ParameterizedTestState>> {
  using StateType = std::shared_ptr<ParameterizedTestState>;
  using GraphType = Graph<StateType, double, SharedPtrStateIndexer>;
  using TreeType = Tree<StateType, double, SharedPtrStateIndexer>;
  using IndexerType = SharedPtrStateIndexer;
  
  static StateType CreateState(int64_t id) {
    return std::make_shared<ParameterizedTestState>(id);
  }
  
  static int64_t GetId(const StateType& state) {
    return state->id_;
  }
  
  static void CleanupStates(const std::vector<StateType>&) {
    // No explicit cleanup needed for shared_ptr
  }
  
  static const char* GetTypeName() { return "SharedPtrType"; }
};

// Test fixture template for parameterized state tests
template<typename StateType>
class ParameterizedStateTest : public testing::Test {
public:
  using Traits = StateTraits<StateType>;
  using GraphType = typename Traits::GraphType;
  using TreeType = typename Traits::TreeType;
  
protected:
  void SetUp() override {
    // Create test states
    for (int i = 0; i < 10; ++i) {
      states.push_back(Traits::CreateState(i));
    }
  }
  
  void TearDown() override {
    Traits::CleanupStates(states);
  }
  
  // Helper to get state ID regardless of type
  int64_t GetStateId(const StateType& state) {
    return Traits::GetId(state);
  }
  
  // Helper to create a simple linear graph: 0 -> 1 -> 2 -> 3 -> 4
  void CreateLinearGraph(GraphType& graph) {
    for (int i = 0; i < 4; ++i) {
      graph.AddEdge(states[i], states[i + 1], 1.0);
    }
  }
  
  // Helper to create a more complex graph for testing
  void CreateComplexGraph(GraphType& graph) {
    // Create a diamond-shaped graph
    // 0 -> 1, 2
    // 1 -> 3
    // 2 -> 3  
    // 3 -> 4
    graph.AddEdge(states[0], states[1], 1.0);
    graph.AddEdge(states[0], states[2], 2.0);
    graph.AddEdge(states[1], states[3], 1.0);
    graph.AddEdge(states[2], states[3], 1.0);
    graph.AddEdge(states[3], states[4], 1.0);
  }
  
  // Helper to create a tree structure
  void CreateTree(TreeType& tree) {
    // Create tree: 0 as root, 1,2 as children of 0, 3,4 as children of 1
    tree.AddEdge(states[0], states[1], 1.0);
    tree.AddEdge(states[0], states[2], 1.0);
    tree.AddEdge(states[1], states[3], 1.0);
    tree.AddEdge(states[1], states[4], 1.0);
  }
  
  std::vector<StateType> states;
};

// Define the types we want to test
using StateTypes = ::testing::Types<
  ParameterizedTestState,                           // Value type
  ParameterizedTestState*,                          // Pointer type
  std::shared_ptr<ParameterizedTestState>           // Shared pointer type
>;

TYPED_TEST_SUITE(ParameterizedStateTest, StateTypes);

// ===== BASIC GRAPH OPERATIONS TESTS =====

TYPED_TEST(ParameterizedStateTest, BasicGraphConstruction) {
  using GraphType = typename TestFixture::GraphType;
  GraphType graph;
  
  // Test basic vertex addition
  EXPECT_EQ(graph.GetTotalVertexNumber(), 0);
  EXPECT_EQ(graph.GetTotalEdgeNumber(), 0);
  
  // Add vertices through edge addition
  this->CreateLinearGraph(graph);
  
  EXPECT_EQ(graph.GetTotalVertexNumber(), 5) << "Linear graph should have 5 vertices";
  EXPECT_EQ(graph.GetTotalEdgeNumber(), 4) << "Linear graph should have 4 edges";
}

TYPED_TEST(ParameterizedStateTest, VertexOperations) {
  using GraphType = typename TestFixture::GraphType;
  GraphType graph;
  
  // Add vertices
  for (int i = 0; i < 5; ++i) {
    auto vertex_it = graph.AddVertex(this->states[i]);
    EXPECT_NE(vertex_it, graph.vertex_end()) 
      << "Failed to add vertex " << i << " for " << TestFixture::Traits::GetTypeName();
  }
  
  EXPECT_EQ(graph.GetTotalVertexNumber(), 5);
  
  // Find vertices
  for (int i = 0; i < 5; ++i) {
    auto vertex_it = graph.FindVertex(this->states[i]);
    EXPECT_NE(vertex_it, graph.vertex_end()) 
      << "Failed to find vertex " << i << " for " << TestFixture::Traits::GetTypeName();
    EXPECT_EQ(vertex_it->vertex_id, i) 
      << "Vertex ID mismatch for " << TestFixture::Traits::GetTypeName();
  }
  
  // Remove vertices
  graph.RemoveVertex(this->states[2]);
  EXPECT_EQ(graph.GetTotalVertexNumber(), 4);
  
  auto removed_vertex = graph.FindVertex(this->states[2]);
  EXPECT_EQ(removed_vertex, graph.vertex_end()) 
    << "Removed vertex should not be findable for " << TestFixture::Traits::GetTypeName();
}

TYPED_TEST(ParameterizedStateTest, EdgeOperations) {
  using GraphType = typename TestFixture::GraphType;
  GraphType graph;
  
  // Add edges
  graph.AddEdge(this->states[0], this->states[1], 1.5);
  graph.AddEdge(this->states[1], this->states[2], 2.5);
  graph.AddEdge(this->states[0], this->states[2], 3.5);
  
  EXPECT_EQ(graph.GetTotalVertexNumber(), 3);
  EXPECT_EQ(graph.GetTotalEdgeNumber(), 3);
  
  // Check edge costs
  auto vertex0 = graph.FindVertex(this->states[0]);
  ASSERT_NE(vertex0, graph.vertex_end());
  
  auto edge_to_1 = vertex0->FindEdge(this->states[1]);
  auto edge_to_2 = vertex0->FindEdge(this->states[2]);
  
  ASSERT_NE(edge_to_1, vertex0->edge_end());
  ASSERT_NE(edge_to_2, vertex0->edge_end());
  
  EXPECT_DOUBLE_EQ(edge_to_1->cost, 1.5);
  EXPECT_DOUBLE_EQ(edge_to_2->cost, 3.5);
  
  // Remove edge
  bool removed = graph.RemoveEdge(this->states[0], this->states[1]);
  EXPECT_TRUE(removed) << "Edge removal should succeed for " << TestFixture::Traits::GetTypeName();
  EXPECT_EQ(graph.GetTotalEdgeNumber(), 2);
  
  // Verify edge is removed
  edge_to_1 = vertex0->FindEdge(this->states[1]);
  EXPECT_EQ(edge_to_1, vertex0->edge_end()) 
    << "Removed edge should not be findable for " << TestFixture::Traits::GetTypeName();
}

TYPED_TEST(ParameterizedStateTest, GraphIterators) {
  using GraphType = typename TestFixture::GraphType;
  GraphType graph;
  
  this->CreateLinearGraph(graph);
  
  // Test vertex iteration
  std::set<int64_t> found_ids;
  for (auto it = graph.vertex_begin(); it != graph.vertex_end(); ++it) {
    found_ids.insert(it->vertex_id);
  }
  
  std::set<int64_t> expected_ids = {0, 1, 2, 3, 4};
  EXPECT_EQ(found_ids, expected_ids) 
    << "Vertex iteration failed for " << TestFixture::Traits::GetTypeName();
  
  // Test edge iteration through vertices
  auto vertex0 = graph.FindVertex(this->states[0]);
  ASSERT_NE(vertex0, graph.vertex_end());
  
  int edge_count = 0;
  for (auto edge_it = vertex0->edge_begin(); edge_it != vertex0->edge_end(); ++edge_it) {
    edge_count++;
    EXPECT_DOUBLE_EQ(edge_it->cost, 1.0);
  }
  EXPECT_EQ(edge_count, 1) << "Vertex 0 should have 1 outgoing edge";
}

// ===== COPY AND MOVE SEMANTICS TESTS =====

TYPED_TEST(ParameterizedStateTest, CopyConstructor) {
  using GraphType = typename TestFixture::GraphType;
  GraphType original_graph;
  
  this->CreateComplexGraph(original_graph);
  
  // Copy construct
  GraphType copied_graph(original_graph);
  
  EXPECT_EQ(copied_graph.GetTotalVertexNumber(), original_graph.GetTotalVertexNumber())
    << "Copy constructor failed for vertex count - " << TestFixture::Traits::GetTypeName();
  EXPECT_EQ(copied_graph.GetTotalEdgeNumber(), original_graph.GetTotalEdgeNumber())
    << "Copy constructor failed for edge count - " << TestFixture::Traits::GetTypeName();
  
  // Verify vertices are findable in copy
  for (int i = 0; i < 5; ++i) {
    auto vertex_it = copied_graph.FindVertex(this->states[i]);
    EXPECT_NE(vertex_it, copied_graph.vertex_end()) 
      << "Vertex " << i << " not found in copied graph - " << TestFixture::Traits::GetTypeName();
  }
}

TYPED_TEST(ParameterizedStateTest, AssignmentOperator) {
  using GraphType = typename TestFixture::GraphType;
  GraphType source_graph, target_graph;
  
  this->CreateComplexGraph(source_graph);
  
  // Create different graph in target
  target_graph.AddEdge(this->states[5], this->states[6], 10.0);
  EXPECT_EQ(target_graph.GetTotalVertexNumber(), 2);
  
  // Assign
  target_graph = source_graph;
  
  EXPECT_EQ(target_graph.GetTotalVertexNumber(), source_graph.GetTotalVertexNumber())
    << "Assignment operator failed for vertex count - " << TestFixture::Traits::GetTypeName();
  EXPECT_EQ(target_graph.GetTotalEdgeNumber(), source_graph.GetTotalEdgeNumber())
    << "Assignment operator failed for edge count - " << TestFixture::Traits::GetTypeName();
}

TYPED_TEST(ParameterizedStateTest, MoveConstructor) {
  using GraphType = typename TestFixture::GraphType;
  GraphType original_graph;
  
  this->CreateComplexGraph(original_graph);
  auto original_vertex_count = original_graph.GetTotalVertexNumber();
  auto original_edge_count = original_graph.GetTotalEdgeNumber();
  
  // Move construct
  GraphType moved_graph(std::move(original_graph));
  
  EXPECT_EQ(moved_graph.GetTotalVertexNumber(), original_vertex_count)
    << "Move constructor failed for vertex count - " << TestFixture::Traits::GetTypeName();
  EXPECT_EQ(moved_graph.GetTotalEdgeNumber(), original_edge_count)
    << "Move constructor failed for edge count - " << TestFixture::Traits::GetTypeName();
  
  // Original should be empty after move
  EXPECT_EQ(original_graph.GetTotalVertexNumber(), 0)
    << "Original graph should be empty after move - " << TestFixture::Traits::GetTypeName();
}

// ===== SEARCH ALGORITHM TESTS =====

TYPED_TEST(ParameterizedStateTest, DijkstraSearch) {
  using GraphType = typename TestFixture::GraphType;
  GraphType graph;
  
  this->CreateLinearGraph(graph);
  
  // Search from 0 to 4
  auto path = Dijkstra::Search(&graph, this->states[0], this->states[4]);
  
  EXPECT_EQ(path.size(), 5) << "Dijkstra path should have 5 nodes - " << TestFixture::Traits::GetTypeName();
  
  if (!path.empty()) {
    EXPECT_EQ(this->GetStateId(path.front()), 0) 
      << "Path should start with state 0 - " << TestFixture::Traits::GetTypeName();
    EXPECT_EQ(this->GetStateId(path.back()), 4) 
      << "Path should end with state 4 - " << TestFixture::Traits::GetTypeName();
  }
}

TYPED_TEST(ParameterizedStateTest, AStarSearch) {
  using GraphType = typename TestFixture::GraphType;
  using StateType = typename TestFixture::Traits::StateType;
  GraphType graph;
  
  this->CreateComplexGraph(graph);
  
  // Simple heuristic function
  std::function<double(StateType, StateType)> heuristic = 
    [this](const StateType& s1, const StateType& s2) {
      return std::abs(this->GetStateId(s1) - this->GetStateId(s2));
    };
  
  auto path = AStar::Search(&graph, this->states[0], this->states[4], heuristic);
  
  EXPECT_FALSE(path.empty()) << "A* should find a path - " << TestFixture::Traits::GetTypeName();
  
  if (!path.empty()) {
    EXPECT_EQ(this->GetStateId(path.front()), 0) 
      << "A* path should start with state 0 - " << TestFixture::Traits::GetTypeName();
    EXPECT_EQ(this->GetStateId(path.back()), 4) 
      << "A* path should end with state 4 - " << TestFixture::Traits::GetTypeName();
  }
}

TYPED_TEST(ParameterizedStateTest, SearchNoPath) {
  using GraphType = typename TestFixture::GraphType;
  using StateType = typename TestFixture::Traits::StateType;
  GraphType graph;
  
  // Create disconnected graph
  graph.AddEdge(this->states[0], this->states[1], 1.0);
  graph.AddEdge(this->states[2], this->states[3], 1.0);
  
  // Try to find path between disconnected components
  auto dijkstra_path = Dijkstra::Search(&graph, this->states[0], this->states[3]);
  EXPECT_TRUE(dijkstra_path.empty()) 
    << "Dijkstra should not find path in disconnected graph - " << TestFixture::Traits::GetTypeName();
  
  std::function<double(StateType, StateType)> heuristic = 
    [this](const StateType& s1, const StateType& s2) {
      return std::abs(this->GetStateId(s1) - this->GetStateId(s2));
    };
  
  auto astar_path = AStar::Search(&graph, this->states[0], this->states[3], heuristic);
  EXPECT_TRUE(astar_path.empty()) 
    << "A* should not find path in disconnected graph - " << TestFixture::Traits::GetTypeName();
}

// ===== TREE TESTS =====

TYPED_TEST(ParameterizedStateTest, TreeBasicOperations) {
  using TreeType = typename TestFixture::TreeType;
  TreeType tree;
  
  this->CreateTree(tree);
  
  EXPECT_EQ(tree.GetTotalVertexNumber(), 5) << "Tree should have 5 vertices - " << TestFixture::Traits::GetTypeName();
  EXPECT_EQ(tree.GetTotalEdgeNumber(), 4) << "Tree should have 4 edges - " << TestFixture::Traits::GetTypeName();
  
  // Test tree structure
  auto root = tree.FindVertex(this->states[0]);
  ASSERT_NE(root, tree.vertex_end());
  
  // Root should have 2 children
  int child_count = 0;
  for (auto edge_it = root->edge_begin(); edge_it != root->edge_end(); ++edge_it) {
    child_count++;
  }
  EXPECT_EQ(child_count, 2) << "Root should have 2 children - " << TestFixture::Traits::GetTypeName();
}

TYPED_TEST(ParameterizedStateTest, TreeSubtreeRemoval) {
  using TreeType = typename TestFixture::TreeType;
  TreeType tree;
  
  this->CreateTree(tree);
  
  // Remove subtree rooted at state[1] (should remove states 1, 3, 4)
  tree.RemoveSubtree(this->states[1]);
  
  EXPECT_EQ(tree.GetTotalVertexNumber(), 2) 
    << "After subtree removal should have 2 vertices - " << TestFixture::Traits::GetTypeName();
  
  // Verify removed vertices are not findable
  EXPECT_EQ(tree.FindVertex(this->states[1]), tree.vertex_end());
  EXPECT_EQ(tree.FindVertex(this->states[3]), tree.vertex_end());
  EXPECT_EQ(tree.FindVertex(this->states[4]), tree.vertex_end());
  
  // Verify remaining vertices are findable
  EXPECT_NE(tree.FindVertex(this->states[0]), tree.vertex_end());
  EXPECT_NE(tree.FindVertex(this->states[2]), tree.vertex_end());
}

// ===== STRESS TESTS =====

TYPED_TEST(ParameterizedStateTest, LargeGraphOperations) {
  using GraphType = typename TestFixture::GraphType;
  using Traits = typename TestFixture::Traits;
  GraphType graph;
  
  const int LARGE_SIZE = 100;
  std::vector<typename Traits::StateType> large_states;
  
  // Create many states
  for (int i = 0; i < LARGE_SIZE; ++i) {
    large_states.push_back(Traits::CreateState(i));
  }
  
  // Add many edges (create a connected graph)
  for (int i = 0; i < LARGE_SIZE - 1; ++i) {
    graph.AddEdge(large_states[i], large_states[i + 1], 1.0);
  }
  
  // Add some cross connections
  for (int i = 0; i < LARGE_SIZE - 10; ++i) {
    graph.AddEdge(large_states[i], large_states[i + 10], 2.0);
  }
  
  EXPECT_EQ(graph.GetTotalVertexNumber(), LARGE_SIZE) 
    << "Large graph vertex count incorrect - " << TestFixture::Traits::GetTypeName();
  EXPECT_GT(graph.GetTotalEdgeNumber(), LARGE_SIZE - 1) 
    << "Large graph edge count incorrect - " << TestFixture::Traits::GetTypeName();
  
  // Test search on large graph
  auto path = Dijkstra::Search(&graph, large_states[0], large_states[LARGE_SIZE - 1]);
  EXPECT_FALSE(path.empty()) 
    << "Should find path in large connected graph - " << TestFixture::Traits::GetTypeName();
  
  // Cleanup
  Traits::CleanupStates(large_states);
}

// ===== STATE TYPE SPECIFIC TESTS =====

TYPED_TEST(ParameterizedStateTest, StateTypeSpecificBehavior) {
  using GraphType = typename TestFixture::GraphType;
  using StateType = typename TestFixture::Traits::StateType;
  GraphType graph;
  
  // Test that works for all state types
  auto vertex_it = graph.AddVertex(this->states[0]);
  EXPECT_NE(vertex_it, graph.vertex_end());
  
  // Add type-specific test information
  std::string type_name = TestFixture::Traits::GetTypeName();
  EXPECT_FALSE(type_name.empty()) << "Type name should not be empty";
  
  // This test documents the behavior for each type
  if (type_name == "ValueType") {
    // For value types, states are copied
    EXPECT_EQ(this->GetStateId(vertex_it->state), 0);
  } else if (type_name == "PointerType") {
    // For pointer types, pointer values are stored
    EXPECT_EQ(this->GetStateId(vertex_it->state), 0);
  } else if (type_name == "SharedPtrType") {
    // For shared_ptr types, shared ownership
    EXPECT_EQ(this->GetStateId(vertex_it->state), 0);
  }
}