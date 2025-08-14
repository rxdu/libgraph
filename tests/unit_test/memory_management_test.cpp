/*
 * memory_management_test.cpp
 *
 * Created on: 2025
 * Description: Tests for memory management, leak detection, and exception safety
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <memory>
#include <vector>
#include <exception>

#include "gtest/gtest.h"

#include "graph/graph.hpp"
#include "graph/tree.hpp"

using namespace xmotion;

// Custom state class to track construction/destruction
class MemoryTrackingState {
public:
  static int construction_count;
  static int destruction_count;
  static int copy_count;
  static int move_count;
  
  static void ResetCounters() {
    construction_count = 0;
    destruction_count = 0;
    copy_count = 0;
    move_count = 0;
  }

  MemoryTrackingState(int64_t id) : id_(id) {
    construction_count++;
  }
  
  ~MemoryTrackingState() {
    destruction_count++;
  }
  
  MemoryTrackingState(const MemoryTrackingState& other) : id_(other.id_) {
    copy_count++;
    construction_count++;
  }
  
  MemoryTrackingState(MemoryTrackingState&& other) noexcept : id_(other.id_) {
    move_count++;
    construction_count++;
  }
  
  MemoryTrackingState& operator=(const MemoryTrackingState& other) {
    if (this != &other) {
      id_ = other.id_;
      copy_count++;
    }
    return *this;
  }
  
  MemoryTrackingState& operator=(MemoryTrackingState&& other) noexcept {
    if (this != &other) {
      id_ = other.id_;
      move_count++;
    }
    return *this;
  }
  
  int64_t id_;
};

int MemoryTrackingState::construction_count = 0;
int MemoryTrackingState::destruction_count = 0;
int MemoryTrackingState::copy_count = 0;
int MemoryTrackingState::move_count = 0;

// Test fixture for memory management tests
class MemoryManagementTest : public testing::Test {
protected:
  void SetUp() override {
    MemoryTrackingState::ResetCounters();
  }
  
  void TearDown() override {
    // Verify no memory leaks in each test
    EXPECT_EQ(MemoryTrackingState::construction_count, 
              MemoryTrackingState::destruction_count) 
      << "Memory leak detected: " 
      << MemoryTrackingState::construction_count << " constructions vs "
      << MemoryTrackingState::destruction_count << " destructions";
  }
};

// ===== BASIC MEMORY MANAGEMENT TESTS =====

TEST_F(MemoryManagementTest, GraphDestructorCleansUpMemory) {
  {
    Graph<MemoryTrackingState> graph;
    
    // Add vertices
    for (int i = 0; i < 10; ++i) {
      graph.AddVertex(MemoryTrackingState(i));
    }
    
    // Add edges to create a more complex structure
    for (int i = 0; i < 9; ++i) {
      graph.AddEdge(MemoryTrackingState(i), MemoryTrackingState(i + 1), 1.0);
    }
    
    // Graph goes out of scope here, should clean up all memory
  }
  
  // TearDown will verify all objects were destroyed
}

TEST_F(MemoryManagementTest, ClearAllCleansUpMemory) {
  Graph<MemoryTrackingState> graph;
  
  // Add vertices
  for (int i = 0; i < 5; ++i) {
    graph.AddVertex(MemoryTrackingState(i));
  }
  
  // Add edges
  for (int i = 0; i < 4; ++i) {
    graph.AddEdge(MemoryTrackingState(i), MemoryTrackingState(i + 1), 1.0);
  }
  
  int initial_count = MemoryTrackingState::construction_count;
  
  // Clear all vertices
  graph.ClearAll();
  
  // Verify some objects were destroyed
  EXPECT_GT(MemoryTrackingState::destruction_count, 0);
  
  // Add new vertices to ensure graph is still functional
  graph.AddVertex(MemoryTrackingState(100));
  EXPECT_TRUE(graph.FindVertex(MemoryTrackingState(100)) != graph.vertex_end());
}

TEST_F(MemoryManagementTest, RemoveVertexCleansUpMemory) {
  Graph<MemoryTrackingState> graph;
  
  // Add vertices
  for (int i = 0; i < 5; ++i) {
    graph.AddVertex(MemoryTrackingState(i));
  }
  
  // Add edges
  graph.AddEdge(MemoryTrackingState(0), MemoryTrackingState(1), 1.0);
  graph.AddEdge(MemoryTrackingState(1), MemoryTrackingState(2), 1.0);
  graph.AddEdge(MemoryTrackingState(2), MemoryTrackingState(3), 1.0);
  
  int count_before_remove = MemoryTrackingState::destruction_count;
  
  // Remove a vertex
  graph.RemoveVertex(MemoryTrackingState(1));
  
  // Should have destroyed the vertex object
  EXPECT_GT(MemoryTrackingState::destruction_count, count_before_remove);
  
  // Verify vertex is removed
  EXPECT_EQ(graph.FindVertex(MemoryTrackingState(1)), graph.vertex_end());
}

// ===== COPY AND MOVE SEMANTICS TESTS =====

TEST_F(MemoryManagementTest, CopyConstructorCreatesDeepCopy) {
  Graph<MemoryTrackingState> graph1;
  
  // Add vertices and edges to first graph
  for (int i = 0; i < 3; ++i) {
    graph1.AddVertex(MemoryTrackingState(i));
  }
  graph1.AddEdge(MemoryTrackingState(0), MemoryTrackingState(1), 1.0);
  graph1.AddEdge(MemoryTrackingState(1), MemoryTrackingState(2), 2.0);
  
  int construction_before_copy = MemoryTrackingState::construction_count;
  
  {
    // Copy construct
    Graph<MemoryTrackingState> graph2(graph1);
    
    // Should have created new objects
    EXPECT_GT(MemoryTrackingState::construction_count, construction_before_copy);
    
    // Verify both graphs have same structure
    EXPECT_EQ(graph2.GetTotalVertexNumber(), graph1.GetTotalVertexNumber());
    EXPECT_EQ(graph2.GetTotalEdgeNumber(), graph1.GetTotalEdgeNumber());
    
    // Modify graph2 shouldn't affect graph1
    graph2.RemoveVertex(MemoryTrackingState(0));
    EXPECT_NE(graph1.FindVertex(MemoryTrackingState(0)), graph1.vertex_end());
  }
  
  // graph2 destroyed, original graph1 should still be valid
  EXPECT_EQ(graph1.GetTotalVertexNumber(), 3);
}

TEST_F(MemoryManagementTest, AssignmentOperatorHandlesMemoryCorrectly) {
  Graph<MemoryTrackingState> graph1;
  Graph<MemoryTrackingState> graph2;
  
  // Add vertices to both graphs
  for (int i = 0; i < 3; ++i) {
    graph1.AddVertex(MemoryTrackingState(i));
    graph2.AddVertex(MemoryTrackingState(i + 10));
  }
  
  int destruction_before_assign = MemoryTrackingState::destruction_count;
  
  // Assignment should clean up graph2's old data
  graph2 = graph1;
  
  // Should have destroyed old graph2 vertices
  EXPECT_GT(MemoryTrackingState::destruction_count, destruction_before_assign);
  
  // Verify graph2 now has graph1's structure
  EXPECT_NE(graph2.FindVertex(MemoryTrackingState(0)), graph2.vertex_end());
  EXPECT_EQ(graph2.FindVertex(MemoryTrackingState(10)), graph2.vertex_end());
}

TEST_F(MemoryManagementTest, MoveConstructorTransfersOwnership) {
  Graph<MemoryTrackingState> graph1;
  
  // Add vertices
  for (int i = 0; i < 5; ++i) {
    graph1.AddVertex(MemoryTrackingState(i));
  }
  
  int construction_before_move = MemoryTrackingState::construction_count;
  
  // Move construct
  Graph<MemoryTrackingState> graph2(std::move(graph1));
  
  // Should not create new vertex objects (ownership transferred)
  EXPECT_EQ(MemoryTrackingState::construction_count, construction_before_move);
  
  // graph2 should have the vertices
  EXPECT_EQ(graph2.GetTotalVertexNumber(), 5);
  
  // graph1 should be empty after move
  EXPECT_EQ(graph1.GetTotalVertexNumber(), 0);
}

// ===== EXCEPTION SAFETY TESTS =====

// Test state that throws during construction
class ThrowingState {
public:
  static int throw_after_count;
  static int construction_count;
  
  ThrowingState(int64_t id) : id_(id) {
    construction_count++;
    if (throw_after_count > 0 && construction_count >= throw_after_count) {
      throw std::runtime_error("Construction exception");
    }
  }
  
  int64_t id_;
  
  static void Reset() {
    throw_after_count = 0;
    construction_count = 0;
  }
};

int ThrowingState::throw_after_count = 0;
int ThrowingState::construction_count = 0;

TEST_F(MemoryManagementTest, ExceptionDuringVertexAdditionDoesNotLeak) {
  Graph<ThrowingState> graph;
  ThrowingState::Reset();
  
  // Add some vertices successfully
  graph.AddVertex(ThrowingState(1));
  graph.AddVertex(ThrowingState(2));
  
  // Set to throw on the next construction (after all the copies during vertex creation)
  ThrowingState::throw_after_count = ThrowingState::construction_count + 1;
  
  // This should throw
  EXPECT_THROW(graph.AddVertex(ThrowingState(3)), std::runtime_error);
  
  // Reset throwing behavior for validation
  ThrowingState::throw_after_count = 0;
  
  // Graph should still be valid with original vertices
  EXPECT_EQ(graph.GetTotalVertexNumber(), 2);
  EXPECT_NE(graph.FindVertex(ThrowingState(1)), graph.vertex_end());
  EXPECT_NE(graph.FindVertex(ThrowingState(2)), graph.vertex_end());
}

// ===== LARGE GRAPH MEMORY TESTS =====

TEST_F(MemoryManagementTest, LargeGraphMemoryManagement) {
  const int VERTEX_COUNT = 1000;
  const int EDGE_COUNT = 5000;
  
  {
    Graph<MemoryTrackingState> graph;
    
    // Add many vertices
    for (int i = 0; i < VERTEX_COUNT; ++i) {
      graph.AddVertex(MemoryTrackingState(i));
    }
    
    // Add many edges (random connections)
    for (int i = 0; i < EDGE_COUNT; ++i) {
      int src = i % VERTEX_COUNT;
      int dst = (i * 7 + 3) % VERTEX_COUNT;  // Pseudo-random destination
      graph.AddEdge(MemoryTrackingState(src), MemoryTrackingState(dst), i * 0.1);
    }
    
    EXPECT_EQ(graph.GetTotalVertexNumber(), VERTEX_COUNT);
    
    // Clear half the vertices
    for (int i = 0; i < VERTEX_COUNT / 2; ++i) {
      graph.RemoveVertex(MemoryTrackingState(i * 2));
    }
    
    EXPECT_EQ(graph.GetTotalVertexNumber(), VERTEX_COUNT / 2);
  }
  
  // TearDown will verify all memory was properly cleaned up
}

// ===== CYCLIC REFERENCE TESTS =====

TEST_F(MemoryManagementTest, CyclicGraphStructureNoLeak) {
  {
    Graph<MemoryTrackingState> graph;
    
    // Create a cyclic graph structure
    for (int i = 0; i < 10; ++i) {
      graph.AddVertex(MemoryTrackingState(i));
    }
    
    // Create cycles
    for (int i = 0; i < 10; ++i) {
      graph.AddEdge(MemoryTrackingState(i), 
                    MemoryTrackingState((i + 1) % 10), 1.0);
      graph.AddEdge(MemoryTrackingState(i), 
                    MemoryTrackingState((i + 5) % 10), 2.0);
    }
    
    // Graph with cycles should still clean up properly
  }
  
  // TearDown will verify no memory leaks
}

// ===== SELF-REFERENTIAL EDGE TESTS =====

TEST_F(MemoryManagementTest, SelfLoopMemoryManagement) {
  Graph<MemoryTrackingState> graph;
  
  // Create vertices with self-loops
  for (int i = 0; i < 5; ++i) {
    graph.AddVertex(MemoryTrackingState(i));
    graph.AddEdge(MemoryTrackingState(i), MemoryTrackingState(i), 1.0);
  }
  
  // Remove vertices with self-loops
  for (int i = 0; i < 5; ++i) {
    graph.RemoveVertex(MemoryTrackingState(i));
  }
  
  EXPECT_EQ(graph.GetTotalVertexNumber(), 0);
  EXPECT_EQ(graph.GetTotalEdgeNumber(), 0);
}

// ===== TREE MEMORY MANAGEMENT TESTS =====

TEST_F(MemoryManagementTest, TreeDestructorCleansUpMemory) {
  {
    Tree<MemoryTrackingState> tree;
    
    // Build a tree structure - Tree auto-creates root when adding first edge
    tree.AddEdge(MemoryTrackingState(0), MemoryTrackingState(1), 1.0);
    tree.AddEdge(MemoryTrackingState(0), MemoryTrackingState(2), 1.0);
    tree.AddEdge(MemoryTrackingState(1), MemoryTrackingState(3), 1.0);
    tree.AddEdge(MemoryTrackingState(1), MemoryTrackingState(4), 1.0);
    
    // Tree goes out of scope, should clean up
  }
  
  // TearDown will verify all memory was cleaned up
}

TEST_F(MemoryManagementTest, TreeSubtreeRemovalCleansUpMemory) {
  Tree<MemoryTrackingState> tree;
  
  // Build a tree - Tree auto-creates vertices when adding edges
  tree.AddEdge(MemoryTrackingState(0), MemoryTrackingState(1), 1.0);
  tree.AddEdge(MemoryTrackingState(0), MemoryTrackingState(2), 1.0);
  tree.AddEdge(MemoryTrackingState(1), MemoryTrackingState(3), 1.0);
  tree.AddEdge(MemoryTrackingState(1), MemoryTrackingState(4), 1.0);
  tree.AddEdge(MemoryTrackingState(2), MemoryTrackingState(5), 1.0);
  
  int destruction_before = MemoryTrackingState::destruction_count;
  
  // Remove a subtree
  tree.RemoveSubtree(MemoryTrackingState(1));
  
  // Should have destroyed the subtree vertices
  EXPECT_GT(MemoryTrackingState::destruction_count, destruction_before);
  
  // Verify subtree is removed
  EXPECT_EQ(tree.FindVertex(MemoryTrackingState(1)), tree.vertex_end());
  EXPECT_EQ(tree.FindVertex(MemoryTrackingState(3)), tree.vertex_end());
  EXPECT_EQ(tree.FindVertex(MemoryTrackingState(4)), tree.vertex_end());
  
  // Other vertices should still exist
  EXPECT_NE(tree.FindVertex(MemoryTrackingState(0)), tree.vertex_end());
  EXPECT_NE(tree.FindVertex(MemoryTrackingState(2)), tree.vertex_end());
  EXPECT_NE(tree.FindVertex(MemoryTrackingState(5)), tree.vertex_end());
}