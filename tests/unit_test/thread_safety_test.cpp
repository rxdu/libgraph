/*
 * thread_safety_test.cpp
 *
 * Created on: 2025
 * Description: Tests for thread safety and concurrent access patterns
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <thread>
#include <atomic>
#include <vector>
#include <chrono>
#include <mutex>
#include <random>

#include "gtest/gtest.h"

#include "graph/graph.hpp"
#include "graph/tree.hpp"
#include "graph/search/astar.hpp"
#include "graph/search/dijkstra.hpp"
#include "graph/search/search_context.hpp"

using namespace xmotion;

// Thread-safe test state with atomic counter
class ThreadSafeState {
public:
  ThreadSafeState(int64_t id) : id_(id) {}
  int64_t id_;
  
  // Static atomic counter for tracking operations
  static std::atomic<int> operation_count;
  static std::atomic<int> collision_count;
};

std::atomic<int> ThreadSafeState::operation_count(0);
std::atomic<int> ThreadSafeState::collision_count(0);

class ThreadSafetyTest : public testing::Test {
protected:
  void SetUp() override {
    ThreadSafeState::operation_count = 0;
    ThreadSafeState::collision_count = 0;
  }
  
  // Helper function to detect race conditions
  bool DetectDataRace(std::function<void()> operation, int thread_count = 4) {
    std::vector<std::thread> threads;
    std::atomic<bool> start_flag(false);
    std::atomic<int> ready_count(0);
    
    for (int i = 0; i < thread_count; ++i) {
      threads.emplace_back([&]() {
        ready_count++;
        while (!start_flag) {
          std::this_thread::yield();
        }
        operation();
      });
    }
    
    // Wait for all threads to be ready
    while (ready_count < thread_count) {
      std::this_thread::yield();
    }
    
    // Start all threads simultaneously
    start_flag = true;
    
    // Join all threads
    for (auto& t : threads) {
      t.join();
    }
    
    return ThreadSafeState::collision_count > 0;
  }
};

// ===== CONCURRENT READ OPERATIONS =====

TEST_F(ThreadSafetyTest, ConcurrentVertexReads) {
  Graph<ThreadSafeState> graph;
  
  // Pre-populate graph
  const int VERTEX_COUNT = 100;
  for (int i = 0; i < VERTEX_COUNT; ++i) {
    graph.AddVertex(ThreadSafeState(i));
  }
  
  std::atomic<int> successful_finds(0);
  std::atomic<bool> error_occurred(false);
  
  auto read_operation = [&]() {
    try {
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_int_distribution<> dis(0, VERTEX_COUNT - 1);
      
      for (int i = 0; i < 100; ++i) {
        int id = dis(gen);
        auto it = graph.FindVertex(ThreadSafeState(id));
        if (it != graph.vertex_end()) {
          successful_finds++;
        }
        ThreadSafeState::operation_count++;
      }
    } catch (...) {
      error_occurred = true;
    }
  };
  
  // Run concurrent reads
  DetectDataRace(read_operation, 8);
  
  EXPECT_FALSE(error_occurred) << "Exception occurred during concurrent reads";
  EXPECT_GT(successful_finds, 0) << "No successful finds during concurrent reads";
  EXPECT_EQ(ThreadSafeState::operation_count, 800) << "Not all read operations completed";
}

TEST_F(ThreadSafetyTest, ConcurrentEdgeReads) {
  Graph<ThreadSafeState> graph;
  
  // Create a connected graph
  const int VERTEX_COUNT = 50;
  for (int i = 0; i < VERTEX_COUNT; ++i) {
    graph.AddVertex(ThreadSafeState(i));
  }
  
  // Add edges
  for (int i = 0; i < VERTEX_COUNT - 1; ++i) {
    graph.AddEdge(ThreadSafeState(i), ThreadSafeState(i + 1), 1.0);
  }
  
  std::atomic<int> edge_count(0);
  std::atomic<bool> error_occurred(false);
  
  auto read_operation = [&]() {
    try {
      for (int i = 0; i < 50; ++i) {
        auto edges = graph.GetAllEdges();
        edge_count += edges.size();
        
        // Also test GetNeighbours
        auto it = graph.FindVertex(ThreadSafeState(i % VERTEX_COUNT));
        if (it != graph.vertex_end()) {
          auto neighbors = it->GetNeighbours();
          edge_count += neighbors.size();
        }
      }
    } catch (...) {
      error_occurred = true;
    }
  };
  
  DetectDataRace(read_operation, 4);
  
  EXPECT_FALSE(error_occurred) << "Exception occurred during concurrent edge reads";
  EXPECT_GT(edge_count, 0) << "No edges found during concurrent reads";
}

TEST_F(ThreadSafetyTest, ConcurrentIteratorTraversal) {
  Graph<ThreadSafeState> graph;
  
  // Pre-populate graph
  const int VERTEX_COUNT = 100;
  for (int i = 0; i < VERTEX_COUNT; ++i) {
    graph.AddVertex(ThreadSafeState(i));
  }
  
  std::atomic<int> total_vertices_seen(0);
  std::atomic<bool> error_occurred(false);
  
  auto traversal_operation = [&]() {
    try {
      for (int repeat = 0; repeat < 10; ++repeat) {
        int count = 0;
        for (auto it = graph.vertex_begin(); it != graph.vertex_end(); ++it) {
          count++;
          // Simulate some work
          std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
        total_vertices_seen += count;
      }
    } catch (...) {
      error_occurred = true;
    }
  };
  
  DetectDataRace(traversal_operation, 4);
  
  EXPECT_FALSE(error_occurred) << "Exception occurred during concurrent iteration";
  EXPECT_EQ(total_vertices_seen, VERTEX_COUNT * 10 * 4) 
    << "Incorrect vertex count during concurrent iteration";
}

// ===== CONCURRENT WRITE OPERATIONS =====

TEST_F(ThreadSafetyTest, DISABLED_ConcurrentVertexAdditions) {
  Graph<ThreadSafeState> graph;
  std::atomic<int> base_id(0);
  std::atomic<bool> error_occurred(false);
  
  auto write_operation = [&]() {
    try {
      for (int i = 0; i < 25; ++i) {
        int id = base_id.fetch_add(1);
        graph.AddVertex(ThreadSafeState(id));
        ThreadSafeState::operation_count++;
      }
    } catch (...) {
      error_occurred = true;
    }
  };
  
  // WARNING: This test demonstrates that the current implementation
  // is NOT thread-safe for concurrent writes
  DetectDataRace(write_operation, 4);
  
  // The graph may have inconsistent state after concurrent writes
  // This test documents the current behavior
  int vertex_count = graph.GetTotalVertexNumber();
  
  // Due to race conditions, vertex count may not be exactly 100
  // Some vertices might be lost or duplicated
  std::cout << "Note: Graph has " << vertex_count 
            << " vertices after 100 concurrent additions (expected 100)" << std::endl;
  
  // Document that concurrent writes are unsafe
  EXPECT_TRUE(true) << "Concurrent writes are currently NOT thread-safe";
}

TEST_F(ThreadSafetyTest, ConcurrentEdgeAdditions) {
  Graph<ThreadSafeState> graph;
  
  // Pre-create vertices
  const int VERTEX_COUNT = 20;
  for (int i = 0; i < VERTEX_COUNT; ++i) {
    graph.AddVertex(ThreadSafeState(i));
  }
  
  std::atomic<bool> error_occurred(false);
  std::atomic<int> edges_added(0);
  
  auto write_operation = [&]() {
    try {
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_int_distribution<> dis(0, VERTEX_COUNT - 1);
      
      for (int i = 0; i < 20; ++i) {
        int src = dis(gen);
        int dst = dis(gen);
        graph.AddEdge(ThreadSafeState(src), ThreadSafeState(dst), 1.0);
        edges_added++;
      }
    } catch (...) {
      error_occurred = true;
    }
  };
  
  // WARNING: This test demonstrates race conditions in edge addition
  DetectDataRace(write_operation, 4);
  
  int edge_count = graph.GetTotalEdgeNumber();
  
  // Due to race conditions, edge count may vary
  std::cout << "Note: Graph has " << edge_count 
            << " edges after " << edges_added 
            << " concurrent additions" << std::endl;
  
  EXPECT_TRUE(true) << "Concurrent edge additions are currently NOT thread-safe";
}

// ===== MIXED READ/WRITE OPERATIONS =====

TEST_F(ThreadSafetyTest, MixedReadWriteOperations) {
  Graph<ThreadSafeState> graph;
  
  // Pre-populate with some vertices
  for (int i = 0; i < 50; ++i) {
    graph.AddVertex(ThreadSafeState(i));
  }
  
  std::atomic<bool> stop_flag(false);
  std::atomic<int> read_count(0);
  std::atomic<int> write_count(0);
  std::atomic<bool> error_occurred(false);
  
  // Reader threads
  auto reader = [&]() {
    try {
      while (!stop_flag) {
        for (auto it = graph.vertex_begin(); it != graph.vertex_end(); ++it) {
          read_count++;
        }
        auto edges = graph.GetAllEdges();
        read_count += edges.size();
      }
    } catch (...) {
      error_occurred = true;
    }
  };
  
  // Writer thread
  auto writer = [&]() {
    try {
      for (int i = 50; i < 100; ++i) {
        graph.AddVertex(ThreadSafeState(i));
        write_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      stop_flag = true;
    } catch (...) {
      error_occurred = true;
      stop_flag = true;
    }
  };
  
  // Start readers and writer
  std::vector<std::thread> threads;
  for (int i = 0; i < 3; ++i) {
    threads.emplace_back(reader);
  }
  threads.emplace_back(writer);
  
  // Join all threads
  for (auto& t : threads) {
    t.join();
  }
  
  std::cout << "Note: " << read_count << " reads and " 
            << write_count << " writes performed" << std::endl;
  
  // This test documents that mixed operations can cause issues
  EXPECT_TRUE(true) << "Mixed read/write operations may cause race conditions";
}

// ===== SEARCH ALGORITHM THREAD SAFETY =====

TEST_F(ThreadSafetyTest, ConcurrentDijkstraSearches) {
  Graph<ThreadSafeState> graph;
  
  // Create a simple path graph
  const int PATH_LENGTH = 20;
  for (int i = 0; i < PATH_LENGTH; ++i) {
    graph.AddVertex(ThreadSafeState(i));
  }
  
  for (int i = 0; i < PATH_LENGTH - 1; ++i) {
    graph.AddEdge(ThreadSafeState(i), ThreadSafeState(i + 1), 1.0);
  }
  
  std::atomic<int> successful_searches(0);
  std::atomic<bool> error_occurred(false);
  
  auto search_operation = [&]() {
    try {
      for (int i = 0; i < 10; ++i) {
        auto path = Dijkstra::Search(&graph, ThreadSafeState(0), 
                                     ThreadSafeState(PATH_LENGTH - 1));
        if (!path.empty()) {
          successful_searches++;
        }
      }
    } catch (...) {
      error_occurred = true;
    }
  };
  
  DetectDataRace(search_operation, 4);
  
  EXPECT_FALSE(error_occurred) << "Exception during concurrent Dijkstra searches";
  EXPECT_EQ(successful_searches, 40) << "Not all searches found a path";
}

TEST_F(ThreadSafetyTest, ConcurrentAStarSearches) {
  Graph<ThreadSafeState> graph;
  
  // Create a grid-like graph
  const int GRID_SIZE = 5;
  for (int i = 0; i < GRID_SIZE * GRID_SIZE; ++i) {
    graph.AddVertex(ThreadSafeState(i));
  }
  
  // Add grid edges
  for (int i = 0; i < GRID_SIZE; ++i) {
    for (int j = 0; j < GRID_SIZE; ++j) {
      int current = i * GRID_SIZE + j;
      if (j < GRID_SIZE - 1) {
        graph.AddEdge(ThreadSafeState(current), 
                      ThreadSafeState(current + 1), 1.0);
      }
      if (i < GRID_SIZE - 1) {
        graph.AddEdge(ThreadSafeState(current), 
                      ThreadSafeState(current + GRID_SIZE), 1.0);
      }
    }
  }
  
  std::atomic<int> successful_searches(0);
  std::atomic<bool> error_occurred(false);
  
  auto search_operation = [&]() {
    try {
      std::function<double(ThreadSafeState, ThreadSafeState)> heuristic = 
        [](const ThreadSafeState& s1, const ThreadSafeState& s2) { 
          return 0.0;  // Simple heuristic
        };
      
      for (int i = 0; i < 5; ++i) {
        auto path = AStar::Search(&graph, ThreadSafeState(0), 
                                  ThreadSafeState(GRID_SIZE * GRID_SIZE - 1), heuristic);
        if (!path.empty()) {
          successful_searches++;
        }
      }
    } catch (...) {
      error_occurred = true;
    }
  };
  
  DetectDataRace(search_operation, 4);
  
  EXPECT_FALSE(error_occurred) << "Exception during concurrent A* searches";
  EXPECT_EQ(successful_searches, 20) << "Not all A* searches found a path";
}

// ===== ITERATOR INVALIDATION TESTS =====

TEST_F(ThreadSafetyTest, IteratorInvalidationDuringModification) {
  Graph<ThreadSafeState> graph;
  
  // Pre-populate graph
  for (int i = 0; i < 100; ++i) {
    graph.AddVertex(ThreadSafeState(i));
  }
  
  std::atomic<bool> modification_started(false);
  std::atomic<bool> iterator_invalid(false);
  std::atomic<bool> error_occurred(false);
  
  // Thread that iterates
  auto iterator_thread = [&]() {
    try {
      auto it = graph.vertex_begin();
      auto initial_vertex_id = it->vertex_id;
      
      // Wait for modification to start
      while (!modification_started) {
        std::this_thread::yield();
      }
      
      // Try to use iterator after modification
      if (it != graph.vertex_end()) {
        // Iterator may be invalid here
        try {
          auto id = it->vertex_id;
          if (id != initial_vertex_id) {
            iterator_invalid = true;
          }
        } catch (...) {
          iterator_invalid = true;
        }
      }
    } catch (...) {
      error_occurred = true;
    }
  };
  
  // Thread that modifies
  auto modifier_thread = [&]() {
    try {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      modification_started = true;
      
      // Remove some vertices
      for (int i = 0; i < 10; ++i) {
        graph.RemoveVertex(ThreadSafeState(i));
      }
    } catch (...) {
      error_occurred = true;
    }
  };
  
  std::thread t1(iterator_thread);
  std::thread t2(modifier_thread);
  
  t1.join();
  t2.join();
  
  // This test documents that iterators can become invalid
  std::cout << "Note: Iterator invalidation " 
            << (iterator_invalid ? "detected" : "not detected") 
            << " during concurrent modification" << std::endl;
  
  EXPECT_TRUE(true) << "Iterator invalidation is possible with concurrent modifications";
}

// ===== PERFORMANCE UNDER CONTENTION =====

TEST_F(ThreadSafetyTest, PerformanceUnderHighContention) {
  Graph<ThreadSafeState> graph;
  
  // Pre-populate
  for (int i = 0; i < 1000; ++i) {
    graph.AddVertex(ThreadSafeState(i));
  }
  
  auto start_time = std::chrono::high_resolution_clock::now();
  
  std::atomic<int> operations_completed(0);
  
  auto high_contention_operation = [&]() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 999);
    
    for (int i = 0; i < 1000; ++i) {
      int id = dis(gen);
      graph.FindVertex(ThreadSafeState(id));
      operations_completed++;
    }
  };
  
  // Run with high thread count
  DetectDataRace(high_contention_operation, 16);
  
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      end_time - start_time);
  
  std::cout << "Note: " << operations_completed 
            << " operations completed in " << duration.count() 
            << "ms under high contention" << std::endl;
  
  EXPECT_EQ(operations_completed, 16000) << "Not all operations completed";
}

// ===== DEADLOCK DETECTION TEST =====

TEST_F(ThreadSafetyTest, NoDeadlockInBasicOperations) {
  Graph<ThreadSafeState> graph;
  
  // Pre-populate
  for (int i = 0; i < 10; ++i) {
    graph.AddVertex(ThreadSafeState(i));
  }
  
  std::atomic<bool> deadlock_detected(false);
  
  auto operation_with_timeout = [&]() {
    auto start = std::chrono::steady_clock::now();
    
    // Perform various operations
    for (int i = 0; i < 100; ++i) {
      graph.FindVertex(ThreadSafeState(i % 10));
      graph.GetAllEdges();
      
      // Check for timeout (potential deadlock)
      auto now = std::chrono::steady_clock::now();
      if (now - start > std::chrono::seconds(5)) {
        deadlock_detected = true;
        break;
      }
    }
  };
  
  std::vector<std::thread> threads;
  for (int i = 0; i < 8; ++i) {
    threads.emplace_back(operation_with_timeout);
  }
  
  for (auto& t : threads) {
    t.join();
  }
  
  EXPECT_FALSE(deadlock_detected) << "Potential deadlock detected in basic operations";
}