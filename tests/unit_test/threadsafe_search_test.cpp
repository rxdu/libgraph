/*
 * threadsafe_search_test.cpp
 *
 * Created on: 2025
 * Description: Tests for thread-safe search algorithms using SearchContext
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <thread>
#include <atomic>
#include <vector>
#include <chrono>
#include <functional>
#include <future>

#include "gtest/gtest.h"

#include "graph/graph.hpp"
#include "graph/search/search_context.hpp"
#include "graph/search/dijkstra_threadsafe.hpp"
#include "graph/search/astar_threadsafe.hpp"
#include "graph/impl/default_indexer.hpp"

using namespace xmotion;

// Test state for thread-safe search tests
class ThreadSafeSearchState {
public:
  ThreadSafeSearchState(int64_t id) : id_(id) {}
  int64_t id_;
  int64_t GetId() const { return id_; }
  
  bool operator==(const ThreadSafeSearchState& other) const {
    return id_ == other.id_;
  }
};

class ThreadSafeSearchTest : public testing::Test {
protected:
  void SetUp() override {
    // Create a test graph: 0 -> 1 -> 2 -> 3 -> 4
    //                     |    |    |    |    |
    //                     v    v    v    v    v  
    //                     5 -> 6 -> 7 -> 8 -> 9
    for (int i = 0; i < 10; ++i) {
      test_graph_.AddVertex(ThreadSafeSearchState(i));
    }
    
    // Horizontal edges (cost 1.0)
    for (int i = 0; i < 4; ++i) {
      test_graph_.AddEdge(ThreadSafeSearchState(i), ThreadSafeSearchState(i + 1), 1.0);
      test_graph_.AddEdge(ThreadSafeSearchState(i + 5), ThreadSafeSearchState(i + 6), 1.0);
    }
    
    // Vertical edges (cost 2.0) 
    for (int i = 0; i < 5; ++i) {
      test_graph_.AddEdge(ThreadSafeSearchState(i), ThreadSafeSearchState(i + 5), 2.0);
    }
    
    // Diagonal shortcuts (cost 3.0)
    test_graph_.AddEdge(ThreadSafeSearchState(0), ThreadSafeSearchState(6), 3.0);
    test_graph_.AddEdge(ThreadSafeSearchState(1), ThreadSafeSearchState(7), 3.0);
    test_graph_.AddEdge(ThreadSafeSearchState(2), ThreadSafeSearchState(8), 3.0);
  }
  
  Graph<ThreadSafeSearchState, double> test_graph_;
};

// ===== BASIC FUNCTIONALITY TESTS =====

TEST_F(ThreadSafeSearchTest, SearchContextBasicOperations) {
  SearchContext<ThreadSafeSearchState, double, DefaultIndexer<ThreadSafeSearchState>> context;
  
  EXPECT_TRUE(context.Empty());
  EXPECT_EQ(context.Size(), 0);
  
  auto& info = context.GetSearchInfo(123);
  EXPECT_EQ(info.g_cost, std::numeric_limits<double>::max());
  EXPECT_FALSE(info.is_checked);
  
  EXPECT_FALSE(context.Empty());
  EXPECT_EQ(context.Size(), 1);
  EXPECT_TRUE(context.HasSearchInfo(123));
  
  info.g_cost = 5.0;
  info.is_checked = true;
  
  const auto& const_info = context.GetSearchInfo(123);
  EXPECT_EQ(const_info.g_cost, 5.0);
  EXPECT_TRUE(const_info.is_checked);
  
  context.Reset();
  EXPECT_EQ(context.Size(), 1);
  EXPECT_EQ(context.GetSearchInfo(123).g_cost, std::numeric_limits<double>::max());
  EXPECT_FALSE(context.GetSearchInfo(123).is_checked);
  
  context.Clear();
  EXPECT_TRUE(context.Empty());
  EXPECT_EQ(context.Size(), 0);
}

TEST_F(ThreadSafeSearchTest, DijkstraThreadSafeBasicPath) {
  SearchContext<ThreadSafeSearchState, double, DefaultIndexer<ThreadSafeSearchState>> context;
  
  auto path = DijkstraThreadSafe::Search(&test_graph_, context,
                                         ThreadSafeSearchState(0), 
                                         ThreadSafeSearchState(4));
  
  ASSERT_EQ(path.size(), 5);
  for (size_t i = 0; i < path.size(); ++i) {
    EXPECT_EQ(path[i].id_, i);
  }
  
  // Verify context has search information
  EXPECT_FALSE(context.Empty());
  EXPECT_TRUE(context.HasSearchInfo(0));
  EXPECT_TRUE(context.HasSearchInfo(4));
  EXPECT_EQ(context.GetSearchInfo(0).g_cost, 0.0);
  EXPECT_EQ(context.GetSearchInfo(4).g_cost, 4.0);
}

TEST_F(ThreadSafeSearchTest, AStarThreadSafeBasicPath) {
  SearchContext<ThreadSafeSearchState, double, DefaultIndexer<ThreadSafeSearchState>> context;
  
  auto heuristic = [](const ThreadSafeSearchState& s1, const ThreadSafeSearchState& s2) {
    return std::abs(s1.id_ - s2.id_);
  };
  
  auto path = AStarThreadSafe::Search(&test_graph_, context,
                                      ThreadSafeSearchState(0), 
                                      ThreadSafeSearchState(9),
                                      heuristic);
  
  EXPECT_FALSE(path.empty());
  EXPECT_EQ(path.front().id_, 0);
  EXPECT_EQ(path.back().id_, 9);
  
  // A* should find a reasonable path
  EXPECT_LE(path.size(), 8); // Should not be longer than naive path
}

TEST_F(ThreadSafeSearchTest, ConvenienceMethodsWork) {
  // Test methods that create their own context
  auto dijkstra_path = DijkstraThreadSafe::Search(&test_graph_,
                                                  ThreadSafeSearchState(0), 
                                                  ThreadSafeSearchState(4));
  EXPECT_EQ(dijkstra_path.size(), 5);
  
  auto heuristic = [](const ThreadSafeSearchState& s1, const ThreadSafeSearchState& s2) {
    return std::abs(s1.id_ - s2.id_);
  };
  
  auto astar_path = AStarThreadSafe::Search(&test_graph_,
                                            ThreadSafeSearchState(0), 
                                            ThreadSafeSearchState(9),
                                            heuristic);
  EXPECT_FALSE(astar_path.empty());
}

// ===== THREAD SAFETY TESTS =====

TEST_F(ThreadSafeSearchTest, ConcurrentDijkstraSearches) {
  const int NUM_THREADS = 8;
  const int SEARCHES_PER_THREAD = 10;
  
  std::atomic<int> successful_searches(0);
  std::atomic<int> failed_searches(0);
  std::vector<std::future<void>> futures;
  
  for (int t = 0; t < NUM_THREADS; ++t) {
    futures.push_back(std::async(std::launch::async, [&, t]() {
      for (int s = 0; s < SEARCHES_PER_THREAD; ++s) {
        try {
          // Each thread searches different paths
          int start_id = (t * 2) % 5; // 0, 2, 4, 1, 3, 0, 2, 4
          int goal_id = start_id + 5; // Bottom row
          
          auto path = DijkstraThreadSafe::Search(&test_graph_,
                                                 ThreadSafeSearchState(start_id),
                                                 ThreadSafeSearchState(goal_id));
          
          if (!path.empty() && path.front().id_ == start_id && path.back().id_ == goal_id) {
            successful_searches++;
          } else {
            failed_searches++;
          }
        } catch (...) {
          failed_searches++;
        }
      }
    }));
  }
  
  // Wait for all threads to complete
  for (auto& future : futures) {
    future.wait();
  }
  
  EXPECT_EQ(successful_searches.load(), NUM_THREADS * SEARCHES_PER_THREAD);
  EXPECT_EQ(failed_searches.load(), 0);
}

TEST_F(ThreadSafeSearchTest, ConcurrentAStarSearches) {
  const int NUM_THREADS = 6;
  const int SEARCHES_PER_THREAD = 15;
  
  std::atomic<int> successful_searches(0);
  std::atomic<int> failed_searches(0);
  std::vector<std::future<void>> futures;
  
  auto heuristic = [](const ThreadSafeSearchState& s1, const ThreadSafeSearchState& s2) {
    return std::abs(s1.id_ - s2.id_);
  };
  
  for (int t = 0; t < NUM_THREADS; ++t) {
    futures.push_back(std::async(std::launch::async, [&, t, heuristic]() {
      for (int s = 0; s < SEARCHES_PER_THREAD; ++s) {
        try {
          // Varied search patterns
          int start_id = t % 10;
          int goal_id = (start_id + 5 + s) % 10;
          
          auto path = AStarThreadSafe::Search(&test_graph_,
                                              ThreadSafeSearchState(start_id),
                                              ThreadSafeSearchState(goal_id),
                                              heuristic);
          
          if (!path.empty() && path.front().id_ == start_id && path.back().id_ == goal_id) {
            successful_searches++;
          } else {
            // Some paths might not exist, that's okay
            if (start_id == goal_id) {
              successful_searches++; // Same start/goal should be handled
            } else {
              failed_searches++;
            }
          }
        } catch (...) {
          failed_searches++;
        }
      }
    }));
  }
  
  // Wait for all threads to complete
  for (auto& future : futures) {
    future.wait();
  }
  
  EXPECT_GT(successful_searches.load(), 0);
  EXPECT_LT(failed_searches.load(), NUM_THREADS * SEARCHES_PER_THREAD / 2);
  
  std::cout << "A* concurrent searches: " << successful_searches.load() 
            << " successful, " << failed_searches.load() << " failed\n";
}

TEST_F(ThreadSafeSearchTest, MixedConcurrentSearchAlgorithms) {
  const int NUM_THREADS = 10;
  const int OPERATIONS_PER_THREAD = 8;
  
  std::atomic<int> dijkstra_success(0);
  std::atomic<int> astar_success(0);
  std::atomic<int> failures(0);
  std::vector<std::future<void>> futures;
  
  auto heuristic = [](const ThreadSafeSearchState& s1, const ThreadSafeSearchState& s2) {
    return std::abs(s1.id_ - s2.id_);
  };
  
  for (int t = 0; t < NUM_THREADS; ++t) {
    futures.push_back(std::async(std::launch::async, [&, t, heuristic]() {
      for (int op = 0; op < OPERATIONS_PER_THREAD; ++op) {
        try {
          int start_id = (t + op) % 5;
          int goal_id = start_id + 5;
          
          if (op % 2 == 0) {
            // Use Dijkstra
            auto path = DijkstraThreadSafe::Search(&test_graph_,
                                                   ThreadSafeSearchState(start_id),
                                                   ThreadSafeSearchState(goal_id));
            if (!path.empty()) {
              dijkstra_success++;
            } else {
              failures++;
            }
          } else {
            // Use A*
            auto path = AStarThreadSafe::Search(&test_graph_,
                                                ThreadSafeSearchState(start_id),
                                                ThreadSafeSearchState(goal_id),
                                                heuristic);
            if (!path.empty()) {
              astar_success++;
            } else {
              failures++;
            }
          }
        } catch (...) {
          failures++;
        }
      }
    }));
  }
  
  // Wait for all threads to complete
  for (auto& future : futures) {
    future.wait();
  }
  
  EXPECT_GT(dijkstra_success.load(), 0);
  EXPECT_GT(astar_success.load(), 0);
  EXPECT_EQ(failures.load(), 0);
  
  std::cout << "Mixed concurrent: " << dijkstra_success.load() << " Dijkstra, "
            << astar_success.load() << " A*, " << failures.load() << " failures\n";
}

// ===== PERFORMANCE AND STRESS TESTS =====

TEST_F(ThreadSafeSearchTest, ContextReusePerformance) {
  SearchContext<ThreadSafeSearchState, double, DefaultIndexer<ThreadSafeSearchState>> reused_context;
  SearchContext<ThreadSafeSearchState, double, DefaultIndexer<ThreadSafeSearchState>> fresh_context;
  
  const int NUM_SEARCHES = 100;
  
  // Time reused context
  auto start_time = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < NUM_SEARCHES; ++i) {
    reused_context.Reset(); // Reset instead of clear for performance
    DijkstraThreadSafe::Search(&test_graph_, reused_context,
                               ThreadSafeSearchState(0), 
                               ThreadSafeSearchState(4));
  }
  auto reused_time = std::chrono::high_resolution_clock::now() - start_time;
  
  // Time fresh contexts
  start_time = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < NUM_SEARCHES; ++i) {
    SearchContext<ThreadSafeSearchState, double, DefaultIndexer<ThreadSafeSearchState>> temp_context;
    DijkstraThreadSafe::Search(&test_graph_, temp_context,
                               ThreadSafeSearchState(0), 
                               ThreadSafeSearchState(4));
  }
  auto fresh_time = std::chrono::high_resolution_clock::now() - start_time;
  
  // Reused context should be faster or at least not significantly slower
  auto reused_ms = std::chrono::duration_cast<std::chrono::microseconds>(reused_time).count();
  auto fresh_ms = std::chrono::duration_cast<std::chrono::microseconds>(fresh_time).count();
  
  std::cout << "Context reuse performance - Reused: " << reused_ms 
            << "μs, Fresh: " << fresh_ms << "μs\n";
  
  EXPECT_LT(reused_ms, fresh_ms * 2); // Reused should not be more than 2x slower
}

TEST_F(ThreadSafeSearchTest, HighConcurrencyStressTest) {
  const int NUM_THREADS = 20;
  const int OPERATIONS_PER_THREAD = 50;
  
  std::atomic<int> total_operations(0);
  std::atomic<int> successful_operations(0);
  std::vector<std::future<void>> futures;
  
  auto start_time = std::chrono::high_resolution_clock::now();
  
  for (int t = 0; t < NUM_THREADS; ++t) {
    futures.push_back(std::async(std::launch::async, [&, t]() {
      for (int op = 0; op < OPERATIONS_PER_THREAD; ++op) {
        total_operations++;
        try {
          int start_id = (t * 7 + op * 3) % 10;
          int goal_id = (start_id + 5) % 10;
          
          auto path = DijkstraThreadSafe::Search(&test_graph_,
                                                 ThreadSafeSearchState(start_id),
                                                 ThreadSafeSearchState(goal_id));
          
          if (!path.empty()) {
            successful_operations++;
          }
        } catch (...) {
          // Count failed operations but don't fail the test
        }
      }
    }));
  }
  
  // Wait for all threads to complete
  for (auto& future : futures) {
    future.wait();
  }
  
  auto duration = std::chrono::high_resolution_clock::now() - start_time;
  auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
  
  EXPECT_EQ(total_operations.load(), NUM_THREADS * OPERATIONS_PER_THREAD);
  EXPECT_GT(successful_operations.load(), total_operations.load() * 0.4); // At least 40% success (50% paths exist)
  
  std::cout << "High concurrency stress test: " << successful_operations.load() 
            << "/" << total_operations.load() << " operations successful in " 
            << duration_ms << "ms\n";
}

TEST_F(ThreadSafeSearchTest, NoPathFoundThreadSafety) {
  // Create a disconnected graph for testing no-path scenarios
  Graph<ThreadSafeSearchState, double> disconnected_graph;
  
  // Island 1: 0-1-2
  for (int i = 0; i < 3; ++i) {
    disconnected_graph.AddVertex(ThreadSafeSearchState(i));
  }
  disconnected_graph.AddEdge(ThreadSafeSearchState(0), ThreadSafeSearchState(1), 1.0);
  disconnected_graph.AddEdge(ThreadSafeSearchState(1), ThreadSafeSearchState(2), 1.0);
  
  // Island 2: 10-11-12
  for (int i = 10; i < 13; ++i) {
    disconnected_graph.AddVertex(ThreadSafeSearchState(i));
  }
  disconnected_graph.AddEdge(ThreadSafeSearchState(10), ThreadSafeSearchState(11), 1.0);
  disconnected_graph.AddEdge(ThreadSafeSearchState(11), ThreadSafeSearchState(12), 1.0);
  
  const int NUM_THREADS = 4;
  std::vector<std::future<bool>> futures;
  
  for (int t = 0; t < NUM_THREADS; ++t) {
    futures.push_back(std::async(std::launch::async, [&]() {
      try {
        // Try to find path between disconnected components
        auto path = DijkstraThreadSafe::Search(&disconnected_graph,
                                               ThreadSafeSearchState(0),
                                               ThreadSafeSearchState(10));
        return path.empty(); // Should be empty (no path)
      } catch (...) {
        return false; // Exception is failure
      }
    }));
  }
  
  // All threads should return true (empty path, no exceptions)
  for (auto& future : futures) {
    EXPECT_TRUE(future.get());
  }
}