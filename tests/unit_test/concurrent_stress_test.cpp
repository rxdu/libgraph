/*
 * concurrent_stress_test.cpp
 *
 * Created on: Dec 2025
 * Description: Concurrent search stress tests for production readiness
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <atomic>
#include <future>
#include <mutex>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "graph/graph.hpp"
#include "graph/search/astar.hpp"
#include "graph/search/bfs.hpp"
#include "graph/search/dfs.hpp"
#include "graph/search/dijkstra.hpp"

using namespace xmotion;

namespace {

// Simple state for testing
struct TestState {
  int64_t id;
  int64_t GetId() const { return id; }

  bool operator==(const TestState& other) const { return id == other.id; }
};

// Type aliases for cleaner code
using TestGraph = Graph<TestState>;
using TestSearchContext = SearchContext<TestState, double, DefaultIndexer<TestState>>;

// Heuristic for A*
double SimpleHeuristic(TestState a, TestState b) {
  return static_cast<double>(std::abs(a.id - b.id));
}

// Helper to convert path to vector of IDs
std::vector<int64_t> PathToIds(const Path<TestState>& path) {
  std::vector<int64_t> ids;
  for (const auto& state : path) {
    ids.push_back(state.id);
  }
  return ids;
}

}  // namespace

class ConcurrentStressTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a reasonably-sized test graph
    const int size = 500;
    for (int i = 0; i < size; ++i) {
      graph_.AddVertex(TestState{i});
    }

    // Create edges
    for (int i = 0; i < size - 1; ++i) {
      graph_.AddEdge(TestState{i}, TestState{i + 1}, 1.0);
      graph_.AddEdge(TestState{i + 1}, TestState{i}, 1.0);

      if (i + 5 < size) {
        graph_.AddEdge(TestState{i}, TestState{i + 5}, 3.0);
        graph_.AddEdge(TestState{i + 5}, TestState{i}, 3.0);
      }

      if (i + 10 < size) {
        graph_.AddEdge(TestState{i}, TestState{i + 10}, 7.0);
        graph_.AddEdge(TestState{i + 10}, TestState{i}, 7.0);
      }
    }
  }

  Graph<TestState> graph_;
};

TEST_F(ConcurrentStressTest, SimultaneousDijkstraSearches_100) {
  const int num_threads = 100;
  std::atomic<int> success_count{0};
  std::atomic<int> failure_count{0};

  // Get reference path for validation
  TestSearchContext ref_context;
  auto ref_path = Dijkstra::Search(&graph_, ref_context, TestState{0}, TestState{499});
  ASSERT_FALSE(ref_path.empty());
  auto ref_ids = PathToIds(ref_path);

  // Launch concurrent searches
  std::vector<std::future<bool>> futures;
  futures.reserve(num_threads);

  for (int i = 0; i < num_threads; ++i) {
    futures.push_back(std::async(std::launch::async, [this, &ref_ids]() {
      TestSearchContext context;
      auto path = Dijkstra::Search(&graph_, context, TestState{0}, TestState{499});

      if (path.empty()) return false;

      auto ids = PathToIds(path);
      return ids == ref_ids;
    }));
  }

  // Collect results
  for (auto& future : futures) {
    if (future.get()) {
      success_count++;
    } else {
      failure_count++;
    }
  }

  EXPECT_EQ(success_count.load(), num_threads)
      << "All concurrent searches should succeed with correct path";
  EXPECT_EQ(failure_count.load(), 0) << "No concurrent searches should fail";
}

TEST_F(ConcurrentStressTest, SimultaneousAStarSearches_100) {
  const int num_threads = 100;
  std::atomic<int> success_count{0};

  // Get reference path for validation
  TestSearchContext ref_context;
  auto ref_path =
      AStar::Search(&graph_, ref_context, TestState{0}, TestState{499}, SimpleHeuristic);
  ASSERT_FALSE(ref_path.empty());
  auto ref_ids = PathToIds(ref_path);

  // Launch concurrent searches
  std::vector<std::future<bool>> futures;
  futures.reserve(num_threads);

  for (int i = 0; i < num_threads; ++i) {
    futures.push_back(std::async(std::launch::async, [this, &ref_ids]() {
      TestSearchContext context;
      auto path =
          AStar::Search(&graph_, context, TestState{0}, TestState{499}, SimpleHeuristic);

      if (path.empty()) return false;

      auto ids = PathToIds(path);
      return ids == ref_ids;
    }));
  }

  // Collect results
  for (auto& future : futures) {
    if (future.get()) {
      success_count++;
    }
  }

  EXPECT_EQ(success_count.load(), num_threads)
      << "All concurrent A* searches should succeed";
}

TEST_F(ConcurrentStressTest, DifferentAlgorithmsMixed) {
  const int searches_per_algorithm = 25;  // 100 total
  std::atomic<int> dijkstra_success{0};
  std::atomic<int> astar_success{0};
  std::atomic<int> bfs_success{0};
  std::atomic<int> dfs_success{0};

  // Get reference paths
  TestSearchContext ref_ctx;
  auto dijkstra_ref = Dijkstra::Search(&graph_, ref_ctx, TestState{0}, TestState{499});
  auto dijkstra_ref_ids = PathToIds(dijkstra_ref);

  TestSearchContext astar_ctx;
  auto astar_ref =
      AStar::Search(&graph_, astar_ctx, TestState{0}, TestState{499}, SimpleHeuristic);
  auto astar_ref_ids = PathToIds(astar_ref);

  TestSearchContext bfs_ctx;
  auto bfs_ref = BFS::Search(&graph_, bfs_ctx, TestState{0}, TestState{499});
  auto bfs_ref_ids = PathToIds(bfs_ref);

  TestSearchContext dfs_ctx;
  auto dfs_ref = DFS::Search(&graph_, dfs_ctx, TestState{0}, TestState{499});
  auto dfs_ref_ids = PathToIds(dfs_ref);

  ASSERT_FALSE(dijkstra_ref.empty());
  ASSERT_FALSE(astar_ref.empty());
  ASSERT_FALSE(bfs_ref.empty());
  ASSERT_FALSE(dfs_ref.empty());

  // Launch mixed concurrent searches
  std::vector<std::future<void>> futures;

  // Dijkstra threads
  for (int i = 0; i < searches_per_algorithm; ++i) {
    futures.push_back(
        std::async(std::launch::async, [this, &dijkstra_ref_ids, &dijkstra_success]() {
          TestSearchContext context;
          auto path = Dijkstra::Search(&graph_, context, TestState{0}, TestState{499});
          if (!path.empty() && PathToIds(path) == dijkstra_ref_ids) {
            dijkstra_success++;
          }
        }));
  }

  // A* threads
  for (int i = 0; i < searches_per_algorithm; ++i) {
    futures.push_back(
        std::async(std::launch::async, [this, &astar_ref_ids, &astar_success]() {
          TestSearchContext context;
          auto path = AStar::Search(&graph_, context, TestState{0}, TestState{499},
                                    SimpleHeuristic);
          if (!path.empty() && PathToIds(path) == astar_ref_ids) {
            astar_success++;
          }
        }));
  }

  // BFS threads
  for (int i = 0; i < searches_per_algorithm; ++i) {
    futures.push_back(
        std::async(std::launch::async, [this, &bfs_ref_ids, &bfs_success]() {
          TestSearchContext context;
          auto path = BFS::Search(&graph_, context, TestState{0}, TestState{499});
          if (!path.empty() && PathToIds(path) == bfs_ref_ids) {
            bfs_success++;
          }
        }));
  }

  // DFS threads
  for (int i = 0; i < searches_per_algorithm; ++i) {
    futures.push_back(
        std::async(std::launch::async, [this, &dfs_ref_ids, &dfs_success]() {
          TestSearchContext context;
          auto path = DFS::Search(&graph_, context, TestState{0}, TestState{499});
          if (!path.empty() && PathToIds(path) == dfs_ref_ids) {
            dfs_success++;
          }
        }));
  }

  // Wait for all
  for (auto& future : futures) {
    future.get();
  }

  EXPECT_EQ(dijkstra_success.load(), searches_per_algorithm)
      << "All Dijkstra searches should succeed";
  EXPECT_EQ(astar_success.load(), searches_per_algorithm)
      << "All A* searches should succeed";
  EXPECT_EQ(bfs_success.load(), searches_per_algorithm)
      << "All BFS searches should succeed";
  EXPECT_EQ(dfs_success.load(), searches_per_algorithm)
      << "All DFS searches should succeed";
}

TEST_F(ConcurrentStressTest, DifferentStartGoalPairs) {
  // Multiple threads searching for different paths simultaneously
  const int num_queries = 50;
  std::atomic<int> success_count{0};

  // Generate different start-goal pairs
  std::vector<std::pair<int64_t, int64_t>> queries;
  for (int i = 0; i < num_queries; ++i) {
    int64_t start = (i * 7) % 400;
    int64_t goal = 400 + (i * 3) % 99;
    queries.push_back({start, goal});
  }

  // Launch concurrent searches with different queries
  std::vector<std::future<bool>> futures;
  futures.reserve(num_queries);

  for (const auto& query : queries) {
    futures.push_back(std::async(std::launch::async, [this, query]() {
      TestSearchContext context;
      auto path =
          Dijkstra::Search(&graph_, context, TestState{query.first}, TestState{query.second});
      return !path.empty();
    }));
  }

  // Collect results
  for (auto& future : futures) {
    if (future.get()) {
      success_count++;
    }
  }

  EXPECT_EQ(success_count.load(), num_queries)
      << "All different-query searches should find paths";
}

TEST_F(ConcurrentStressTest, SharedStrategyObject) {
  // Verify that strategy objects can be safely shared (they're stateless)
  const int num_threads = 50;
  std::atomic<int> success_count{0};

  // Get reference for validation
  TestSearchContext ref_context;
  auto ref_path =
      AStar::Search(&graph_, ref_context, TestState{0}, TestState{499}, SimpleHeuristic);
  ASSERT_FALSE(ref_path.empty());
  auto ref_ids = PathToIds(ref_path);

  // All threads use the same heuristic function (which is safe)
  std::vector<std::future<bool>> futures;
  futures.reserve(num_threads);

  for (int i = 0; i < num_threads; ++i) {
    futures.push_back(std::async(std::launch::async, [this, &ref_ids]() {
      TestSearchContext context;
      // All use SimpleHeuristic - should be safe as it's stateless
      auto path =
          AStar::Search(&graph_, context, TestState{0}, TestState{499}, SimpleHeuristic);

      if (path.empty()) return false;
      return PathToIds(path) == ref_ids;
    }));
  }

  for (auto& future : futures) {
    if (future.get()) {
      success_count++;
    }
  }

  EXPECT_EQ(success_count.load(), num_threads);
}

TEST_F(ConcurrentStressTest, RapidSuccessiveSearches) {
  // Test rapid back-to-back searches from same thread
  const int num_threads = 10;
  const int searches_per_thread = 100;
  std::atomic<int> total_success{0};

  std::vector<std::future<int>> futures;
  futures.reserve(num_threads);

  for (int t = 0; t < num_threads; ++t) {
    futures.push_back(std::async(std::launch::async, [this, searches_per_thread]() {
      int local_success = 0;
      TestSearchContext context;

      for (int i = 0; i < searches_per_thread; ++i) {
        auto path = Dijkstra::Search(&graph_, context, TestState{0}, TestState{499});
        if (!path.empty()) {
          local_success++;
        }
      }
      return local_success;
    }));
  }

  for (auto& future : futures) {
    total_success += future.get();
  }

  EXPECT_EQ(total_success.load(), num_threads * searches_per_thread)
      << "All rapid successive searches should succeed";
}

TEST_F(ConcurrentStressTest, HighContentionScenario) {
  // Create maximum contention by having many threads start simultaneously
  const int num_threads = 200;
  std::atomic<int> success_count{0};
  std::atomic<int> ready_count{0};
  std::atomic<bool> start_flag{false};

  std::vector<std::thread> threads;
  threads.reserve(num_threads);

  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back([this, &success_count, &ready_count, &start_flag]() {
      ready_count++;

      // Wait for all threads to be ready
      while (!start_flag.load()) {
        std::this_thread::yield();
      }

      // All threads start searching at the same time
      TestSearchContext context;
      auto path = Dijkstra::Search(&graph_, context, TestState{0}, TestState{499});

      if (!path.empty()) {
        success_count++;
      }
    });
  }

  // Wait for all threads to be ready
  while (ready_count.load() < num_threads) {
    std::this_thread::yield();
  }

  // Start all threads simultaneously
  start_flag.store(true);

  // Join all threads
  for (auto& thread : threads) {
    thread.join();
  }

  EXPECT_EQ(success_count.load(), num_threads)
      << "All high-contention searches should succeed";
}

TEST_F(ConcurrentStressTest, LongRunningConcurrentSearches) {
  // Test sustained concurrent load
  const int num_threads = 20;
  const int duration_seconds = 2;
  std::atomic<int> total_searches{0};
  std::atomic<int> failed_searches{0};
  std::atomic<bool> stop_flag{false};

  std::vector<std::thread> threads;
  threads.reserve(num_threads);

  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back([this, &total_searches, &failed_searches, &stop_flag]() {
      while (!stop_flag.load()) {
        TestSearchContext context;
        auto path = Dijkstra::Search(&graph_, context, TestState{0}, TestState{499});

        total_searches++;
        if (path.empty()) {
          failed_searches++;
        }
      }
    });
  }

  // Let searches run for specified duration
  std::this_thread::sleep_for(std::chrono::seconds(duration_seconds));
  stop_flag.store(true);

  // Join all threads
  for (auto& thread : threads) {
    thread.join();
  }

  EXPECT_GT(total_searches.load(), 0) << "Some searches should have completed";
  EXPECT_EQ(failed_searches.load(), 0) << "No searches should have failed";

  RecordProperty("total_searches", total_searches.load());
  RecordProperty("searches_per_second", total_searches.load() / duration_seconds);
}
