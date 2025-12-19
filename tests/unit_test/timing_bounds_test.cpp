/*
 * timing_bounds_test.cpp
 *
 * Created on: Dec 2025
 * Description: Timing bounds and latency distribution tests for production readiness
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <vector>

#include "gtest/gtest.h"

#include "graph/graph.hpp"
#include "graph/search/astar.hpp"
#include "graph/search/dijkstra.hpp"
#include "graph/search/bfs.hpp"
#include "graph/search/dfs.hpp"

using namespace xmotion;

namespace {

// Simple state for testing
struct TestState {
  int64_t id;
  int64_t GetId() const { return id; }
};

// Type aliases for cleaner code
using TestGraph = Graph<TestState>;
using TestSearchContext = SearchContext<TestState, double, DefaultIndexer<TestState>>;

// Heuristic that returns 0 (makes A* behave like Dijkstra)
double ZeroHeuristic(TestState a, TestState b) { return 0.0; }

// Timer helper
class ScopedTimer {
 public:
  ScopedTimer() : start_(std::chrono::high_resolution_clock::now()) {}

  double ElapsedUs() const {
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::micro>(end - start_).count();
  }

  double ElapsedMs() const { return ElapsedUs() / 1000.0; }

 private:
  std::chrono::high_resolution_clock::time_point start_;
};

// Percentile calculation helper
double Percentile(std::vector<double>& data, double p) {
  if (data.empty()) return 0;
  std::sort(data.begin(), data.end());
  size_t idx = static_cast<size_t>(p * (data.size() - 1));
  return data[idx];
}

}  // namespace

class TimingBoundsTest : public ::testing::Test {
 protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(TimingBoundsTest, SearchLatency_Distribution) {
  // Create a moderate-sized graph for latency testing
  Graph<TestState> graph;
  const int num_vertices = 1000;

  // Create a graph with random-ish connectivity
  for (int i = 0; i < num_vertices; ++i) {
    graph.AddVertex(TestState{i});
  }

  // Add edges to create a connected graph (chain with shortcuts)
  for (int i = 0; i < num_vertices - 1; ++i) {
    graph.AddEdge(TestState{i}, TestState{i + 1}, 1.0);
    graph.AddEdge(TestState{i + 1}, TestState{i}, 1.0);

    // Add some shortcuts
    if (i + 10 < num_vertices) {
      graph.AddEdge(TestState{i}, TestState{i + 10}, 5.0);
      graph.AddEdge(TestState{i + 10}, TestState{i}, 5.0);
    }
  }

  // Run multiple searches and collect timing data
  std::vector<double> latencies;
  const int num_runs = 100;

  for (int i = 0; i < num_runs; ++i) {
    TestSearchContext context;
    TestState start{0};
    TestState goal{num_vertices - 1};

    ScopedTimer timer;
    auto path = Dijkstra::Search(&graph, context, start, goal);
    double elapsed = timer.ElapsedUs();

    latencies.push_back(elapsed);
    ASSERT_FALSE(path.empty()) << "Path should be found on run " << i;
  }

  // Calculate percentiles
  double p50 = Percentile(latencies, 0.50);
  double p90 = Percentile(latencies, 0.90);
  double p99 = Percentile(latencies, 0.99);

  // Log results for analysis
  double sum = std::accumulate(latencies.begin(), latencies.end(), 0.0);
  double avg = sum / latencies.size();

  // Verify reasonable latency bounds
  // P99 should not be more than 10x P50 (indicates consistent performance)
  EXPECT_LT(p99, p50 * 10) << "P99/P50 ratio too high, indicating inconsistent latency";

  // All searches should complete in reasonable time (< 100ms)
  EXPECT_LT(p99, 100000) << "P99 latency exceeds 100ms";

  // Record stats for informational purposes
  RecordProperty("avg_latency_us", avg);
  RecordProperty("p50_latency_us", p50);
  RecordProperty("p90_latency_us", p90);
  RecordProperty("p99_latency_us", p99);
}

TEST_F(TimingBoundsTest, AdversarialGraph_LongChain) {
  // Test worst-case scenario: long chain graph
  // This is adversarial for search algorithms as there's only one path
  Graph<TestState> graph;
  const int chain_length = 5000;

  for (int i = 0; i < chain_length; ++i) {
    graph.AddVertex(TestState{i});
  }

  for (int i = 0; i < chain_length - 1; ++i) {
    graph.AddEdge(TestState{i}, TestState{i + 1}, 1.0);
  }

  TestSearchContext context;
  TestState start{0};
  TestState goal{chain_length - 1};

  ScopedTimer timer;
  auto path = Dijkstra::Search(&graph, context, start, goal);
  double elapsed = timer.ElapsedMs();

  EXPECT_FALSE(path.empty());
  EXPECT_EQ(path.size(), static_cast<size_t>(chain_length));

  // Long chain should still complete in reasonable time
  EXPECT_LT(elapsed, 1000.0) << "Long chain search took " << elapsed << "ms";

  RecordProperty("chain_length", chain_length);
  RecordProperty("search_time_ms", elapsed);
}

TEST_F(TimingBoundsTest, AdversarialGraph_DenseCluster) {
  // Test dense graph scenario: fully connected cluster
  // This maximizes edge count relative to vertices
  Graph<TestState> graph;
  const int cluster_size = 200;  // 200 * 199 = 39800 edges

  for (int i = 0; i < cluster_size; ++i) {
    graph.AddVertex(TestState{i});
  }

  // Create fully connected graph
  for (int i = 0; i < cluster_size; ++i) {
    for (int j = i + 1; j < cluster_size; ++j) {
      double cost = static_cast<double>(std::abs(i - j));
      graph.AddEdge(TestState{i}, TestState{j}, cost);
      graph.AddEdge(TestState{j}, TestState{i}, cost);
    }
  }

  TestSearchContext context;
  TestState start{0};
  TestState goal{cluster_size - 1};

  ScopedTimer timer;
  auto path = Dijkstra::Search(&graph, context, start, goal);
  double elapsed = timer.ElapsedMs();

  EXPECT_FALSE(path.empty());
  // Direct path should be found (cost = cluster_size - 1)
  EXPECT_LE(path.size(), static_cast<size_t>(cluster_size));

  // Dense graph should still complete in reasonable time
  EXPECT_LT(elapsed, 5000.0) << "Dense cluster search took " << elapsed << "ms";

  RecordProperty("cluster_size", cluster_size);
  RecordProperty("edge_count", graph.GetEdgeCount());
  RecordProperty("search_time_ms", elapsed);
}

TEST_F(TimingBoundsTest, AdversarialGraph_StarTopology) {
  // Test star topology: one hub connected to all others
  // This creates many equal-cost paths
  Graph<TestState> graph;
  const int num_leaves = 1000;

  // Hub is vertex 0
  graph.AddVertex(TestState{0});

  for (int i = 1; i <= num_leaves; ++i) {
    graph.AddVertex(TestState{i});
    graph.AddEdge(TestState{0}, TestState{i}, 1.0);
    graph.AddEdge(TestState{i}, TestState{0}, 1.0);
  }

  // Search from one leaf to another (must go through hub)
  TestSearchContext context;
  TestState start{1};
  TestState goal{num_leaves};

  ScopedTimer timer;
  auto path = Dijkstra::Search(&graph, context, start, goal);
  double elapsed = timer.ElapsedMs();

  EXPECT_FALSE(path.empty());
  EXPECT_EQ(path.size(), 3u);  // leaf -> hub -> leaf

  // Star topology should be fast (short path)
  EXPECT_LT(elapsed, 100.0) << "Star topology search took " << elapsed << "ms";

  RecordProperty("num_leaves", num_leaves);
  RecordProperty("search_time_ms", elapsed);
}

TEST_F(TimingBoundsTest, WorstCaseHeuristic) {
  // Test A* with a pessimistic heuristic
  // Zero heuristic makes A* behave like Dijkstra
  Graph<TestState> graph;
  const int size = 500;

  for (int i = 0; i < size; ++i) {
    graph.AddVertex(TestState{i});
  }

  for (int i = 0; i < size - 1; ++i) {
    graph.AddEdge(TestState{i}, TestState{i + 1}, 1.0);
    graph.AddEdge(TestState{i + 1}, TestState{i}, 1.0);
  }

  TestSearchContext context;
  TestState start{0};
  TestState goal{size - 1};

  ScopedTimer timer;
  auto path = AStar::Search(&graph, context, start, goal, ZeroHeuristic);
  double elapsed = timer.ElapsedMs();

  EXPECT_FALSE(path.empty());
  EXPECT_EQ(path.size(), static_cast<size_t>(size));

  // Should still complete in reasonable time
  EXPECT_LT(elapsed, 500.0) << "A* with zero heuristic took " << elapsed << "ms";
}

TEST_F(TimingBoundsTest, MultipleAlgorithmsComparison) {
  // Compare timing of different algorithms on the same graph
  Graph<TestState> graph;
  const int size = 500;

  for (int i = 0; i < size; ++i) {
    graph.AddVertex(TestState{i});
  }

  for (int i = 0; i < size - 1; ++i) {
    graph.AddEdge(TestState{i}, TestState{i + 1}, 1.0);
    graph.AddEdge(TestState{i + 1}, TestState{i}, 1.0);
    if (i + 5 < size) {
      graph.AddEdge(TestState{i}, TestState{i + 5}, 3.0);
      graph.AddEdge(TestState{i + 5}, TestState{i}, 3.0);
    }
  }

  TestState start{0};
  TestState goal{size - 1};

  // Dijkstra
  {
    TestSearchContext context;
    ScopedTimer timer;
    auto path = Dijkstra::Search(&graph, context, start, goal);
    double elapsed = timer.ElapsedMs();
    EXPECT_FALSE(path.empty());
    RecordProperty("dijkstra_time_ms", elapsed);
  }

  // A*
  {
    TestSearchContext context;
    ScopedTimer timer;
    auto path = AStar::Search(&graph, context, start, goal, ZeroHeuristic);
    double elapsed = timer.ElapsedMs();
    EXPECT_FALSE(path.empty());
    RecordProperty("astar_time_ms", elapsed);
  }

  // BFS
  {
    TestSearchContext context;
    ScopedTimer timer;
    auto path = BFS::Search(&graph, context, start, goal);
    double elapsed = timer.ElapsedMs();
    EXPECT_FALSE(path.empty());
    RecordProperty("bfs_time_ms", elapsed);
  }

  // DFS
  {
    TestSearchContext context;
    ScopedTimer timer;
    auto path = DFS::Search(&graph, context, start, goal);
    double elapsed = timer.ElapsedMs();
    EXPECT_FALSE(path.empty());
    RecordProperty("dfs_time_ms", elapsed);
  }
}

TEST_F(TimingBoundsTest, EmptyGraphSearch) {
  // Verify fast failure on empty graph
  Graph<TestState> graph;

  TestSearchContext context;
  TestState start{0};
  TestState goal{1};

  ScopedTimer timer;
  auto path = Dijkstra::Search(&graph, context, start, goal);
  double elapsed = timer.ElapsedUs();

  EXPECT_TRUE(path.empty());
  // Empty graph search should be nearly instantaneous
  EXPECT_LT(elapsed, 1000.0) << "Empty graph search took " << elapsed << "us";
}

TEST_F(TimingBoundsTest, NoPathExists) {
  // Verify fast failure when no path exists
  Graph<TestState> graph;
  const int size = 100;

  // Create two disconnected components
  for (int i = 0; i < size; ++i) {
    graph.AddVertex(TestState{i});
    graph.AddVertex(TestState{i + size});
  }

  for (int i = 0; i < size - 1; ++i) {
    graph.AddEdge(TestState{i}, TestState{i + 1}, 1.0);
    graph.AddEdge(TestState{i + size}, TestState{i + size + 1}, 1.0);
  }

  TestSearchContext context;
  TestState start{0};
  TestState goal{size + size - 1};  // In other component

  ScopedTimer timer;
  auto path = Dijkstra::Search(&graph, context, start, goal);
  double elapsed = timer.ElapsedMs();

  EXPECT_TRUE(path.empty());
  // Should explore first component and return, not hang
  EXPECT_LT(elapsed, 100.0) << "No-path search took " << elapsed << "ms";
}
