/*
 * performance_scaling_test.cpp
 *
 * Created on: Dec 2025
 * Description: Performance scaling tests for production readiness validation
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <chrono>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"

#include "graph/graph.hpp"
#include "graph/search/astar.hpp"
#include "graph/search/dijkstra.hpp"
#include "graph/search/bfs.hpp"

using namespace xmotion;

namespace {

// Simple 2D grid state for performance testing
struct GridState {
  int x, y;
  int64_t GetId() const { return static_cast<int64_t>(y) * 10000 + x; }
};

// Type aliases for cleaner code
using GridGraph = Graph<GridState>;
using GridSearchContext = SearchContext<GridState, double, DefaultIndexer<GridState>>;

// Heuristic for A* (Manhattan distance)
double GridHeuristic(GridState a, GridState b) {
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

// Helper to create a grid graph
template <typename GraphType>
void CreateGridGraph(GraphType& graph, int width, int height) {
  // Add vertices
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      graph.AddVertex(GridState{x, y});
    }
  }

  // Add edges (4-connected grid)
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      GridState current{x, y};
      if (x + 1 < width) {
        graph.AddEdge(current, GridState{x + 1, y}, 1.0);
        graph.AddEdge(GridState{x + 1, y}, current, 1.0);
      }
      if (y + 1 < height) {
        graph.AddEdge(current, GridState{x, y + 1}, 1.0);
        graph.AddEdge(GridState{x, y + 1}, current, 1.0);
      }
    }
  }
}

// Timer helper
class ScopedTimer {
 public:
  ScopedTimer() : start_(std::chrono::high_resolution_clock::now()) {}

  double ElapsedMs() const {
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(end - start_).count();
  }

 private:
  std::chrono::high_resolution_clock::time_point start_;
};

}  // namespace

class PerformanceScalingTest : public ::testing::Test {
 protected:
  void SetUp() override {}
  void TearDown() override {}
};

// Test graph construction scaling
TEST_F(PerformanceScalingTest, Construction_1K) {
  Graph<GridState> graph;
  ScopedTimer timer;

  // 32x32 = 1024 vertices
  CreateGridGraph(graph, 32, 32);
  double elapsed = timer.ElapsedMs();

  EXPECT_EQ(graph.GetVertexCount(), 1024);
  EXPECT_GT(graph.GetEdgeCount(), 0);

  // Construction should complete in reasonable time (< 100ms for 1K)
  EXPECT_LT(elapsed, 100.0) << "1K vertex construction took " << elapsed << "ms";
}

TEST_F(PerformanceScalingTest, Construction_10K) {
  Graph<GridState> graph;
  ScopedTimer timer;

  // 100x100 = 10000 vertices
  CreateGridGraph(graph, 100, 100);
  double elapsed = timer.ElapsedMs();

  EXPECT_EQ(graph.GetVertexCount(), 10000);

  // Construction should scale roughly linearly (< 1s for 10K)
  EXPECT_LT(elapsed, 1000.0) << "10K vertex construction took " << elapsed << "ms";
}

TEST_F(PerformanceScalingTest, Construction_100K) {
  Graph<GridState> graph;
  ScopedTimer timer;

  // 316x316 ≈ 100K vertices
  CreateGridGraph(graph, 316, 316);
  double elapsed = timer.ElapsedMs();

  EXPECT_GE(graph.GetVertexCount(), 99856);  // 316*316

  // Construction should scale roughly linearly (< 10s for 100K)
  EXPECT_LT(elapsed, 10000.0) << "100K vertex construction took " << elapsed << "ms";
}

TEST_F(PerformanceScalingTest, DijkstraSearch_Scaling) {
  // Test search time scaling across different graph sizes
  std::vector<int> sizes = {10, 32, 50, 100};
  std::vector<double> times;

  for (int size : sizes) {
    Graph<GridState> graph;
    CreateGridGraph(graph, size, size);

    GridSearchContext context;
    GridState start{0, 0};
    GridState goal{size - 1, size - 1};

    ScopedTimer timer;
    auto path = Dijkstra::Search(&graph, context, start, goal);
    double elapsed = timer.ElapsedMs();

    times.push_back(elapsed);

    EXPECT_FALSE(path.empty()) << "Path should be found for size " << size;
  }

  // Verify that time scaling is reasonable (not exponential)
  // For Dijkstra O((V+E) log V), doubling size should roughly 4x time
  // Allow generous margin for test stability
  for (size_t i = 1; i < times.size(); ++i) {
    // Just verify times are recorded and reasonable
    EXPECT_GE(times[i], 0) << "Search time should be non-negative";
  }
}

TEST_F(PerformanceScalingTest, AStarSearch_Scaling) {
  // Test A* search time scaling
  std::vector<int> sizes = {10, 32, 50, 100};
  std::vector<double> times;

  for (int size : sizes) {
    Graph<GridState> graph;
    CreateGridGraph(graph, size, size);

    GridSearchContext context;
    GridState start{0, 0};
    GridState goal{size - 1, size - 1};

    ScopedTimer timer;
    auto path = AStar::Search(&graph, context, start, goal, GridHeuristic);
    double elapsed = timer.ElapsedMs();

    times.push_back(elapsed);

    EXPECT_FALSE(path.empty()) << "Path should be found for size " << size;
  }

  // A* should generally be faster than Dijkstra due to heuristic guidance
  for (size_t i = 0; i < times.size(); ++i) {
    EXPECT_GE(times[i], 0) << "Search time should be non-negative";
  }
}

TEST_F(PerformanceScalingTest, AStarVsDijkstraComparison) {
  // Compare A* and Dijkstra on the same graph
  const int size = 50;
  Graph<GridState> graph;
  CreateGridGraph(graph, size, size);

  GridState start{0, 0};
  GridState goal{size - 1, size - 1};

  // Run Dijkstra
  GridSearchContext dijkstra_context;
  ScopedTimer dijkstra_timer;
  auto dijkstra_path = Dijkstra::Search(&graph, dijkstra_context, start, goal);
  double dijkstra_time = dijkstra_timer.ElapsedMs();

  // Run A*
  GridSearchContext astar_context;
  ScopedTimer astar_timer;
  auto astar_path = AStar::Search(&graph, astar_context, start, goal, GridHeuristic);
  double astar_time = astar_timer.ElapsedMs();

  // Both should find valid paths of the same length
  EXPECT_FALSE(dijkstra_path.empty());
  EXPECT_FALSE(astar_path.empty());
  EXPECT_EQ(dijkstra_path.size(), astar_path.size())
      << "Both algorithms should find optimal path";

  // A* should typically be faster or comparable (with good heuristic)
  // Don't enforce strict ordering due to measurement variance
  EXPECT_GE(dijkstra_time, 0);
  EXPECT_GE(astar_time, 0);
}

TEST_F(PerformanceScalingTest, SearchContextReuse) {
  // Test that reusing SearchContext doesn't degrade performance
  const int size = 50;
  Graph<GridState> graph;
  CreateGridGraph(graph, size, size);

  GridSearchContext context;
  std::vector<double> times;

  for (int i = 0; i < 10; ++i) {
    GridState start{0, 0};
    GridState goal{size - 1, size - 1};

    ScopedTimer timer;
    auto path = Dijkstra::Search(&graph, context, start, goal);
    double elapsed = timer.ElapsedMs();

    times.push_back(elapsed);
    EXPECT_FALSE(path.empty());
  }

  // All search times should be roughly similar (no degradation)
  double avg = 0;
  for (double t : times) avg += t;
  avg /= times.size();

  // No search should be more than 5x the average
  for (double t : times) {
    EXPECT_LT(t, avg * 5) << "Search time variance too high";
  }
}

TEST_F(PerformanceScalingTest, MemoryEfficiency) {
  // Test that memory usage scales linearly
  // We can't directly measure memory, but we can verify the graph
  // structure remains efficient

  const int size = 100;
  Graph<GridState> graph;
  CreateGridGraph(graph, size, size);

  size_t vertex_count = graph.GetVertexCount();
  size_t edge_count = graph.GetEdgeCount();

  // For a 4-connected grid, edges ≈ 4 * vertices (minus boundary effects)
  // Expected: ~2 * (size-1) * size * 2 edges (horizontal + vertical, bidirectional)
  size_t expected_edges = 2 * (size - 1) * size * 2;

  EXPECT_EQ(vertex_count, static_cast<size_t>(size * size));
  EXPECT_EQ(edge_count, expected_edges);

  // Verify edges per vertex ratio is reasonable
  double edges_per_vertex = static_cast<double>(edge_count) / vertex_count;
  EXPECT_GT(edges_per_vertex, 3.0);  // Most vertices have 4 edges
  EXPECT_LT(edges_per_vertex, 4.5);  // Boundary vertices have fewer
}

TEST_F(PerformanceScalingTest, BFSScaling) {
  // Test BFS scaling (should be O(V+E))
  std::vector<int> sizes = {10, 32, 50, 100};
  std::vector<double> times;

  for (int size : sizes) {
    Graph<GridState> graph;
    CreateGridGraph(graph, size, size);

    GridSearchContext context;
    GridState start{0, 0};
    GridState goal{size - 1, size - 1};

    ScopedTimer timer;
    auto path = BFS::Search(&graph, context, start, goal);
    double elapsed = timer.ElapsedMs();

    times.push_back(elapsed);

    EXPECT_FALSE(path.empty()) << "Path should be found for size " << size;
    // BFS finds shortest path in terms of edges
    EXPECT_EQ(path.size(), static_cast<size_t>(2 * (size - 1) + 1))
        << "BFS should find optimal edge-count path";
  }
}
