/*
 * determinism_test.cpp
 *
 * Created on: Dec 2025
 * Description: Determinism validation tests for production readiness
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <algorithm>
#include <random>
#include <set>
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

class DeterminismTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a standard test graph
    const int size = 100;
    for (int i = 0; i < size; ++i) {
      graph_.AddVertex(TestState{i});
    }

    // Create edges with varying costs
    for (int i = 0; i < size - 1; ++i) {
      graph_.AddEdge(TestState{i}, TestState{i + 1}, 1.0);
      graph_.AddEdge(TestState{i + 1}, TestState{i}, 1.0);

      if (i + 3 < size) {
        graph_.AddEdge(TestState{i}, TestState{i + 3}, 2.5);
        graph_.AddEdge(TestState{i + 3}, TestState{i}, 2.5);
      }

      if (i + 7 < size) {
        graph_.AddEdge(TestState{i}, TestState{i + 7}, 5.0);
        graph_.AddEdge(TestState{i + 7}, TestState{i}, 5.0);
      }
    }
  }

  Graph<TestState> graph_;
};

TEST_F(DeterminismTest, IdenticalPaths_Dijkstra_1000Runs) {
  TestState start{0};
  TestState goal{99};

  // Run first search to get reference path
  TestSearchContext ref_context;
  auto ref_path = Dijkstra::Search(&graph_, ref_context, start, goal);
  ASSERT_FALSE(ref_path.empty()) << "Reference path should be found";
  auto ref_ids = PathToIds(ref_path);

  // Run 1000 searches and verify identical results
  for (int i = 0; i < 1000; ++i) {
    TestSearchContext context;
    auto path = Dijkstra::Search(&graph_, context, start, goal);

    ASSERT_FALSE(path.empty()) << "Path should be found on run " << i;
    auto ids = PathToIds(path);

    EXPECT_EQ(ids, ref_ids) << "Path differs on run " << i;
  }
}

TEST_F(DeterminismTest, IdenticalPaths_AStar_1000Runs) {
  TestState start{0};
  TestState goal{99};

  // Run first search to get reference path
  TestSearchContext ref_context;
  auto ref_path = AStar::Search(&graph_, ref_context, start, goal, SimpleHeuristic);
  ASSERT_FALSE(ref_path.empty()) << "Reference path should be found";
  auto ref_ids = PathToIds(ref_path);

  // Run 1000 searches and verify identical results
  for (int i = 0; i < 1000; ++i) {
    TestSearchContext context;
    auto path = AStar::Search(&graph_, context, start, goal, SimpleHeuristic);

    ASSERT_FALSE(path.empty()) << "Path should be found on run " << i;
    auto ids = PathToIds(path);

    EXPECT_EQ(ids, ref_ids) << "Path differs on run " << i;
  }
}

TEST_F(DeterminismTest, IdenticalPaths_BFS_1000Runs) {
  TestState start{0};
  TestState goal{99};

  // Run first search to get reference path
  TestSearchContext ref_context;
  auto ref_path = BFS::Search(&graph_, ref_context, start, goal);
  ASSERT_FALSE(ref_path.empty()) << "Reference path should be found";
  auto ref_ids = PathToIds(ref_path);

  // Run 1000 searches and verify identical results
  for (int i = 0; i < 1000; ++i) {
    TestSearchContext context;
    auto path = BFS::Search(&graph_, context, start, goal);

    ASSERT_FALSE(path.empty()) << "Path should be found on run " << i;
    auto ids = PathToIds(path);

    EXPECT_EQ(ids, ref_ids) << "Path differs on run " << i;
  }
}

TEST_F(DeterminismTest, ContextReusePreservesDeterminism) {
  // Verify that reusing a SearchContext produces identical results
  TestState start{0};
  TestState goal{99};

  TestSearchContext context;

  // Run multiple searches with same context
  std::vector<std::vector<int64_t>> all_paths;
  for (int i = 0; i < 100; ++i) {
    auto path = Dijkstra::Search(&graph_, context, start, goal);
    ASSERT_FALSE(path.empty()) << "Path should be found on run " << i;
    all_paths.push_back(PathToIds(path));
  }

  // All paths should be identical
  for (size_t i = 1; i < all_paths.size(); ++i) {
    EXPECT_EQ(all_paths[i], all_paths[0])
        << "Path differs on context reuse run " << i;
  }
}

TEST_F(DeterminismTest, DifferentStartGoalPairs) {
  // Verify determinism across different search queries
  std::vector<std::pair<int64_t, int64_t>> queries = {
      {0, 99}, {10, 90}, {25, 75}, {0, 50}, {50, 99}};

  for (const auto& query : queries) {
    TestState start{query.first};
    TestState goal{query.second};

    // Get reference path
    TestSearchContext ref_context;
    auto ref_path = Dijkstra::Search(&graph_, ref_context, start, goal);
    ASSERT_FALSE(ref_path.empty());
    auto ref_ids = PathToIds(ref_path);

    // Run 100 times and verify
    for (int i = 0; i < 100; ++i) {
      TestSearchContext context;
      auto path = Dijkstra::Search(&graph_, context, start, goal);
      auto ids = PathToIds(path);

      EXPECT_EQ(ids, ref_ids) << "Path differs for query (" << query.first
                              << ", " << query.second << ") on run " << i;
    }
  }
}

TEST_F(DeterminismTest, ConstructionOrderIndependence) {
  // Create two graphs with same edges added in different orders
  // Verify that searches produce equivalent paths

  Graph<TestState> graph1;
  Graph<TestState> graph2;

  // Add vertices to both graphs
  for (int i = 0; i < 50; ++i) {
    graph1.AddVertex(TestState{i});
    graph2.AddVertex(TestState{i});
  }

  // Store edges to add
  std::vector<std::tuple<int64_t, int64_t, double>> edges;
  for (int i = 0; i < 49; ++i) {
    edges.push_back({i, i + 1, 1.0});
    if (i + 5 < 50) {
      edges.push_back({i, i + 5, 3.0});
    }
  }

  // Add edges in forward order to graph1
  for (const auto& edge : edges) {
    graph1.AddEdge(TestState{std::get<0>(edge)}, TestState{std::get<1>(edge)},
                   std::get<2>(edge));
    graph1.AddEdge(TestState{std::get<1>(edge)}, TestState{std::get<0>(edge)},
                   std::get<2>(edge));
  }

  // Add edges in reverse order to graph2
  for (auto it = edges.rbegin(); it != edges.rend(); ++it) {
    graph2.AddEdge(TestState{std::get<0>(*it)}, TestState{std::get<1>(*it)},
                   std::get<2>(*it));
    graph2.AddEdge(TestState{std::get<1>(*it)}, TestState{std::get<0>(*it)},
                   std::get<2>(*it));
  }

  // Search both graphs
  TestState start{0};
  TestState goal{49};

  TestSearchContext context1;
  auto path1 = Dijkstra::Search(&graph1, context1, start, goal);

  TestSearchContext context2;
  auto path2 = Dijkstra::Search(&graph2, context2, start, goal);

  ASSERT_FALSE(path1.empty());
  ASSERT_FALSE(path2.empty());

  // Paths should have the same length (optimal)
  EXPECT_EQ(path1.size(), path2.size())
      << "Paths from differently-constructed graphs should have same length";

  // Start and end should be the same
  EXPECT_EQ(path1.front().id, path2.front().id);
  EXPECT_EQ(path1.back().id, path2.back().id);
}

TEST_F(DeterminismTest, HashIndependence) {
  // Verify that vertex ID ordering doesn't affect results
  // This tests for hash-map iteration order dependencies

  // Create graph with non-sequential IDs
  Graph<TestState> graph;
  std::vector<int64_t> ids = {100, 5, 42, 17, 88, 3, 71, 29, 56, 12};

  for (int64_t id : ids) {
    graph.AddVertex(TestState{id});
  }

  // Connect in order of the vector
  for (size_t i = 0; i < ids.size() - 1; ++i) {
    graph.AddEdge(TestState{ids[i]}, TestState{ids[i + 1]}, 1.0);
    graph.AddEdge(TestState{ids[i + 1]}, TestState{ids[i]}, 1.0);
  }

  TestState start{ids.front()};
  TestState goal{ids.back()};

  // Get reference path
  TestSearchContext ref_context;
  auto ref_path = Dijkstra::Search(&graph, ref_context, start, goal);
  ASSERT_FALSE(ref_path.empty());
  auto ref_ids = PathToIds(ref_path);

  // Run many times to detect hash-order dependencies
  for (int i = 0; i < 500; ++i) {
    TestSearchContext context;
    auto path = Dijkstra::Search(&graph, context, start, goal);
    auto path_ids = PathToIds(path);

    EXPECT_EQ(path_ids, ref_ids) << "Hash-order dependency detected on run " << i;
  }
}

TEST_F(DeterminismTest, OptimalPathCostConsistency) {
  // Verify that the path cost is consistent across runs
  TestState start{0};
  TestState goal{99};

  // Calculate reference path cost
  TestSearchContext ref_context;
  auto ref_path = Dijkstra::Search(&graph_, ref_context, start, goal);
  ASSERT_FALSE(ref_path.empty());

  // Calculate path cost from context
  auto& goal_info = ref_context.GetSearchInfo(goal.id);
  double ref_cost = goal_info.GetGCost<double>();

  // Verify cost is consistent across runs
  for (int i = 0; i < 100; ++i) {
    TestSearchContext context;
    auto path = Dijkstra::Search(&graph_, context, start, goal);
    ASSERT_FALSE(path.empty());

    auto& info = context.GetSearchInfo(goal.id);
    double cost = info.GetGCost<double>();

    EXPECT_DOUBLE_EQ(cost, ref_cost) << "Path cost differs on run " << i;
  }
}

TEST_F(DeterminismTest, AllAlgorithmsConsistentWithSelf) {
  // Verify that each algorithm is self-consistent
  TestState start{0};
  TestState goal{99};

  // Test Dijkstra
  {
    TestSearchContext ref_ctx;
    auto ref_path = Dijkstra::Search(&graph_, ref_ctx, start, goal);
    auto ref_ids = PathToIds(ref_path);

    for (int i = 0; i < 100; ++i) {
      TestSearchContext ctx;
      auto path = Dijkstra::Search(&graph_, ctx, start, goal);
      EXPECT_EQ(PathToIds(path), ref_ids) << "Dijkstra inconsistent on run " << i;
    }
  }

  // Test A*
  {
    TestSearchContext ref_ctx;
    auto ref_path = AStar::Search(&graph_, ref_ctx, start, goal, SimpleHeuristic);
    auto ref_ids = PathToIds(ref_path);

    for (int i = 0; i < 100; ++i) {
      TestSearchContext ctx;
      auto path = AStar::Search(&graph_, ctx, start, goal, SimpleHeuristic);
      EXPECT_EQ(PathToIds(path), ref_ids) << "A* inconsistent on run " << i;
    }
  }

  // Test BFS
  {
    TestSearchContext ref_ctx;
    auto ref_path = BFS::Search(&graph_, ref_ctx, start, goal);
    auto ref_ids = PathToIds(ref_path);

    for (int i = 0; i < 100; ++i) {
      TestSearchContext ctx;
      auto path = BFS::Search(&graph_, ctx, start, goal);
      EXPECT_EQ(PathToIds(path), ref_ids) << "BFS inconsistent on run " << i;
    }
  }

  // Test DFS
  {
    TestSearchContext ref_ctx;
    auto ref_path = DFS::Search(&graph_, ref_ctx, start, goal);
    auto ref_ids = PathToIds(ref_path);

    for (int i = 0; i < 100; ++i) {
      TestSearchContext ctx;
      auto path = DFS::Search(&graph_, ctx, start, goal);
      EXPECT_EQ(PathToIds(path), ref_ids) << "DFS inconsistent on run " << i;
    }
  }
}
