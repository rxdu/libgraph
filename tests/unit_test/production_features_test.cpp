/*
 * production_features_test.cpp
 *
 * Tests for production readiness features:
 * - PathResult with cost and metadata
 * - SearchLimits for early termination
 * - Multi-goal search
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <limits>
#include <cmath>

#include "graph/graph.hpp"
#include "graph/search/dijkstra.hpp"
#include "graph/search/astar.hpp"
#include "graph/search/search_context.hpp"

using namespace xmotion;

// Test state with coordinates
struct Point {
    int x, y;
    int64_t GetId() const { return static_cast<int64_t>(y * 1000 + x); }
    bool operator==(const Point& other) const { return x == other.x && y == other.y; }
};

class ProductionFeaturesTest : public ::testing::Test {
protected:
    using StateIndexer = DefaultIndexer<Point>;
    using GraphType = Graph<Point, double, StateIndexer>;
    using ContextType = SearchContext<Point, double, StateIndexer>;

    void SetUp() override {
        graph = std::make_shared<GraphType>();

        // Create a 5x5 grid graph
        // (0,0) -- (1,0) -- (2,0) -- (3,0) -- (4,0)
        //   |        |        |        |        |
        // (0,1) -- (1,1) -- (2,1) -- (3,1) -- (4,1)
        //   |        |        |        |        |
        // (0,2) -- (1,2) -- (2,2) -- (3,2) -- (4,2)
        //   |        |        |        |        |
        // (0,3) -- (1,3) -- (2,3) -- (3,3) -- (4,3)
        //   |        |        |        |        |
        // (0,4) -- (1,4) -- (2,4) -- (3,4) -- (4,4)

        for (int y = 0; y < 5; ++y) {
            for (int x = 0; x < 5; ++x) {
                graph->AddVertex({x, y});
            }
        }

        // Add horizontal edges (cost = 1.0)
        for (int y = 0; y < 5; ++y) {
            for (int x = 0; x < 4; ++x) {
                graph->AddEdge({x, y}, {x+1, y}, 1.0);
                graph->AddEdge({x+1, y}, {x, y}, 1.0);
            }
        }

        // Add vertical edges (cost = 1.0)
        for (int y = 0; y < 4; ++y) {
            for (int x = 0; x < 5; ++x) {
                graph->AddEdge({x, y}, {x, y+1}, 1.0);
                graph->AddEdge({x, y+1}, {x, y}, 1.0);
            }
        }
    }

    std::shared_ptr<GraphType> graph;
};

// ============================================================================
// PathResult Tests
// ============================================================================

TEST_F(ProductionFeaturesTest, PathResultContainsCorrectCost) {
    ContextType context;
    Point start{0, 0};
    Point goal{4, 4};

    auto strategy = MakeDijkstraStrategy<Point, double, StateIndexer>();
    auto start_it = graph->FindVertex(start);
    auto goal_it = graph->FindVertex(goal);

    auto result = SearchAlgorithm<decltype(strategy), Point, double, StateIndexer>
        ::SearchWithResult(graph.get(), context, start_it, goal_it, strategy);

    EXPECT_TRUE(result.found);
    EXPECT_FALSE(result.path.empty());
    EXPECT_DOUBLE_EQ(result.total_cost, 8.0);  // Manhattan distance: |4-0| + |4-0| = 8
    EXPECT_GT(result.nodes_expanded, 0u);
}

TEST_F(ProductionFeaturesTest, PathResultForUnreachableGoal) {
    // Create isolated vertex
    auto isolated_graph = std::make_shared<GraphType>();
    isolated_graph->AddVertex({0, 0});
    isolated_graph->AddVertex({5, 5});  // No edges to this vertex

    ContextType context;
    auto strategy = MakeDijkstraStrategy<Point, double, StateIndexer>();
    auto start_it = isolated_graph->FindVertex(Point{0, 0});
    auto goal_it = isolated_graph->FindVertex(Point{5, 5});

    auto result = SearchAlgorithm<decltype(strategy), Point, double, StateIndexer>
        ::SearchWithResult(isolated_graph.get(), context, start_it, goal_it, strategy);

    EXPECT_FALSE(result.found);
    EXPECT_TRUE(result.path.empty());
}

TEST_F(ProductionFeaturesTest, PathResultBoolConversion) {
    PathResult<Point> success_result;
    success_result.found = true;
    success_result.path = {{0, 0}, {1, 0}};

    PathResult<Point> fail_result;
    fail_result.found = false;

    EXPECT_TRUE(static_cast<bool>(success_result));
    EXPECT_FALSE(static_cast<bool>(fail_result));
}

// ============================================================================
// SearchLimits Tests
// ============================================================================

TEST_F(ProductionFeaturesTest, SearchLimitsFactoryMethods) {
    auto unlimited = SearchLimits::Unlimited();
    EXPECT_FALSE(unlimited.HasLimits());
    EXPECT_EQ(unlimited.max_expansions, 0u);
    EXPECT_EQ(unlimited.timeout_ms, 0u);

    auto max_exp = SearchLimits::MaxExpansions(100);
    EXPECT_TRUE(max_exp.HasLimits());
    EXPECT_EQ(max_exp.max_expansions, 100u);
    EXPECT_EQ(max_exp.timeout_ms, 0u);

    auto timeout = SearchLimits::Timeout(500);
    EXPECT_TRUE(timeout.HasLimits());
    EXPECT_EQ(timeout.max_expansions, 0u);
    EXPECT_EQ(timeout.timeout_ms, 500u);
}

TEST_F(ProductionFeaturesTest, SearchWithMaxExpansionsLimit) {
    ContextType context;
    Point start{0, 0};
    Point goal{4, 4};

    auto strategy = MakeDijkstraStrategy<Point, double, StateIndexer>();
    auto start_it = graph->FindVertex(start);
    auto goal_it = graph->FindVertex(goal);

    // Limit expansions to just 3 - not enough to find path
    auto limits = SearchLimits::MaxExpansions(3);

    auto result = SearchAlgorithm<decltype(strategy), Point, double, StateIndexer>
        ::SearchWithLimits(graph.get(), context, start_it, goal_it, strategy, limits);

    // With only 3 expansions, we shouldn't find the full path
    EXPECT_FALSE(result.found);
    EXPECT_LE(result.nodes_expanded, 3u);
}

TEST_F(ProductionFeaturesTest, SearchWithUnlimitedFindsPath) {
    ContextType context;
    Point start{0, 0};
    Point goal{4, 4};

    auto strategy = MakeDijkstraStrategy<Point, double, StateIndexer>();
    auto start_it = graph->FindVertex(start);
    auto goal_it = graph->FindVertex(goal);

    // No limits
    auto limits = SearchLimits::Unlimited();

    auto result = SearchAlgorithm<decltype(strategy), Point, double, StateIndexer>
        ::SearchWithLimits(graph.get(), context, start_it, goal_it, strategy, limits);

    EXPECT_TRUE(result.found);
    EXPECT_FALSE(result.path.empty());
    EXPECT_DOUBLE_EQ(result.total_cost, 8.0);
}

// ============================================================================
// Multi-Goal Search Tests
// ============================================================================

TEST_F(ProductionFeaturesTest, MultiGoalSearchFindsNearestGoal) {
    ContextType context;
    Point start{0, 0};

    // Multiple goals at different distances
    std::vector<Point> goals = {
        {4, 4},  // Distance 8 (furthest)
        {2, 0},  // Distance 2 (nearest)
        {0, 3},  // Distance 3
    };

    auto result = Dijkstra::SearchMultiGoal(graph.get(), context, start, goals);

    EXPECT_TRUE(result.found);
    EXPECT_FALSE(result.path.empty());
    EXPECT_DOUBLE_EQ(result.total_cost, 2.0);  // Should find path to {2,0}
    EXPECT_EQ(result.goal_index, 1u);  // Index of {2, 0} in goals vector
    Point expected_goal{2, 0};
    EXPECT_EQ(result.goal_vertex_id, expected_goal.GetId());
}

TEST_F(ProductionFeaturesTest, MultiGoalSearchStartIsGoal) {
    ContextType context;
    Point start{0, 0};

    std::vector<Point> goals = {
        {0, 0},  // Start is a goal
        {4, 4},
    };

    auto result = Dijkstra::SearchMultiGoal(graph.get(), context, start, goals);

    EXPECT_TRUE(result.found);
    EXPECT_EQ(result.goal_index, 0u);
    EXPECT_DOUBLE_EQ(result.total_cost, 0.0);
}

TEST_F(ProductionFeaturesTest, MultiGoalSearchNoValidGoals) {
    ContextType context;
    Point start{0, 0};

    // Goals that don't exist in graph
    std::vector<Point> goals = {
        {10, 10},
        {20, 20},
    };

    auto result = Dijkstra::SearchMultiGoal(graph.get(), context, start, goals);

    EXPECT_FALSE(result.found);
    EXPECT_TRUE(result.path.empty());
}

TEST_F(ProductionFeaturesTest, MultiGoalSearchEmptyGoals) {
    ContextType context;
    Point start{0, 0};

    std::vector<Point> goals;  // Empty

    auto result = Dijkstra::SearchMultiGoal(graph.get(), context, start, goals);

    EXPECT_FALSE(result.found);
}

TEST_F(ProductionFeaturesTest, MultiGoalResultBoolConversion) {
    MultiGoalResult<Point> success_result;
    success_result.found = true;
    success_result.goal_vertex_id = 1;

    MultiGoalResult<Point> fail_result;
    fail_result.found = false;

    EXPECT_TRUE(static_cast<bool>(success_result));
    EXPECT_FALSE(static_cast<bool>(fail_result));
}

TEST_F(ProductionFeaturesTest, MultiGoalWithAStarHeuristic) {
    ContextType context;
    Point start{0, 0};

    std::vector<Point> goals = {
        {4, 4},
        {2, 0},  // Nearest
        {0, 3},
    };

    // For multi-goal A*, use minimum heuristic to any goal
    auto min_heuristic = [&goals](const Point& current, const Point& /*single_goal*/) {
        double min_h = std::numeric_limits<double>::max();
        for (const auto& g : goals) {
            double h = std::abs(current.x - g.x) + std::abs(current.y - g.y);
            min_h = std::min(min_h, h);
        }
        return min_h;
    };

    auto result = AStar::SearchMultiGoal(graph.get(), context, start, goals, min_heuristic);

    EXPECT_TRUE(result.found);
    EXPECT_DOUBLE_EQ(result.total_cost, 2.0);  // Nearest is {2,0}
    EXPECT_EQ(result.goal_index, 1u);
}

// ============================================================================
// Nodes Expanded Counter Tests
// ============================================================================

TEST_F(ProductionFeaturesTest, NodesExpandedTracking) {
    ContextType context;
    Point start{0, 0};
    Point goal{2, 2};

    // Initial state
    EXPECT_EQ(context.GetNodesExpanded(), 0u);

    auto path = Dijkstra::Search(graph.get(), context, start, goal);

    // After search, some nodes should have been expanded
    EXPECT_GT(context.GetNodesExpanded(), 0u);

    // Record count
    size_t first_search_expansions = context.GetNodesExpanded();

    // After Reset, counter should be back to 0
    context.Reset();
    EXPECT_EQ(context.GetNodesExpanded(), 0u);

    // Second search
    path = Dijkstra::Search(graph.get(), context, start, goal);

    // Should have expanded same number of nodes
    EXPECT_EQ(context.GetNodesExpanded(), first_search_expansions);
}

TEST_F(ProductionFeaturesTest, NodesExpandedInResult) {
    ContextType context;
    Point start{0, 0};
    Point goal{1, 1};

    auto strategy = MakeDijkstraStrategy<Point, double, StateIndexer>();
    auto start_it = graph->FindVertex(start);
    auto goal_it = graph->FindVertex(goal);

    auto result = SearchAlgorithm<decltype(strategy), Point, double, StateIndexer>
        ::SearchWithResult(graph.get(), context, start_it, goal_it, strategy);

    EXPECT_TRUE(result.found);
    EXPECT_GT(result.nodes_expanded, 0u);
    EXPECT_EQ(result.nodes_expanded, context.GetNodesExpanded());
}
