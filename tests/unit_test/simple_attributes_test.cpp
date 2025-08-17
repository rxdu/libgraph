/*
 * simple_attributes_test.cpp
 *
 * Created on: Aug 2025
 * Description: Simple unit tests for vertex and edge attributes
 */

#include <gtest/gtest.h>
#include <string>

#include "graph/graph.hpp"
#include "graph/search/search_context.hpp"

using namespace xmotion;

// Simple test state
struct SimpleTestState {
    int id;
    SimpleTestState(int i) : id(i) {}
    int64_t GetId() const { return id; }
};

class SimpleAttributeTest : public ::testing::Test {
protected:
    using TestGraph = Graph<SimpleTestState, double>;
    TestGraph graph;
};

TEST_F(SimpleAttributeTest, StateBasedVertexProperties) {
    // Demonstrate state-based approach for persistent vertex data
    struct RichState {
        int id;
        std::string color;
        int weight;
        
        RichState(int i, const std::string& c, int w) : id(i), color(c), weight(w) {}
        int64_t GetId() const { return id; }
    };
    
    using RichGraph = Graph<RichState, double>;
    RichGraph rich_graph;
    
    // Add vertex with rich state containing persistent properties
    RichState s1(1, "red", 42);
    auto v_iter = rich_graph.AddVertex(s1);
    
    ASSERT_NE(v_iter, rich_graph.vertex_end());
    EXPECT_EQ(rich_graph.GetVertexCount(), 1);
    
    // Access persistent properties through state
    EXPECT_EQ(v_iter->state.color, "red");
    EXPECT_EQ(v_iter->state.weight, 42);
    EXPECT_EQ(v_iter->state.id, 1);
}

TEST_F(SimpleAttributeTest, StateBasedEdgeProperties) {
    // Demonstrate transition-based approach for persistent edge data
    struct RoadInfo {
        double distance;
        std::string road_type;
        int lanes;
        
        RoadInfo(double d, const std::string& type, int l) 
            : distance(d), road_type(type), lanes(l) {}
            
        // Implicit conversion to double for compatibility
        operator double() const { return distance; }
    };
    
    using RoadGraph = Graph<SimpleTestState, RoadInfo>;
    RoadGraph road_graph;
    
    SimpleTestState s1(1), s2(2);
    road_graph.AddVertex(s1);
    road_graph.AddVertex(s2);
    
    // Add edge with rich transition data
    RoadInfo road_data(10.5, "highway", 4);
    road_graph.AddEdge(s1, s2, road_data);
    
    auto v1 = road_graph.FindVertex(s1);
    auto edge_iter = v1->FindEdge(s2.GetId());
    ASSERT_NE(edge_iter, v1->edge_end());
    
    // Access persistent edge properties through cost/transition
    EXPECT_EQ(edge_iter->cost.distance, 10.5);
    EXPECT_EQ(edge_iter->cost.road_type, "highway");
    EXPECT_EQ(edge_iter->cost.lanes, 4);
}

TEST_F(SimpleAttributeTest, SearchContextFlexibleAttributes) {
    using TestSearchContext = SearchContext<SimpleTestState, double, DefaultIndexer<SimpleTestState>>;
    TestSearchContext context;
    
    // Test flexible attributes in search context
    context.SetVertexAttribute(1, "g_cost", 10.0);
    context.SetVertexAttribute(1, "algorithm", std::string("dijkstra"));
    
    EXPECT_TRUE(context.HasVertexAttribute(1, "g_cost"));
    EXPECT_EQ(context.GetVertexAttribute<double>(1, "g_cost"), 10.0);
    EXPECT_EQ(context.GetVertexAttribute<std::string>(1, "algorithm"), "dijkstra");
    
    // Test traditional fields still work
    auto& info = context.GetSearchInfo(1);
    info.g_cost = 20.0;
    info.parent_id = 2;
    
    EXPECT_EQ(info.g_cost, 20.0);
    EXPECT_EQ(info.parent_id, 2);
}

TEST_F(SimpleAttributeTest, SearchContextClearAndReset) {
    using TestSearchContext = SearchContext<SimpleTestState, double, DefaultIndexer<SimpleTestState>>;
    TestSearchContext context;
    
    // Set some search attributes
    context.SetVertexAttribute(1, "temp_value", 123);
    context.SetVertexAttribute(1, "algorithm_state", std::string("processing"));
    context.SetVertexAttribute(2, "visited", true);
    
    EXPECT_TRUE(context.HasVertexAttribute(1, "temp_value"));
    EXPECT_TRUE(context.HasVertexAttribute(1, "algorithm_state"));
    EXPECT_TRUE(context.HasVertexAttribute(2, "visited"));
    
    // Clear removes all search data completely
    context.Clear();
    EXPECT_FALSE(context.HasVertexAttribute(1, "temp_value"));
    EXPECT_FALSE(context.HasVertexAttribute(1, "algorithm_state"));
    EXPECT_FALSE(context.HasVertexAttribute(2, "visited"));
    EXPECT_EQ(context.Size(), 0);
}