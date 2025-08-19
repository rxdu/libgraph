/*
 * stl_iterator_compatibility_test.cpp
 *
 * Test STL compatibility of graph iterators with standard algorithms and containers
 */

#include <iostream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <numeric>
#include <set>
#include <unordered_set>

#include "gtest/gtest.h"

#include "graph/graph.hpp"

using namespace xmotion;

struct STLTestState {
    int64_t id;
    std::string name;
    
    STLTestState(int64_t id_, const std::string& name_) : id(id_), name(name_) {}
    
    int64_t GetId() const { return id; }
    
    bool operator==(const STLTestState& other) const {
        return id == other.id && name == other.name;
    }
};

class STLIteratorCompatibilityTest : public ::testing::Test {
protected:
    Graph<STLTestState, double> graph;
    std::vector<STLTestState> test_states;
    
    void SetUp() override {
        // Create test states
        test_states.emplace_back(1, "Alpha");
        test_states.emplace_back(2, "Beta");
        test_states.emplace_back(3, "Gamma");
        test_states.emplace_back(4, "Delta");
        test_states.emplace_back(5, "Epsilon");
        
        // Add to graph
        for (const auto& state : test_states) {
            graph.AddVertex(state);
        }
        
        // Add some edges
        graph.AddEdge(test_states[0], test_states[1], 1.0);
        graph.AddEdge(test_states[1], test_states[2], 2.0);
        graph.AddEdge(test_states[2], test_states[3], 3.0);
        graph.AddEdge(test_states[3], test_states[4], 4.0);
    }
};

TEST_F(STLIteratorCompatibilityTest, IteratorTraits) {
    // Test that iterator_traits work correctly
    using vertex_iter = Graph<STLTestState, double>::vertex_iterator;
    using const_vertex_iter = Graph<STLTestState, double>::const_vertex_iterator;
    
    // Check iterator_traits for vertex_iterator (C++11 compatible)
    static_assert(std::is_same<std::iterator_traits<vertex_iter>::iterator_category, 
                              std::forward_iterator_tag>::value, 
                  "vertex_iterator should be forward iterator");
    static_assert(std::is_same<std::iterator_traits<vertex_iter>::difference_type, 
                              std::ptrdiff_t>::value, 
                  "vertex_iterator difference_type should be ptrdiff_t");
    
    // Check iterator_traits for const_vertex_iterator
    static_assert(std::is_same<std::iterator_traits<const_vertex_iter>::iterator_category, 
                              std::forward_iterator_tag>::value, 
                  "const_vertex_iterator should be forward iterator");
    static_assert(std::is_same<std::iterator_traits<const_vertex_iter>::difference_type, 
                              std::ptrdiff_t>::value, 
                  "const_vertex_iterator difference_type should be ptrdiff_t");
}

TEST_F(STLIteratorCompatibilityTest, STLAlgorithmDistance) {
    // Test std::distance works with iterators
    auto distance = std::distance(graph.vertex_begin(), graph.vertex_end());
    EXPECT_EQ(distance, 5);
    
    // Test with const iterators
    auto const_distance = std::distance(graph.vertex_cbegin(), graph.vertex_cend());
    EXPECT_EQ(const_distance, 5);
}

TEST_F(STLIteratorCompatibilityTest, STLAlgorithmAdvance) {
    // Test std::advance works with iterators
    auto it = graph.vertex_begin();
    std::advance(it, 2);
    
    EXPECT_NE(it, graph.vertex_begin());
    EXPECT_NE(it, graph.vertex_end());
    
    // Advance to end
    std::advance(it, 3);
    EXPECT_EQ(it, graph.vertex_end());
}

TEST_F(STLIteratorCompatibilityTest, STLAlgorithmForEach) {
    // Test std::for_each works with iterators
    std::vector<int64_t> collected_ids;
    
    std::for_each(graph.vertex_begin(), graph.vertex_end(), 
                  [&collected_ids](const Graph<STLTestState, double>::Vertex& vertex) {
                      collected_ids.push_back(vertex.state.id);
                  });
    
    EXPECT_EQ(collected_ids.size(), 5);
    
    // Sort for comparison since graph iteration order isn't guaranteed
    std::sort(collected_ids.begin(), collected_ids.end());
    std::vector<int64_t> expected_ids = {1, 2, 3, 4, 5};
    EXPECT_EQ(collected_ids, expected_ids);
}

TEST_F(STLIteratorCompatibilityTest, STLAlgorithmFind) {
    // Test std::find_if works with iterators
    auto it = std::find_if(graph.vertex_begin(), graph.vertex_end(),
                          [](const Graph<STLTestState, double>::Vertex& vertex) {
                              return vertex.state.name == "Gamma";
                          });
    
    EXPECT_NE(it, graph.vertex_end());
    EXPECT_EQ(it->state.name, "Gamma");
    EXPECT_EQ(it->state.id, 3);
}

TEST_F(STLIteratorCompatibilityTest, STLAlgorithmCount) {
    // Test std::count_if works with iterators
    auto count = std::count_if(graph.vertex_begin(), graph.vertex_end(),
                              [](const Graph<STLTestState, double>::Vertex& vertex) {
                                  return vertex.state.id > 2;
                              });
    
    EXPECT_EQ(count, 3); // Gamma, Delta, Epsilon
}

TEST_F(STLIteratorCompatibilityTest, STLAlgorithmTransform) {
    // Test std::transform works with iterators
    std::vector<std::string> names;
    
    std::transform(graph.vertex_begin(), graph.vertex_end(),
                   std::back_inserter(names),
                   [](const Graph<STLTestState, double>::Vertex& vertex) {
                       return vertex.state.name;
                   });
    
    EXPECT_EQ(names.size(), 5);
    
    // Check that all expected names are present
    std::sort(names.begin(), names.end());
    std::vector<std::string> expected_names = {"Alpha", "Beta", "Delta", "Epsilon", "Gamma"};
    EXPECT_EQ(names, expected_names);
}

TEST_F(STLIteratorCompatibilityTest, STLAlgorithmCopy) {
    // Test that iterators work with copy-like operations  
    // Since vertices have complex internal structure, test copying values instead
    std::vector<STLTestState> states;
    
    std::transform(graph.vertex_begin(), graph.vertex_end(),
                   std::back_inserter(states),
                   [](const Graph<STLTestState, double>::Vertex& vertex) { return vertex.state; });
    
    EXPECT_EQ(states.size(), 5);
    
    // Verify copied states
    std::vector<int64_t> copied_ids;
    for (const auto& state : states) {
        copied_ids.push_back(state.id);
    }
    
    std::sort(copied_ids.begin(), copied_ids.end());
    std::vector<int64_t> expected_ids = {1, 2, 3, 4, 5};
    EXPECT_EQ(copied_ids, expected_ids);
}

TEST_F(STLIteratorCompatibilityTest, RangeBasedFor) {
    // Test range-based for loops work
    std::vector<int64_t> ids;
    
    for (const auto& vertex : graph.vertices()) {
        ids.push_back(vertex.state.id);
    }
    
    EXPECT_EQ(ids.size(), 5);
    
    std::sort(ids.begin(), ids.end());
    std::vector<int64_t> expected_ids = {1, 2, 3, 4, 5};
    EXPECT_EQ(ids, expected_ids);
}

TEST_F(STLIteratorCompatibilityTest, IteratorSwap) {
    // Test std::swap works with iterators
    auto it1 = graph.vertex_begin();
    auto it2 = graph.vertex_begin();
    ++it2; // Point to second vertex
    
    auto original_it1_state = it1->state.id;
    auto original_it2_state = it2->state.id;
    
    // Swap iterators
    std::swap(it1, it2);
    
    // Verify swap worked
    EXPECT_EQ(it1->state.id, original_it2_state);
    EXPECT_EQ(it2->state.id, original_it1_state);
}

TEST_F(STLIteratorCompatibilityTest, IteratorCopy) {
    // Test iterator copy semantics
    auto it1 = graph.vertex_begin();
    auto it2 = it1; // Copy constructor
    
    EXPECT_EQ(it1, it2);
    EXPECT_EQ(it1->state.id, it2->state.id);
    
    // Test assignment
    auto it3 = graph.vertex_begin();
    ++it3;
    it3 = it1; // Assignment
    
    EXPECT_EQ(it1, it3);
    EXPECT_EQ(it1->state.id, it3->state.id);
}

TEST_F(STLIteratorCompatibilityTest, ConstCorrectness) {
    // Test const iterator conversion and usage
    auto mutable_it = graph.vertex_begin();
    Graph<STLTestState, double>::const_vertex_iterator const_it = mutable_it;
    
    EXPECT_EQ(mutable_it->state.id, const_it->state.id);
    
    // Test that const iterators work with STL algorithms
    auto count = std::count_if(graph.vertex_cbegin(), graph.vertex_cend(),
                              [](const Graph<STLTestState, double>::Vertex& vertex) {
                                  return vertex.state.id % 2 == 0;
                              });
    
    EXPECT_EQ(count, 2); // Beta (2) and Delta (4)
}

TEST_F(STLIteratorCompatibilityTest, EdgeIteratorSTLCompatibility) {
    // Test edge iterators work with STL algorithms
    // Find the vertex with ID 1 (which has the outgoing edge to vertex 2)
    auto vertex_it = graph.FindVertex(1);
    ASSERT_NE(vertex_it, graph.vertex_end()) << "Vertex with ID 1 should exist";
    
    // Count edges from first vertex
    auto edge_count = std::distance(vertex_it->edge_begin(), vertex_it->edge_end());
    EXPECT_EQ(edge_count, 1); // First vertex has one outgoing edge
    
    // Test for_each on edges
    std::vector<double> edge_costs;
    std::for_each(vertex_it->edge_begin(), vertex_it->edge_end(),
                  [&edge_costs](const Graph<STLTestState, double>::Edge& edge) {
                      edge_costs.push_back(edge.cost);
                  });
    
    EXPECT_EQ(edge_costs.size(), 1);
    EXPECT_EQ(edge_costs[0], 1.0);
}