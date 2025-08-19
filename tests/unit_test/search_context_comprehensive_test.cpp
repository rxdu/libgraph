/*
 * search_context_comprehensive_test.cpp
 *
 * Created on: Aug 2025
 * Description: Comprehensive tests for SearchContext and attribute system
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "graph/graph.hpp"
#include "graph/search/search_context.hpp"
#include "graph/exceptions.hpp"

using namespace xmotion;

// Test state for search context testing
struct ContextTestState {
  int id;
  explicit ContextTestState(int i) : id(i) {}
  bool operator==(const ContextTestState& other) const { return id == other.id; }
  int GetId() const { return id; }
};

// Custom cost type for testing
struct CustomCost {
  double primary;
  int secondary;
  
  CustomCost(double p = 0.0, int s = 0) : primary(p), secondary(s) {}
  
  bool operator==(const CustomCost& other) const {
    return primary == other.primary && secondary == other.secondary;
  }
  
  bool operator<(const CustomCost& other) const {
    if (primary != other.primary) return primary < other.primary;
    return secondary < other.secondary;
  }
  
  static CustomCost max() {
    return CustomCost(std::numeric_limits<double>::max(), 
                     std::numeric_limits<int>::max());
  }
};

// Specialize CostTraits for CustomCost
namespace xmotion {
template<>
struct CostTraits<CustomCost> {
    static CustomCost infinity() { return CustomCost::max(); }
};
}

class SearchContextComprehensiveTest : public ::testing::Test {
protected:
  void SetUp() override {
    context_ = std::make_unique<SearchContext<ContextTestState, double, DefaultIndexer<ContextTestState>>>();
    custom_context_ = std::make_unique<SearchContext<ContextTestState, CustomCost, DefaultIndexer<ContextTestState>>>();
    graph_ = std::make_unique<Graph<ContextTestState, double>>();
  }

  void TearDown() override {
    context_.reset();
    custom_context_.reset();
    graph_.reset();
  }

  std::unique_ptr<SearchContext<ContextTestState, double, DefaultIndexer<ContextTestState>>> context_;
  std::unique_ptr<SearchContext<ContextTestState, CustomCost, DefaultIndexer<ContextTestState>>> custom_context_;
  std::unique_ptr<Graph<ContextTestState, double>> graph_;
};

// Test SearchVertexInfo basic functionality
TEST_F(SearchContextComprehensiveTest, SearchVertexInfoBasicFunctionality) {
  typename SearchContext<ContextTestState, double, DefaultIndexer<ContextTestState>>::SearchVertexInfo info;
  
  // Test initial state
  EXPECT_FALSE(info.GetChecked());
  EXPECT_FALSE(info.GetInOpenList());
  EXPECT_EQ(info.GetParent(), -1);
  EXPECT_DOUBLE_EQ(info.GetGCost<double>(), std::numeric_limits<double>::max());
  
  // Test setting values
  info.SetChecked(true);
  info.SetInOpenList(true);
  info.SetParent(42);
  info.SetGCost(3.14);
  info.SetHCost(2.71);
  info.SetFCost(5.85);
  
  EXPECT_TRUE(info.GetChecked());
  EXPECT_TRUE(info.GetInOpenList());
  EXPECT_EQ(info.GetParent(), 42);
  EXPECT_DOUBLE_EQ(info.GetGCost<double>(), 3.14);
  EXPECT_DOUBLE_EQ(info.GetHCost<double>(), 2.71);
  EXPECT_DOUBLE_EQ(info.GetFCost<double>(), 5.85);
}

// Test legacy property accessors
TEST_F(SearchContextComprehensiveTest, LegacyPropertyAccessors) {
  typename SearchContext<ContextTestState, double, DefaultIndexer<ContextTestState>>::SearchVertexInfo info;
  
  // Test legacy field access
  info.is_checked = true;
  info.is_in_openlist = true;
  info.parent_id = 123;
  info.g_cost = 1.5;
  info.h_cost = 2.5;
  info.f_cost = 4.0;
  
  // Verify through getters
  EXPECT_TRUE(info.GetChecked());
  EXPECT_TRUE(info.GetInOpenList());
  EXPECT_EQ(info.GetParent(), 123);
  EXPECT_DOUBLE_EQ(info.GetGCost<double>(), 1.5);
  EXPECT_DOUBLE_EQ(info.GetHCost<double>(), 2.5);
  EXPECT_DOUBLE_EQ(info.GetFCost<double>(), 4.0);
  
  // Test implicit conversions
  bool checked = info.is_checked;
  bool in_openlist = info.is_in_openlist;
  int64_t parent = info.parent_id;
  double g = info.g_cost;
  
  EXPECT_TRUE(checked);
  EXPECT_TRUE(in_openlist);
  EXPECT_EQ(parent, 123);
  EXPECT_DOUBLE_EQ(g, 1.5);
}

// Test custom attributes functionality
TEST_F(SearchContextComprehensiveTest, CustomAttributesFunctionality) {
  typename SearchContext<ContextTestState, double, DefaultIndexer<ContextTestState>>::SearchVertexInfo info;
  
  // Test setting custom attributes
  info.SetAttribute("custom_int", 42);
  info.SetAttribute("custom_string", std::string("hello"));
  info.SetAttribute("custom_double", 3.14159);
  info.SetAttribute("custom_bool", true);
  
  // Test getting custom attributes
  EXPECT_EQ(info.GetAttribute<int>("custom_int"), 42);
  EXPECT_EQ(info.GetAttribute<std::string>("custom_string"), "hello");
  EXPECT_DOUBLE_EQ(info.GetAttribute<double>("custom_double"), 3.14159);
  EXPECT_TRUE(info.GetAttribute<bool>("custom_bool"));
  
  // Test GetAttributeOr with existing keys
  EXPECT_EQ(info.GetAttributeOr<int>("custom_int", 0), 42);
  
  // Test GetAttributeOr with non-existing keys
  EXPECT_EQ(info.GetAttributeOr<int>("non_existing", 999), 999);
  
  // Test HasAttribute
  EXPECT_TRUE(info.HasAttribute("custom_int"));
  EXPECT_FALSE(info.HasAttribute("non_existing"));
  
  // Test GetAttributeKeys
  auto keys = info.GetAttributeKeys();
  EXPECT_GE(keys.size(), 4); // At least our 4 custom attributes
  
  // Test RemoveAttribute
  EXPECT_TRUE(info.RemoveAttribute("custom_int"));
  EXPECT_FALSE(info.HasAttribute("custom_int"));
  EXPECT_FALSE(info.RemoveAttribute("non_existing"));
}

// Test SearchContext basic operations
TEST_F(SearchContextComprehensiveTest, SearchContextBasicOperations) {
  EXPECT_TRUE(context_->Empty());
  EXPECT_EQ(context_->Size(), 0);
  
  // Add search info
  auto& info1 = context_->GetSearchInfo(1);
  info1.SetGCost(1.5);
  
  auto& info2 = context_->GetSearchInfo(2);
  info2.SetGCost(2.5);
  
  EXPECT_FALSE(context_->Empty());
  EXPECT_EQ(context_->Size(), 2);
  EXPECT_TRUE(context_->HasSearchInfo(1));
  EXPECT_TRUE(context_->HasSearchInfo(2));
  EXPECT_FALSE(context_->HasSearchInfo(3));
  
  // Test const access
  const auto& const_context = *context_;
  const auto& const_info1 = const_context.GetSearchInfo(1);
  EXPECT_DOUBLE_EQ(const_info1.GetGCost<double>(), 1.5);
  
  // Test exception for non-existent vertex
  EXPECT_THROW(const_context.GetSearchInfo(999), ElementNotFoundError);
}

// Test SearchContext with custom cost types
TEST_F(SearchContextComprehensiveTest, CustomCostTypes) {
  auto& info = custom_context_->GetSearchInfo(1);
  
  CustomCost cost1(3.14, 42);
  CustomCost cost2(2.71, 24);
  
  info.SetGCost(cost1);
  info.SetHCost(cost2);
  
  auto retrieved_g = info.GetGCost<CustomCost>();
  auto retrieved_h = info.GetHCost<CustomCost>();
  
  EXPECT_EQ(retrieved_g.primary, 3.14);
  EXPECT_EQ(retrieved_g.secondary, 42);
  EXPECT_EQ(retrieved_h.primary, 2.71);
  EXPECT_EQ(retrieved_h.secondary, 24);
  
  // Test default infinity value
  auto& info2 = custom_context_->GetSearchInfo(2);
  auto default_g = info2.GetGCost<CustomCost>();
  auto infinity = CostTraits<CustomCost>::infinity();
  
  EXPECT_EQ(default_g.primary, infinity.primary);
  EXPECT_EQ(default_g.secondary, infinity.secondary);
}

// Test SearchContext vertex attribute methods
TEST_F(SearchContextComprehensiveTest, VertexAttributeMethods) {
  // Test SetVertexAttribute and GetVertexAttribute
  context_->SetVertexAttribute(1, "score", 100.0);
  context_->SetVertexAttribute(1, "name", std::string("vertex1"));
  context_->SetVertexAttribute(1, "active", true);
  
  EXPECT_DOUBLE_EQ(context_->GetVertexAttribute<double>(1, "score"), 100.0);
  EXPECT_EQ(context_->GetVertexAttribute<std::string>(1, "name"), "vertex1");
  EXPECT_TRUE(context_->GetVertexAttribute<bool>(1, "active"));
  
  // Test GetVertexAttributeOr
  EXPECT_DOUBLE_EQ(context_->GetVertexAttributeOr(1, "score", 0.0), 100.0);
  EXPECT_DOUBLE_EQ(context_->GetVertexAttributeOr(1, "missing", 999.0), 999.0);
  EXPECT_EQ(context_->GetVertexAttributeOr(999, "score", -1.0), -1.0); // Non-existent vertex
  
  // Test HasVertexAttribute
  EXPECT_TRUE(context_->HasVertexAttribute(1, "score"));
  EXPECT_FALSE(context_->HasVertexAttribute(1, "missing"));
  EXPECT_FALSE(context_->HasVertexAttribute(999, "score"));
  
  // Test GetVertexAttributeKeys
  auto keys = context_->GetVertexAttributeKeys(1);
  EXPECT_GE(keys.size(), 3);
  
  auto empty_keys = context_->GetVertexAttributeKeys(999);
  EXPECT_TRUE(empty_keys.empty());
}

// Test SearchContext convenience methods
TEST_F(SearchContextComprehensiveTest, ConvenienceMethods) {
  // Test SetGCost and GetGCost
  context_->SetGCost(1, 5.5);
  EXPECT_DOUBLE_EQ(context_->GetGCost<double>(1), 5.5);
  EXPECT_DOUBLE_EQ(context_->GetGCost<double>(999), std::numeric_limits<double>::max());
  
  // Test SetParent and GetParent
  context_->SetParent(1, 42, true); // Legacy mode
  EXPECT_EQ(context_->GetParent(1, true), 42);
  
  context_->SetParent(2, 24, false); // Flexible attribute mode
  EXPECT_EQ(context_->GetParent(2, false), 24);
  EXPECT_EQ(context_->GetParent(999, true), -1);
}

// Test SearchContext copy and move operations
TEST_F(SearchContextComprehensiveTest, CopyAndMoveOperations) {
  // Set up original context
  auto& info1 = context_->GetSearchInfo(1);
  info1.SetGCost(1.5);
  info1.SetAttribute("custom", 42);
  
  auto& info2 = context_->GetSearchInfo(2);
  info2.SetGCost(2.5);
  info2.SetAttribute("custom", 24);
  
  // Test SearchVertexInfo copy constructor
  typename SearchContext<ContextTestState, double, DefaultIndexer<ContextTestState>>::SearchVertexInfo copied_info(info1);
  EXPECT_DOUBLE_EQ(copied_info.GetGCost<double>(), 1.5);
  EXPECT_EQ(copied_info.GetAttribute<int>("custom"), 42);
  
  // Test SearchVertexInfo copy assignment
  typename SearchContext<ContextTestState, double, DefaultIndexer<ContextTestState>>::SearchVertexInfo assigned_info;
  assigned_info = info2;
  EXPECT_DOUBLE_EQ(assigned_info.GetGCost<double>(), 2.5);
  EXPECT_EQ(assigned_info.GetAttribute<int>("custom"), 24);
  
  // Test SearchVertexInfo move operations
  typename SearchContext<ContextTestState, double, DefaultIndexer<ContextTestState>>::SearchVertexInfo moved_info(std::move(copied_info));
  EXPECT_DOUBLE_EQ(moved_info.GetGCost<double>(), 1.5);
  EXPECT_EQ(moved_info.GetAttribute<int>("custom"), 42);
}

// Test SearchContext Reset functionality
TEST_F(SearchContextComprehensiveTest, ResetFunctionality) {
  // Populate context
  for (int i = 1; i <= 10; ++i) {
    auto& info = context_->GetSearchInfo(i);
    info.SetGCost(static_cast<double>(i));
    info.SetAttribute("value", i * 10);
  }
  
  EXPECT_EQ(context_->Size(), 10);
  
  // Reset should clear values but keep entries for memory efficiency
  context_->Reset();
  
  // Size should remain the same (entries kept for reuse)
  EXPECT_EQ(context_->Size(), 10);
  
  // But values should be reset to defaults
  for (int i = 1; i <= 10; ++i) {
    const auto& info = context_->GetSearchInfo(i);
    // Note: After reset, attributes are cleared, so accessing them may throw
    EXPECT_FALSE(info.HasAttribute("value"));
  }
  
  // Clear should remove all entries
  context_->Clear();
  EXPECT_EQ(context_->Size(), 0);
  EXPECT_TRUE(context_->Empty());
}

// Test ReconstructPath functionality
TEST_F(SearchContextComprehensiveTest, ReconstructPathFunctionality) {
  // Set up graph
  for (int i = 1; i <= 4; ++i) {
    graph_->AddVertex(ContextTestState(i));
  }
  
  graph_->AddEdge(ContextTestState(1), ContextTestState(2), 1.0);
  graph_->AddEdge(ContextTestState(2), ContextTestState(3), 1.0);
  graph_->AddEdge(ContextTestState(3), ContextTestState(4), 1.0);
  
  // Set up search context as if a search was performed: 1 -> 2 -> 3 -> 4
  auto& info1 = context_->GetSearchInfo(1);
  info1.parent_id = -1; // Start vertex
  
  auto& info2 = context_->GetSearchInfo(2);
  info2.parent_id = 1;
  
  auto& info3 = context_->GetSearchInfo(3);
  info3.parent_id = 2;
  
  auto& info4 = context_->GetSearchInfo(4);
  info4.parent_id = 3;
  
  // Reconstruct path
  auto path = context_->ReconstructPath(graph_.get(), 4);
  
  EXPECT_EQ(path.size(), 4);
  EXPECT_EQ(path[0].id, 1);
  EXPECT_EQ(path[1].id, 2);
  EXPECT_EQ(path[2].id, 3);
  EXPECT_EQ(path[3].id, 4);
  
  // Test with non-existent goal
  EXPECT_THROW(context_->ReconstructPath(graph_.get(), 999), ElementNotFoundError);
  
  // Test with unreachable goal (no parent chain)
  auto& info5 = context_->GetSearchInfo(5);
  info5.parent_id = -1; // Isolated
  
  auto empty_path = context_->ReconstructPath(graph_.get(), 5);
  EXPECT_TRUE(empty_path.empty());
}

// Test SearchContext performance with large data
TEST_F(SearchContextComprehensiveTest, LargeDataPerformance) {
  const int LARGE_N = 1000;
  
  auto start_time = std::chrono::high_resolution_clock::now();
  
  // Add many vertices with attributes
  for (int i = 1; i <= LARGE_N; ++i) {
    auto& info = context_->GetSearchInfo(i);
    info.SetGCost(static_cast<double>(i));
    info.SetHCost(static_cast<double>(i * 2));
    info.SetAttribute("iteration", i);
    info.SetAttribute("name", "vertex_" + std::to_string(i));
  }
  
  auto mid_time = std::chrono::high_resolution_clock::now();
  
  // Access all vertices
  double sum = 0.0;
  for (int i = 1; i <= LARGE_N; ++i) {
    const auto& info = context_->GetSearchInfo(i);
    sum += info.GetGCost<double>();
    sum += info.GetAttribute<int>("iteration");
  }
  
  auto end_time = std::chrono::high_resolution_clock::now();
  
  auto insert_duration = std::chrono::duration_cast<std::chrono::milliseconds>(mid_time - start_time);
  auto access_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - mid_time);
  
  EXPECT_EQ(context_->Size(), LARGE_N);
  EXPECT_GT(sum, 0); // Ensure computation happened
  
  // Performance should be reasonable
  EXPECT_LT(insert_duration.count(), 100); // Less than 100ms for insertions
  EXPECT_LT(access_duration.count(), 50);  // Less than 50ms for access
}

// Test error conditions and edge cases
TEST_F(SearchContextComprehensiveTest, ErrorConditionsAndEdgeCases) {
  typename SearchContext<ContextTestState, double, DefaultIndexer<ContextTestState>>::SearchVertexInfo info;
  
  // Test accessing attributes when none exist
  EXPECT_FALSE(info.HasAttribute("nonexistent"));
  EXPECT_THROW(info.GetAttribute<int>("nonexistent"), std::out_of_range);
  EXPECT_EQ(info.GetAttributeOr<int>("nonexistent", 42), 42);
  
  // Test empty attribute keys
  auto keys = info.GetAttributeKeys();
  EXPECT_TRUE(keys.empty());
  
  // Test removing non-existent attribute
  EXPECT_FALSE(info.RemoveAttribute("nonexistent"));
  
  // Test SearchContext edge cases
  EXPECT_THROW(context_->GetVertexAttribute<int>(999, "key"), ElementNotFoundError);
  
  // Test Reset on empty context
  SearchContext<ContextTestState, double, DefaultIndexer<ContextTestState>> empty_context;
  empty_context.Reset(); // Should not crash
  empty_context.Clear(); // Should not crash
  
  EXPECT_TRUE(empty_context.Empty());
  EXPECT_EQ(empty_context.Size(), 0);
}

// Test iterator-based SearchContext access
TEST_F(SearchContextComprehensiveTest, IteratorBasedAccess) {
  // Set up graph and context
  for (int i = 1; i <= 3; ++i) {
    graph_->AddVertex(ContextTestState(i));
  }
  
  auto vertex1_it = graph_->FindVertex(ContextTestState(1));
  auto vertex2_it = graph_->FindVertex(ContextTestState(2));
  
  // Test non-const iterator access
  auto& info1 = context_->GetSearchInfo(vertex1_it);
  info1.SetGCost(1.5);
  
  auto& info2 = context_->GetSearchInfo(vertex2_it);
  info2.SetGCost(2.5);
  
  // Test const iterator access
  const auto& const_info1 = context_->GetSearchInfo(vertex1_it);
  EXPECT_DOUBLE_EQ(const_info1.GetGCost<double>(), 1.5);
  
  // Test with const vertex iterator
  auto const_vertex1_it = static_cast<const Graph<ContextTestState, double>&>(*graph_).FindVertex(ContextTestState(1));
  auto& info1_from_const_it = context_->GetSearchInfo(const_vertex1_it);
  EXPECT_DOUBLE_EQ(info1_from_const_it.GetGCost<double>(), 1.5);
}