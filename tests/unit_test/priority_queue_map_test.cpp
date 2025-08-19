/*
 * priority_queue_map_test.cpp
 *
 * Test element_map_ consistency in DynamicPriorityQueue
 */

#include <iostream>
#include <vector>
#include <memory>
#include <set>

#include "gtest/gtest.h"
#include "graph/impl/dynamic_priority_queue.hpp"

using namespace xmotion;

struct MapTestElement {
  MapTestElement() = default;
  MapTestElement(int64_t _id, double _value) : id(_id), value(_value) {}
  
  int64_t id = -1;
  double value = 0;
  
  int64_t GetId() const { return id; }
};

struct MapTestComparator {
  bool operator()(const MapTestElement& x, const MapTestElement& y) const {
    return x.value < y.value;
  }
};

TEST(DynamicPriorityQueueMapTest, ElementMapConsistency) {
  DynamicPriorityQueue<MapTestElement, MapTestComparator> pq;
  
  // Test 1: Push multiple elements and verify Contains() works
  std::vector<MapTestElement> elements;
  for (int i = 0; i < 10; ++i) {
    elements.push_back(MapTestElement(i, 10.0 - i));
    pq.Push(elements.back());
  }
  
  // All elements should be contained
  for (const auto& elem : elements) {
    EXPECT_TRUE(pq.Contains(elem)) << "Element with id " << elem.id << " not found";
  }
  
  // Test 2: Pop elements and verify they're removed from map
  std::set<int64_t> popped_ids;
  for (int i = 0; i < 5; ++i) {
    auto elem = pq.Pop();
    popped_ids.insert(elem.id);
    // Popped element should no longer be contained
    EXPECT_FALSE(pq.Contains(elem)) << "Popped element with id " << elem.id << " still in map";
  }
  
  // Remaining elements should still be contained
  for (const auto& elem : elements) {
    if (popped_ids.find(elem.id) == popped_ids.end()) {
      EXPECT_TRUE(pq.Contains(elem)) << "Remaining element with id " << elem.id << " not found";
    }
  }
  
  // Test 3: Update elements and verify map consistency
  MapTestElement update_elem(3, 0.5);  // Should still be in queue
  pq.Update(update_elem);
  EXPECT_TRUE(pq.Contains(update_elem)) << "Updated element not found";
  
  // Test 4: Clear and verify map is empty
  pq.Clear();
  for (const auto& elem : elements) {
    EXPECT_FALSE(pq.Contains(elem)) << "Element with id " << elem.id << " still in map after Clear()";
  }
}

TEST(DynamicPriorityQueueMapTest, VectorConstructor) {
  // Test that vector constructor properly initializes element_map_
  std::vector<MapTestElement> elements;
  for (int i = 0; i < 5; ++i) {
    elements.push_back(MapTestElement(i, i * 2.0));
  }
  
  DynamicPriorityQueue<MapTestElement, MapTestComparator> pq(elements);
  
  // All elements should be contained
  for (const auto& elem : elements) {
    EXPECT_TRUE(pq.Contains(elem)) << "Element with id " << elem.id << " not found after vector construction";
  }
  
  // Pop all and verify order and map cleanup
  double last_value = -1;
  while (!pq.Empty()) {
    auto elem = pq.Pop();
    EXPECT_GE(elem.value, last_value) << "Heap order violated";
    last_value = elem.value;
    EXPECT_FALSE(pq.Contains(elem)) << "Popped element still in map";
  }
}

TEST(DynamicPriorityQueueMapTest, StressTest) {
  // Stress test with many operations
  DynamicPriorityQueue<MapTestElement, MapTestComparator> pq;
  
  // Push 100 elements
  for (int i = 0; i < 100; ++i) {
    pq.Push(MapTestElement(i, rand() % 1000 / 10.0));
  }
  
  // Pop 50 elements
  for (int i = 0; i < 50; ++i) {
    auto elem = pq.Pop();
    EXPECT_FALSE(pq.Contains(elem));
  }
  
  // Update some remaining elements
  for (int i = 60; i < 80; ++i) {
    MapTestElement update(i, rand() % 1000 / 10.0);
    pq.Update(update);
  }
  
  // Push 50 more elements
  for (int i = 100; i < 150; ++i) {
    pq.Push(MapTestElement(i, rand() % 1000 / 10.0));
  }
  
  // Verify we have the expected number of elements
  EXPECT_EQ(pq.GetQueueElementNumber(), 100);
  
  // Pop all remaining and verify heap property
  double last_value = -1;
  while (!pq.Empty()) {
    auto elem = pq.Pop();
    EXPECT_GE(elem.value, last_value) << "Heap order violated";
    last_value = elem.value;
  }
}