/*
 * priority_queue_test.cpp
 *
 * Created on: Jan 18, 2021 22:05
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <iostream>
#include <vector>
#include <memory>

#include "gtest/gtest.h"
#include "graph/impl/dynamic_priority_queue.hpp"
#include "graph/graph.hpp"

using namespace xmotion;

struct TestElement {
  TestElement() = default;
  TestElement(uint64_t _id, double _value) : id(_id), value(_value){};

  int64_t id = -1;
  double value = 0;
};

struct TEComparator {
  bool operator()(const TestElement& x, const TestElement& y) const {
    if (x.value < y.value) return true;
    return false;
  }

  bool operator()(TestElement* x, TestElement* y) const {
    if (x->value < y->value) return true;
    return false;
  }
};

struct DynamicPriorityQueueTest : testing::Test {
  DynamicPriorityQueueTest() {
    std::size_t i = 0;
    elements.push_back(TestElement(i++, 10));
    elements.push_back(TestElement(i++, 5));
    elements.push_back(TestElement(i++, 20));
    elements.push_back(TestElement(i++, 8));
    elements.push_back(TestElement(i++, 6));
    elements.push_back(TestElement(i++, 2));
    elements.push_back(TestElement(i++, 1));
    elements.push_back(TestElement(i++, 30));
  }

  std::vector<TestElement> elements;
};

TEST_F(DynamicPriorityQueueTest, QueueForGraph) {
  // Use TestElement's value field for priority (instead of deprecated vertex g_cost)
  // Set values for priority ordering: elements[0]=10, elements[1]=5, elements[2]=20
  // After sorting by value: elements[1](5) < elements[0](10) < elements[2](20)

  Graph<TestElement*, double> graph;

  graph.AddEdge(&elements[0], &elements[1], 1.0);
  graph.AddEdge(&elements[1], &elements[2], 2.0);

  using VertexIterator = Graph<TestElement*, double>::vertex_iterator;

  struct VertexComparator {
    bool operator()(VertexIterator x, VertexIterator y) const {
      // Compare using TestElement's value field
      return (x->state->value < y->state->value);
    }
  };

  struct VertexIndexer {
    int64_t operator()(VertexIterator vtx) const {
      return static_cast<int64_t>(vtx->vertex_id);
    }
  };

  DynamicPriorityQueue<VertexIterator, VertexComparator, VertexIndexer> queue;

  VertexComparator compare;
  compare(graph.FindVertex(0), graph.FindVertex(1));

  queue.Push(graph.FindVertex(0));  // value = 10
  queue.Push(graph.FindVertex(1));  // value = 5
  queue.Push(graph.FindVertex(2));  // value = 20

  // Min-heap: smallest value comes out first
  ASSERT_EQ(queue.Pop()->state->value, 5.0);   // elements[1]
  ASSERT_EQ(queue.Pop()->state->value, 10.0);  // elements[0]
  ASSERT_EQ(queue.Pop()->state->value, 20.0);  // elements[2]
}