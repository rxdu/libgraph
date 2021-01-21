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
#include "graph/details/dynamic_priority_queue.hpp"
#include "graph/graph.hpp"

using namespace rdu;

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
  Graph<TestElement*, double> graph;

  graph.AddEdge(&elements[0], &elements[1], 1.0);
  graph.AddEdge(&elements[1], &elements[2], 2.0);

  graph.FindVertex(0)->g_cost = 1.0;
  graph.FindVertex(1)->g_cost = 2.0;
  graph.FindVertex(2)->g_cost = 3.0;

  using VertexIterator = Graph<TestElement*, double>::vertex_iterator;

  struct VertexComparator {
    bool operator()(VertexIterator x, VertexIterator y) const {
      return (x->g_cost < y->g_cost);
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

  queue.Push(graph.FindVertex(0));
  queue.Push(graph.FindVertex(1));
  queue.Push(graph.FindVertex(2));

  ASSERT_EQ(queue.Pop()->g_cost, 1.0);
  ASSERT_EQ(queue.Pop()->g_cost, 2.0);
  ASSERT_EQ(queue.Pop()->g_cost, 3.0);
}