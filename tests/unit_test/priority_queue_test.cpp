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

using namespace rdu;

struct TestElement {
  TestElement() = default;
  TestElement(uint64_t _id, double _value) : id(_id), value(_value){};

  int64_t id;
  double value;
};

struct TEComparator {
  bool operator()(const TestElement& x, const TestElement& y) const {
    if (x.value < y.value) return true;
    return false;
  }
};

std::ostream& operator<<(std::ostream& os, const TestElement& e) {
  os << e.value;
  return os;
}

struct DynamicPriorityQueueTest : testing::Test {
  DynamicPriorityQueueTest() {
    queue.Push(TestElement(0, 10));
    queue.Push(TestElement(1, 5));
    queue.Push(TestElement(2, 2));
  }

  DynamicPriorityQueue<TestElement, TEComparator> queue;
};

TEST_F(DynamicPriorityQueueTest, DefaultConstructor) { queue.PrintInfo(); }
