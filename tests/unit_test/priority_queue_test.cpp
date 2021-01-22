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

std::ostream& operator<<(std::ostream& os, const TestElement& e) {
  os << e.value;
  return os;
}

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

TEST_F(DynamicPriorityQueueTest, Constructor) {
  DynamicPriorityQueue<TestElement, TEComparator> queue1;
  ASSERT_TRUE(queue1.Empty());

  DynamicPriorityQueue<TestElement, TEComparator> queue2(50);
  queue2.Push(elements[0]);
  ASSERT_FALSE(queue2.Empty());

  DynamicPriorityQueue<TestElement, TEComparator> queue3(elements);
  ASSERT_FALSE(queue3.Empty());
  ASSERT_EQ(queue3.GetQueueElementNumber(), elements.size());
}

TEST_F(DynamicPriorityQueueTest, PointerType) {
  std::vector<TestElement*> element_ptrs;
  for (std::size_t i = 0; i < elements.size(); ++i)
    element_ptrs.push_back(&elements[i]);
  DynamicPriorityQueue<TestElement*, TEComparator> queue(element_ptrs);
  ASSERT_EQ(queue.GetQueueElementNumber(), elements.size());

  ASSERT_FLOAT_EQ(queue.Peek()->id, 6);
  ASSERT_FLOAT_EQ(queue.Peek()->value, 1);
}

TEST_F(DynamicPriorityQueueTest, PushPop) {
  DynamicPriorityQueue<TestElement, TEComparator> queue;
  queue.Push(elements[0]);
  ASSERT_FLOAT_EQ(queue.GetQueueElementNumber(), 1);
  ASSERT_TRUE(queue.Contains(elements[0]));
  ASSERT_FALSE(queue.Contains(elements[1]));

  elements[0].value = 1.1;
  queue.Push(elements[0]);
  ASSERT_FLOAT_EQ(queue.Peek().value, 1.1);

  for (std::size_t i = 1; i < elements.size(); ++i) queue.Push(elements[i]);
  ASSERT_EQ(queue.GetQueueElementNumber(), elements.size());
  ASSERT_TRUE(queue.Contains(elements[1]));

  queue.Peek();
  ASSERT_EQ(queue.GetQueueElementNumber(), elements.size());

  queue.Pop();
  ASSERT_EQ(queue.GetQueueElementNumber(), elements.size() - 1);

  queue.PrintQueue();

  queue.Clear();
  ASSERT_FLOAT_EQ(queue.GetQueueElementNumber(), 0);

  queue.PrintQueue();
}

TEST_F(DynamicPriorityQueueTest, HeapProperty) {
  // 10, 5, 20, 8, 6, 2, 1, 30
  DynamicPriorityQueue<TestElement, TEComparator> queue;
  ASSERT_EQ(queue.Peek().id, -1);
  ASSERT_EQ(queue.Pop().id, -1);

  {
    testing::internal::CaptureStdout();
    queue.PrintQueue();
    std::string output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(output, "\n");
  }

  // add 10
  queue.Push(elements[0]);
  ASSERT_EQ(queue.Peek().id, 0);
  ASSERT_FLOAT_EQ(queue.Peek().value, 10);

  {
    testing::internal::CaptureStdout();
    queue.PrintQueue();
    std::string output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(output, "10 \n");
  }

  // add 5
  queue.Push(elements[1]);
  ASSERT_EQ(queue.Peek().id, 1);
  ASSERT_FLOAT_EQ(queue.Peek().value, 5);

  {
    testing::internal::CaptureStdout();
    queue.PrintQueue();
    std::string output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(output, "5 10 \n");
  }

  // add 20
  queue.Push(elements[2]);
  ASSERT_EQ(queue.Peek().id, 1);
  ASSERT_FLOAT_EQ(queue.Peek().value, 5);

  {
    testing::internal::CaptureStdout();
    queue.PrintQueue();
    std::string output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(output, "5 10 20 \n");
  }

  // add 8
  queue.Push(elements[3]);
  ASSERT_EQ(queue.Peek().id, 1);
  ASSERT_FLOAT_EQ(queue.Peek().value, 5);

  {
    testing::internal::CaptureStdout();
    queue.PrintQueue();
    std::string output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(output, "5 8 20 10 \n");
  }

  // add 6
  queue.Push(elements[4]);
  ASSERT_EQ(queue.Peek().id, 1);
  ASSERT_FLOAT_EQ(queue.Peek().value, 5);

  {
    testing::internal::CaptureStdout();
    queue.PrintQueue();
    std::string output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(output, "5 6 20 10 8 \n");
  }

  // add 2
  queue.Push(elements[5]);
  ASSERT_EQ(queue.Peek().id, 5);
  ASSERT_FLOAT_EQ(queue.Peek().value, 2);

  {
    testing::internal::CaptureStdout();
    queue.PrintQueue();
    std::string output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(output, "2 6 5 10 8 20 \n");
  }

  // add 1
  queue.Push(elements[6]);
  ASSERT_EQ(queue.Peek().id, 6);
  ASSERT_FLOAT_EQ(queue.Peek().value, 1);

  {
    testing::internal::CaptureStdout();
    queue.PrintQueue();
    std::string output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(output, "1 6 2 10 8 20 5 \n");
  }

  // add 30
  queue.Push(elements[7]);
  ASSERT_EQ(queue.Peek().id, 6);
  ASSERT_FLOAT_EQ(queue.Peek().value, 1);

  {
    testing::internal::CaptureStdout();
    queue.PrintQueue();
    std::string output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(output, "1 6 2 10 8 20 5 30 \n");
  }

  // now pop
  ASSERT_FLOAT_EQ(queue.Peek().value, 1);
  ASSERT_FLOAT_EQ(queue.Pop().value, 1);

  {
    testing::internal::CaptureStdout();
    queue.PrintQueue();
    std::string output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(output, "2 6 5 10 8 20 30 \n");
  }

  ASSERT_FLOAT_EQ(queue.Peek().value, 2);
  ASSERT_FLOAT_EQ(queue.Pop().value, 2);

  {
    testing::internal::CaptureStdout();
    queue.PrintQueue();
    std::string output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(output, "5 6 20 10 8 30 \n");
  }

  ASSERT_FLOAT_EQ(queue.Peek().value, 5);
  ASSERT_FLOAT_EQ(queue.Pop().value, 5);

  {
    testing::internal::CaptureStdout();
    queue.PrintQueue();
    std::string output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(output, "6 8 20 10 30 \n");
  }

  ASSERT_FLOAT_EQ(queue.Peek().value, 6);
  ASSERT_FLOAT_EQ(queue.Pop().value, 6);

  {
    testing::internal::CaptureStdout();
    queue.PrintQueue();
    std::string output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(output, "8 10 20 30 \n");
  }

  ASSERT_FLOAT_EQ(queue.Peek().value, 8);
  ASSERT_FLOAT_EQ(queue.Pop().value, 8);

  {
    testing::internal::CaptureStdout();
    queue.PrintQueue();
    std::string output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(output, "10 30 20 \n");
  }

  ASSERT_FLOAT_EQ(queue.Peek().value, 10);
  ASSERT_FLOAT_EQ(queue.Pop().value, 10);

  {
    testing::internal::CaptureStdout();
    queue.PrintQueue();
    std::string output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(output, "20 30 \n");
  }

  ASSERT_FLOAT_EQ(queue.Peek().value, 20);
  ASSERT_FLOAT_EQ(queue.Pop().value, 20);

  {
    testing::internal::CaptureStdout();
    queue.PrintQueue();
    std::string output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(output, "30 \n");
  }

  ASSERT_FLOAT_EQ(queue.Peek().value, 30);
  ASSERT_FLOAT_EQ(queue.Pop().value, 30);

  {
    testing::internal::CaptureStdout();
    queue.PrintQueue();
    std::string output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(output, "\n");
  }
}

TEST_F(DynamicPriorityQueueTest, Update) {
  // 10, 5, 20, 8, 6
  DynamicPriorityQueue<TestElement, TEComparator> queue;
  for (int i = 0; i < 5; ++i) queue.Push(elements[i]);

  ASSERT_FLOAT_EQ(queue.Peek().value, 5);

  elements[4].value = 5;
  queue.Update(elements[4]);

  ASSERT_FLOAT_EQ(queue.Peek().id, 1);
  ASSERT_FLOAT_EQ(queue.Peek().value, 5);

  elements[4].value = 3;
  queue.Update(elements[4]);

  ASSERT_FLOAT_EQ(queue.Peek().id, 4);
  ASSERT_FLOAT_EQ(queue.Peek().value, 3);

  elements[4].value = 7;
  queue.Update(elements[4]);

  ASSERT_FLOAT_EQ(queue.Peek().id, 1);
  ASSERT_FLOAT_EQ(queue.Peek().value, 5);

  elements[1].value = 9;
  elements[4].value = 15;
  queue.Update(elements[1]);
  queue.Update(elements[4]);

  ASSERT_FLOAT_EQ(queue.Peek().id, 3);
  ASSERT_FLOAT_EQ(queue.Peek().value, 8);
}
