/*
 * priority_queue_test.cpp
 *
 * Created on: Jan 18, 2021 22:05
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <stdio.h>
#include <vector>
#include <memory>

#include "gtest/gtest.h"

#include "graph/details/dynamic_priority_queue.hpp"

using namespace rdu;

struct TestElement {
  TestElement(uint64_t id) : id_(id){};

  int64_t id_;
};

struct DynamicPriorityQueueTest : testing::Test {
  DynamicPriorityQueueTest() {}

  ~DynamicPriorityQueueTest() {}
};

TEST_F(DynamicPriorityQueueTest, DefaultConstructor) {}
