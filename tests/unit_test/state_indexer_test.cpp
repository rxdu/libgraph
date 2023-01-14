/*
 * graph_iter_test.cpp
 *
 * Created on: Mar 13, 2018 12:26
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include <stdio.h>
#include <vector>
#include <memory>

#include "gtest/gtest.h"

#include "graph/details/default_indexer.hpp"

using namespace rdu;

struct TestStateId {
  TestStateId(uint64_t id) : id(id){};

  int64_t id;

  bool operator==(const TestStateId &other) {
    if (id == other.id) return true;
    return false;
  }
};

struct TestStateIdu {
  TestStateIdu(uint64_t id) : id_(id){};

  int64_t id_;

  bool operator==(const TestStateIdu &other) {
    if (id_ == other.id_) return true;
    return false;
  }
};

class TestStateIdf {
 public:
  TestStateIdf(uint64_t id) : id_(id){};

  bool operator==(const TestStateIdf &other) {
    if (id_ == other.id_) return true;
    return false;
  }

  int64_t GetId() const { return id_; }

 private:
  int64_t id_;
};

//////////////////////////////////////////////////

struct IndexerTest : testing::Test {
  TestStateId node{1};
  TestStateId *node_ptr;
  std::shared_ptr<TestStateId> node_shared_ptr;

  TestStateIdu nodeu{1};
  TestStateIdu *nodeu_ptr;
  std::shared_ptr<TestStateIdu> nodeu_shared_ptr;

  TestStateIdf nodef{1};
  TestStateIdf *nodef_ptr;
  std::shared_ptr<TestStateIdf> nodef_shared_ptr;

  IndexerTest() {
    node_ptr = new TestStateId(2);
    node_shared_ptr = std::make_shared<TestStateId>(3);

    nodeu_ptr = new TestStateIdu(2);
    nodeu_shared_ptr = std::make_shared<TestStateIdu>(3);

    nodef_ptr = new TestStateIdf(2);
    nodef_shared_ptr = std::make_shared<TestStateIdf>(3);
  }

  virtual ~IndexerTest() {
    delete node_ptr;
    delete nodeu_ptr;
    delete nodef_ptr;
  }
};

TEST_F(IndexerTest, HasId) {
  {
    DefaultIndexer<TestStateId> indexer;
    ASSERT_EQ(indexer(node), 1)
        << "Failed to index state of value-type with id";
  }

  {
    DefaultIndexer<TestStateId *> indexer;
    ASSERT_EQ(indexer(node_ptr), 2)
        << "Failed to index state of pointer-type with id";
  }

  {
    DefaultIndexer<std::shared_ptr<TestStateId>> indexer;
    ASSERT_EQ(indexer(node_shared_ptr), 3)
        << "Failed to index state of shared_ptr-type with id";
  }
}

TEST_F(IndexerTest, HasIdUnderscore) {
  {
    DefaultIndexer<TestStateIdu> indexer;
    ASSERT_EQ(indexer(nodeu), 1)
        << "Failed to index state of value-type with id_";
  }

  {
    DefaultIndexer<TestStateIdu *> indexer;
    ASSERT_EQ(indexer(nodeu_ptr), 2)
        << "Failed to index state of pointer-type with id_";
  }

  {
    DefaultIndexer<std::shared_ptr<TestStateIdu>> indexer;
    ASSERT_EQ(indexer(nodeu_shared_ptr), 3)
        << "Failed to index state of shared_ptr-type with id_";
  }
}

TEST_F(IndexerTest, HasGetId) {
  {
    DefaultIndexer<TestStateIdf> indexer;
    ASSERT_EQ(indexer(nodef), 1)
        << "Failed to index state of value-type with id_";
  }

  {
    DefaultIndexer<TestStateIdf *> indexer;
    ASSERT_EQ(indexer(nodef_ptr), 2)
        << "Failed to index state of pointer-type with id_";
  }

  {
    DefaultIndexer<std::shared_ptr<TestStateIdf>> indexer;
    ASSERT_EQ(indexer(nodef_shared_ptr), 3)
        << "Failed to index state of shared_ptr-type with id_";
  }
}
