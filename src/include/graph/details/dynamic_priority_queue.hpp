/*
 * dynamic_priority_queue.hpp
 *
 * Created on: Sep 04, 2018 12:10
 * Description:
 *
 * Reference:
 * [1] Data Structures & Algorithm Analysis in C++, 4th edition. Mark A. Weiss.
 * Published by Pearson (2013).
 * [2] https://github.com/csbence/DynamicPriorityQueue
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef DYNAMIC_PRIORITY_QUEUE_HPP
#define DYNAMIC_PRIORITY_QUEUE_HPP

#include <list>
#include <vector>
#include <unordered_map>
#include <algorithm>

#include "graph/details/default_indexer.hpp"

namespace rdu {
/// A priority queue implementation that supports element priority update.
template <typename Comparable,
          typename ItemIndexer = DefaultIndexer<Comparable>>
class DynamicPriorityQueue {
 public:
  DynamicPriorityQueue() = default;

  void Insert(const Comparable& element) {
    // check whether element is already in queue
    if (!Contains(element)) {
      const std::size_t index = elements_.size();
      elements_.push_back(element);
      element_map_[GetItemIndex(element)] = index;
      if (index > 1) PercolateUp(index);
    }
    // otherwise update existing element
    Update(element);
  }

  void Update(const Comparable& element) {
    // find element first
  }

  bool Empty() const { return elements_.empty(); }

  Comparable GetMin() const { return elements_[0]; }
  void DeleteMin() {}

  Comparable PopMin() {
    auto min = elements_[0];
    DeleteMin();
    return min;
  }

  void Remove(const Comparable& element) {
    // find element first
  }

  bool Contains(const Comparable& element) const {
    return element_map_.find(element) != element_map_.end() ? true : false;
  }

 private:
  std::vector<Comparable> elements_;
  std::unordered_map<int64_t, std::size_t> element_map_;
  ItemIndexer GetItemIndex;

  void PercolateUp(const std::size_t index);
  void PercolateDown(const std::size_t index);
};
}  // namespace rdu

#endif /* DYNAMIC_PRIORITY_QUEUE_HPP */
