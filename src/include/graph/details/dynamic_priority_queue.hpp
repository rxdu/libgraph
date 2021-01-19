/*
 * dynamic_priority_queue.hpp
 *
 * Created on: Sep 04, 2018 12:10
 * Description:
 * A dynamic array (std::vector) is used to contain elements of the queue.
 * Position 0 of the vector is not used for convenience of indexing:
 *
 * "For any element in array position i, the left child is in position 2i, the
 * right child is in the cell after the left child (2i + 1), and the parent is
 * in position i/2."
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
template <typename T, typename Comparator = std::less<T>,
          typename ItemIndexer = DefaultIndexer<T>>
class DynamicPriorityQueue {
 public:
  DynamicPriorityQueue(std::size_t initial_capacity = 100) {
    array_.resize(initial_capacity);
  }

  void Push(const T& element) {
    std::cout << "pushing: " << element << std::endl;
    // determine if resizing is necessary
    if (element_num_ == array_.size() - 1) array_.resize(array_.size() * 2);

    // check whether element is already in queue
    if (!Contains(element)) {
      PercolateUp(element, ++element_num_);
    } else {
      Update(element);
    }
  }

  void Update(const T& element) {
    // do nothing if element is not in queue
    auto index_entry = element_map_.find(GetItemIndex(element));
    if (index_entry == element_map_.end()) return;
    auto index = index_entry->second;
    if (Compare(element, array_[index])) {
      array_[index] = element;
      PercolateUp(element, index);
    } else {
      array_[index] = element;
      PercolateDown(index);
    }
  }

  bool Empty() const { return (element_num_ == 0); }

  T Peek() const { return array_[0]; }
  void DeleteMin() {}

  T Pop() {
    auto min = array_[0];
    DeleteMin();
    return min;
  }

  void Remove(const T& element) {
    // find element first
  }

  bool Contains(const T& element) const {
    return element_map_.find(GetItemIndex(element)) != element_map_.end();
  }

  void PrintInfo() {
    for (std::size_t i = 1; i < element_num_ + 1; ++i) {
      std::cout << array_[i] << " ";
    }
    std::cout << std::endl;
  }

 private:
  std::size_t element_num_ = 0;
  std::vector<T> array_;
  std::unordered_map<int64_t, std::size_t> element_map_;

  ItemIndexer GetItemIndex;
  Comparator Compare;

  void PercolateUp(const T& element, std::size_t index) {
    T new_element = element;
    // keep floating up until heap-order property is satisfied
    for (; Compare(element, array_[index / 2]); index /= 2) {
      array_[index] = std::move(array_[index / 2]);
    }
    // insert new element
    array_[index] = std::move(new_element);
    element_map_[GetItemIndex(element)] = index;
  }

  void PercolateDown(std::size_t index) {
    std::size_t child;
    T tmp = array_[index];
    // keep sinking down until heap-order property is satisfied
    for (; index * 2 <= element_num_; index = child) {
      child = index * 2;
      // check which child is smaller (if right child exists)
      if (child != element_num_ && Compare(array_[child + 1], array_[child]))
        ++child;
      // float child up if desired (according to Compare())
      if (Compare(array_[child], tmp))
        array_[index] = std::move(array_[child]);
      else
        break;
    }
    // place element at the new location
    array_[index] = std::move(tmp);
  }
};
}  // namespace rdu

#endif /* DYNAMIC_PRIORITY_QUEUE_HPP */
