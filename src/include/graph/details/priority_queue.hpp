/*
 * priority_queue.hpp
 *
 * Created on: Nov 30, 2017 11:41
 * Description:
 *
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef PRIORITY_QUEUE_HPP
#define PRIORITY_QUEUE_HPP

#include <utility>
#include <queue>

namespace rdu {
template <typename T>
struct PQElementComparator {
  bool operator()(const T &lhs, const T &rhs) const {
    return (lhs.first > rhs.first);
  }
};

/// A simple priority queue implementation.
// Source: http://www.redblobgames.com/pathfinding/a-star/implementation.html
template <typename T, typename V = double>
class PriorityQueue {
  typedef std::pair<V, T> PQElement;
  std::priority_queue<PQElement, std::vector<PQElement>,
                      PQElementComparator<PQElement>>
      elements;

 public:
  inline void Push(T item, V priority) { elements.emplace(priority, item); }

  inline T Pop() {
    T best_item = elements.top().second;
    elements.pop();
    return best_item;
  }

  inline bool Empty() const { return elements.empty(); }

  inline size_t GetQueueElementNumber() const { return elements.size(); }
};
}  // namespace rdu

#endif /* PRIORITY_QUEUE_HPP */
