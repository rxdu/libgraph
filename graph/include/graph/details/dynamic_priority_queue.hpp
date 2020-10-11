/* 
 * dynamic_priority_queue.hpp
 * 
 * Created on: Sep 04, 2018 12:10
 * Description: 
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
#include "graph/details/default_comparator.hpp"

namespace librav
{
/// A priority queue implementation that supports element priority update.
// Reference:
// [1] https://github.com/csbence/DynamicPriorityQueue
template <typename Item, typename Comparator = DefaultComparator<Item>, typename ItemIndexer = DefaultIndexer<Item>>
class DynamicPriorityQueue
{
public:
  inline void push(Item item)
  {
    int64_t idx = GetItemIndex(item);
    auto it = data_.find(idx);
    // insert
    if (it == data_.end())
    {
      data_.insert(std::make_pair(idx, item));

      // if (data_.size() == 2)
      // {
      //   std::make_heap(data_.begin(), data_.end(), comparator);
      // }
    }
    // update
    else
    {
      it->second = item;
    }
  };

  void make()
  {
    // std::make_heap(data_.begin(), data_.end(), Comparator());
    std::make_heap(raw_data_.begin(), raw_data_.end(), Comparator());
  }

  inline Item get(){};

  inline bool empty() const { return data_.empty(); }

  void print_queue()
  {
    for (auto &pair : data_)
      std::cout << "id: " << pair.first << " , value: " << pair.second->value_ << std::endl;
  }

private:
  std::vector<Item> raw_data_;
  std::unordered_map<int64_t, Item> data_;
  ItemIndexer GetItemIndex;
};
} // namespace librav

#endif /* DYNAMIC_PRIORITY_QUEUE_HPP */
