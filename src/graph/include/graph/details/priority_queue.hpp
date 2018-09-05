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

namespace librav
{
template <typename T>
struct PQElementComparator
{
	bool operator()(const T &lhs, const T &rhs) const
	{
		return (lhs.first > rhs.first);
	}
};

/// A simple priority queue implementation.
// Source: http://www.redblobgames.com/pathfinding/a-star/implementation.html
template <typename T, typename V = double>
struct PriorityQueue
{
	typedef std::pair<V, T> PQElement;

	// std::priority_queue<PQElement, std::vector<PQElement>,
	// 					std::greater<PQElement>>
	// 	elements;
	std::priority_queue<PQElement, std::vector<PQElement>,
						PQElementComparator<PQElement>>
		elements;

	inline bool empty() const { return elements.empty(); }

	inline size_t size() const { return elements.size(); }

	inline void put(T item, V priority)
	{
		elements.emplace(priority, item);
	}

	inline T get()
	{
		T best_item = elements.top().second;
		elements.pop();
		return best_item;
	}
};
} // namespace librav

#endif /* PRIORITY_QUEUE_HPP */
