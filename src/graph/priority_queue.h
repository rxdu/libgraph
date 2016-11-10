/*
 * priority_queue.h
 *
 *  Created on: Feb 2, 2016
 *      Author: rdu
 */

#ifndef SRC_GRAPH_PRIORITY_QUEUE_H_
#define SRC_GRAPH_PRIORITY_QUEUE_H_

#include <vector>

namespace srcl_ctrl {

// TODO to be finished, not completed yet
template<typename Comparable>
class PriorityQueue {
public:
	PriorityQueue();
	~PriorityQueue();

private:

public:
	bool IsEmpty( ) const;
	const Comparable & FindMin( ) const;

	void Insert(const Comparable& x);
	void DeleteMin( );
	void DeleteMin( Comparable & minItem );
	void MakeEmpty( );

private:
	int	size_;  // Number of elements in heap
	std::vector<Comparable> elements_;    // The heap array

	void buildHeap( );
	void percolateDown( int hole );
};

}

#endif /* SRC_GRAPH_PRIORITY_QUEUE_H_ */
