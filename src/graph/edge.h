/* 
 * edge.h
 * 
 * Created on: Jul 14, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef EDGE_H
#define EDGE_H

#include <iostream>

namespace librav {

/****************************************************************************/
/*								 Edge  										*/
/****************************************************************************/
/// An edge data structure template.
template<typename VertexPtrType>
class Edge
{
public:
	/**
	 * @param src a pointer to the source vertex of the edge
	 * @param dst a pointer to the destination vertex of the edge
	 * @param c cost associated with the edge
	 */
	Edge(VertexPtrType src, VertexPtrType dst, double c = 0.0):
		src_(src),dst_(dst), cost_(c){};
	~Edge() = default;

	VertexPtrType src_;
	VertexPtrType dst_;
	double cost_;

	/**
	 * == operator overloading. If two edges connect the same pair of vertices, they're
	 * regarded as equal.
	 */
	bool operator ==(const Edge<VertexPtrType>& other);

	/**
	 * This operation checks if two edges connect the same vertex pair.
	 * If two edges connect the same pair of vertices, return true, otherwise false.
	 */
	bool operator -=(const Edge<VertexPtrType>& other);

	/**
	 * Print edge information: start vertex id, destination vertex id, edge cost.
	 */
	void PrintEdge() const;
};

}

#include "graph/internal/edge_impl.h"

#endif /* EDGE_H */
