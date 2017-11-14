/* 
 * graph.h
 * 
 * Created on: Nov 10, 2017 
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef GRAPH_H
#define GRAPH_H

#include "graph/internal/graph_impl.h"
#include "graph/internal/bds_base.h"

namespace librav
{
    
// Alias ended with "_t" should be used in user applications
template<typename T>
using Graph_t = Graph<T>;

template<typename T>
using Vertex_t = Vertex<T>;

template<typename T>
using Edge_t = Edge<Vertex<T>*>;

template<typename T>
using Path_t = std::vector<Vertex<T>*>;

}

#endif /* GRAPH_H */
