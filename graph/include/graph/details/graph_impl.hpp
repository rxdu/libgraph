/* 
 * graph_impl.hpp
 * 
 * Created on: Sep 04, 2018 01:56
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef GRAPH_IMPL_HPP
#define GRAPH_IMPL_HPP

#include <type_traits>

namespace librav
{
template <typename State, typename Transition, typename StateIndexer>
Graph<State, Transition, StateIndexer>::Graph(const Graph<State, Transition, StateIndexer> &other)
{
    for (auto &pair : other.vertex_map_)
    {
        auto vertex = pair.second;
        for (auto &edge : vertex->edges_to_)
            this->AddEdge(edge.src_->state_, edge.dst_->state_, edge.cost_);
    }
}

template <typename State, typename Transition, typename StateIndexer>
Graph<State, Transition, StateIndexer>::Graph(Graph<State, Transition, StateIndexer> &&other)
{
    vertex_map_ = std::move(other.vertex_map_);
}

template <typename State, typename Transition, typename StateIndexer>
Graph<State, Transition, StateIndexer> &Graph<State, Transition, StateIndexer>::operator=(const Graph<State, Transition, StateIndexer> &other)
{
    Graph<State, Transition, StateIndexer> temp = other;
    std::swap(*this, temp);
    return *this;
}

template <typename State, typename Transition, typename StateIndexer>
Graph<State, Transition, StateIndexer> &Graph<State, Transition, StateIndexer>::operator=(Graph<State, Transition, StateIndexer> &&other)
{
    std::swap(vertex_map_, other.vertex_map_);
    return *this;
}

template <typename State, typename Transition, typename StateIndexer>
Graph<State, Transition, StateIndexer>::~Graph()
{
    for (auto &vertex_pair : vertex_map_)
        delete vertex_pair.second;
};

template <typename State, typename Transition, typename StateIndexer>
typename Graph<State, Transition, StateIndexer>::vertex_iterator Graph<State, Transition, StateIndexer>::AddVertex(State state)
{
    return ObtainVertexFromVertexMap(state);
}

template <typename State, typename Transition, typename StateIndexer>
void Graph<State, Transition, StateIndexer>::RemoveVertex(int64_t state_id)
{
    auto it = vertex_map_.find(state_id);

    // remove if specified vertex exists
    if (it != vertex_map_.end())
    {
        // remove from other vertices that connect to the vertex to be deleted
        auto vtx = vertex_iterator(it);
        for (auto &asv : vtx->vertices_from_)
            for (auto eit = asv->edges_to_.begin(); eit != asv->edges_to_.end(); eit++)
                if ((*eit).dst_ == vtx)
                {
                    asv->edges_to_.erase(eit);
                    break;
                }

        // remove from vertex map
        auto vptr = it->second;
        vertex_map_.erase(it);
        delete vptr;
    }
}

template <typename State, typename Transition, typename StateIndexer>
void Graph<State, Transition, StateIndexer>::AddEdge(State sstate, State dstate, Transition trans)
{
    auto src_vertex = ObtainVertexFromVertexMap(sstate);
    auto dst_vertex = ObtainVertexFromVertexMap(dstate);

    // update transition if edge already exists
    auto it = src_vertex->FindEdge(dstate);
    if (it != src_vertex->edge_end())
    {
        it->cost_ = trans;
        return;
    }

    dst_vertex->vertices_from_.push_back(src_vertex);
    src_vertex->edges_to_.emplace_back(src_vertex, dst_vertex, trans);
}

template <typename State, typename Transition, typename StateIndexer>
bool Graph<State, Transition, StateIndexer>::RemoveEdge(State sstate, State dstate)
{
    auto src_vertex = FindVertex(sstate);
    auto dst_vertex = FindVertex(dstate);

    if ((src_vertex != vertex_end()) && (dst_vertex != vertex_end()))
    {
        for (auto it = src_vertex->edges_to_.begin(); it != src_vertex->edges_to_.end(); ++it)
        {
            if (it->dst_ == dst_vertex)
            {
                src_vertex->edges_to_.erase(it);
                dst_vertex->vertices_from_.erase(std::remove(dst_vertex->vertices_from_.begin(), dst_vertex->vertices_from_.end(), src_vertex), dst_vertex->vertices_from_.end());
                return true;
            }
        }
    }

    return false;
}

template <typename State, typename Transition, typename StateIndexer>
void Graph<State, Transition, StateIndexer>::AddUndirectedEdge(State sstate, State dstate, Transition trans)
{
    AddEdge(sstate, dstate, trans);
    AddEdge(dstate, sstate, trans);
}

template <typename State, typename Transition, typename StateIndexer>
bool Graph<State, Transition, StateIndexer>::RemoveUndirectedEdge(State src_node, State dst_node)
{
    bool edge1 = RemoveEdge(src_node, dst_node);
    bool edge2 = RemoveEdge(dst_node, src_node);

    if (edge1 && edge2)
        return true;
    else
        return false;
}

template <typename State, typename Transition, typename StateIndexer>
std::vector<typename Graph<State, Transition, StateIndexer>::edge_iterator> Graph<State, Transition, StateIndexer>::GetAllEdges() const
{
    std::vector<typename Graph<State, Transition, StateIndexer>::edge_iterator> edges;
    for (auto &vertex_pair : vertex_map_)
    {
        auto vertex = vertex_pair.second;
        for (auto it = vertex->edge_begin(); it != vertex->edge_end(); ++it)
            edges.push_back(it);
    }
    return edges;
}

template <typename State, typename Transition, typename StateIndexer>
void Graph<State, Transition, StateIndexer>::ResetGraphVertices()
{
    for (auto &vertex_pair : vertex_map_)
        vertex_pair.second->ClearVertexSearchInfo();
}

template <typename State, typename Transition, typename StateIndexer>
void Graph<State, Transition, StateIndexer>::ClearGraph()
{
    for (auto &vertex_pair : vertex_map_)
        delete vertex_pair.second;
    vertex_map_.clear();
}

template <typename State, typename Transition, typename StateIndexer>
typename Graph<State, Transition, StateIndexer>::vertex_iterator Graph<State, Transition, StateIndexer>::ObtainVertexFromVertexMap(State state)
{
    int64_t state_id = GetStateIndex(state);
    auto it = vertex_map_.find(state_id);

    if (it == vertex_map_.end())
    {
        auto new_vertex = new Vertex(state, state_id);
        new_vertex->search_parent_ = vertex_end();
        vertex_map_.insert(std::make_pair(state_id, new_vertex));
        return vertex_iterator(vertex_map_.find(state_id));
    }

    return vertex_iterator(it);
}
} // namespace librav

#endif /* GRAPH_IMPL_HPP */
