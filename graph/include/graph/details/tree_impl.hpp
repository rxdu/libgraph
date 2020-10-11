/* 
 * tree_impl.hpp
 * 
 * Created on: Dec 30, 2018 07:36
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TREE_IMPL_HPP
#define TREE_IMPL_HPP

#include <type_traits>

namespace librav
{
template <typename State, typename Transition, typename StateIndexer>
typename Tree<State, Transition, StateIndexer>::vertex_iterator Tree<State, Transition, StateIndexer>::AddVertex(State state)
{
    // only add root vertex if tree is empty
    if (!TreeType::vertex_map_.empty())
        return TreeType::vertex_end();
    root_ = TreeType::ObtainVertexFromVertexMap(state);
    return root_;
}

template <typename State, typename Transition, typename StateIndexer>
int32_t Tree<State, Transition, StateIndexer>::GetVertexDepth(int64_t state_id)
{
    auto vtx = TreeType::FindVertex(state_id);

    if (vtx != TreeType::vertex_end())
    {
        int32_t depth = 0;
        auto parent = vtx->vertices_from_;
        while (!parent.empty())
        {
            ++depth;
            parent = parent.front()->vertices_from_;
        }
        return depth;
    }

    return -1;
}

template <typename State, typename Transition, typename StateIndexer>
typename Tree<State, Transition, StateIndexer>::vertex_iterator Tree<State, Transition, StateIndexer>::GetParentVertex(int64_t state_id)
{
    auto vtx = TreeType::FindVertex(state_id);

    assert((vtx != TreeType::vertex_end()) && (vtx->vertices_from_.size() <= 1));

    if (vtx == root_)
        return TreeType::vertex_end();
    else
        return vtx->vertices_from_.front();
}

template <typename State, typename Transition, typename StateIndexer>
void Tree<State, Transition, StateIndexer>::RemoveSubtree(int64_t state_id)
{
    auto vtx = TreeType::FindVertex(state_id);

    // remove if specified vertex exists
    if (vtx != TreeType::vertex_end())
    {
        // remove from other vertices that connect to the vertex to be deleted
        for (auto &asv : vtx->vertices_from_)
            for (auto eit = asv->edges_to_.begin(); eit != asv->edges_to_.end(); eit++)
                if ((*eit).dst_ == vtx)
                {
                    asv->edges_to_.erase(eit);
                    break;
                }

        // remove all subsequent vertices
        std::vector<vertex_iterator> child_vertices;
        child_vertices.push_back(vtx);
        std::vector<vertex_iterator> direct_children = vtx->GetNeighbours();
        while (!direct_children.empty())
        {
            // add direct children
            child_vertices.insert(child_vertices.end(), direct_children.begin(), direct_children.end());
            std::vector<vertex_iterator> all_children;
            for (auto &vtx : direct_children)
            {
                auto chds = vtx->GetNeighbours();
                if (!chds.empty())
                    all_children.insert(all_children.end(), chds.begin(), chds.end());
            }
            direct_children = all_children;
        }
        for (auto &vtx : child_vertices)
        {
            // remove from vertex map
            auto vptr = TreeType::vertex_map_[vtx->GetVertexID()];
            TreeType::vertex_map_.erase(vtx);
            delete vptr;
        }
    }
}

template <typename State, typename Transition, typename StateIndexer>
void Tree<State, Transition, StateIndexer>::AddEdge(State sstate, State dstate, Transition trans)
{
    bool tree_empty = TreeType::vertex_map_.empty();

    auto src_vertex = TreeType::ObtainVertexFromVertexMap(sstate);
    auto dst_vertex = TreeType::ObtainVertexFromVertexMap(dstate);

    // set root if tree is empty or a parent vertex is connected to root_
    if (tree_empty || (dst_vertex == root_))
        root_ = src_vertex;

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

// template <typename State, typename Transition, typename StateIndexer>
// bool Tree<State, Transition, StateIndexer>::RemoveEdge(State sstate, State dstate)
// {
//     auto src_vertex = TreeType::FindVertex(sstate);
//     auto dst_vertex = TreeType::FindVertex(dstate);

//     if ((src_vertex != TreeType::vertex_end()) && (dst_vertex != TreeType::vertex_end()))
//     {
//         for (auto it = src_vertex->edges_to_.begin(); it != src_vertex->edges_to_.end(); ++it)
//         {
//             if (it->dst_ == dst_vertex)
//             {
//                 src_vertex->edges_to_.erase(it);
//                 dst_vertex->vertices_from_.erase(std::remove(dst_vertex->vertices_from_.begin(), dst_vertex->vertices_from_.end(), src_vertex), dst_vertex->vertices_from_.end());
//                 // RemoveSubtree(dst_vertex->GetVertexID());
//                 return true;
//             }
//         }
//     }

//     return false;
// }

template <typename State, typename Transition, typename StateIndexer>
void Tree<State, Transition, StateIndexer>::ClearAll()
{
    for (auto &vertex_pair : TreeType::vertex_map_)
        delete vertex_pair.second;
    TreeType::vertex_map_.clear();
    root_ = TreeType::vertex_end();
}
} // namespace librav

#endif /* TREE_IMPL_HPP */
