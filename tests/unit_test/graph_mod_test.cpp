/*
 * graph_mod_test.cpp
 *
 * Created on: Mar 14, 2018 15:37
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include <stdio.h>
#include <vector>
#include <set>
#include <tuple>

#include "gtest/gtest.h"

#include "graph/graph.hpp"

using namespace xmotion;

struct TestState {
  TestState(uint64_t id) : id_(id) {};

  int64_t id_;
};

struct GraphModificationTest : testing::Test {
  GraphModificationTest() {
    for (int i = 0; i < 9; i++) nodes.push_back(new TestState(i));
  }

  virtual ~GraphModificationTest() {
    for (auto &nd : nodes) delete nd;
  }

  std::vector<TestState *> nodes;
};

TEST_F(GraphModificationTest, VertexMod) {
  // create a graph
  Graph<TestState *> graph;

  ASSERT_EQ(graph.GetTotalVertexNumber(), 0)
      << "Graph should have no vertex now";

  graph.AddVertex(nodes[0]);
  graph.AddVertex(nodes[1]);

  ASSERT_EQ(graph.GetTotalVertexNumber(), 2)
      << "Failed to add vertices to pointer-type graph ";

  ASSERT_EQ(graph.FindVertex(0)->vertex_id, 0)
      << "Failed to find added vertex by associated state ID from pointer-type "
         "graph ";
  ASSERT_EQ(graph.FindVertex(nodes[1])->vertex_id, 1)
      << "Failed to find added vertex by associated state from pointer-type "
         "graph ";

  ASSERT_EQ(graph.FindVertex(-1), graph.vertex_end());
  ASSERT_EQ(graph.FindVertex(nodes[2]), graph.vertex_end());

  graph.RemoveVertex(0);

  ASSERT_EQ(graph.GetTotalVertexNumber(), 1)
      << "Failed to remove vertex by associated state ID from pointer-type "
         "graph ";
  ASSERT_TRUE(graph.FindVertex(0) == graph.vertex_end())
      << "Failed to remove vertex by associated state ID from pointer-type "
         "graph ";
  ASSERT_EQ(graph.FindVertex(1)->vertex_id, 1)
      << "Removed wrong vertex by associated state ID from pointer-type graph ";

  graph.RemoveVertex(nodes[1]);

  ASSERT_EQ(graph.GetTotalVertexNumber(), 0)
      << "Failed to remove vertex by associated state from pointer-type graph ";
  ASSERT_TRUE(graph.FindVertex(1) == graph.vertex_end())
      << "Failed to remove vertex by associated state from pointer-type graph ";
}

TEST_F(GraphModificationTest, EdgeMod) {
  Graph<TestState *> graph;

  ASSERT_EQ(graph.GetTotalEdgeNumber(), 0) << "Graph should have no edge now";

  graph.AddEdge(nodes[0], nodes[1], 1.2);
  graph.AddEdge(nodes[1], nodes[2], 1.5);

  // print edges
  {
    auto edges = graph.GetAllEdges();
    for (auto &edge : edges) {
      edge->PrintEdge();
    }
  }

  ASSERT_EQ(graph.GetTotalEdgeNumber(), 2)
      << "Failed to add directed edges to pointer-type graph";
  std::vector<Graph<TestState *>::edge_iterator> edges;
  for (auto it = graph.FindVertex(0)->edge_begin();
       it != graph.FindVertex(0)->edge_end(); ++it)
    edges.push_back(it);
  ASSERT_EQ(edges.size(), 1)
      << "Wrong number of directed edges added to vertex in pointer-type graph";
  ASSERT_EQ(edges.front()->src->vertex_id, 0)
      << "Wrong src of directed edges added to vertex in pointer-type graph";
  ASSERT_EQ(edges.front()->dst->vertex_id, 1)
      << "Wrong dst of directed edges added to vertex in pointer-type graph";
  ASSERT_EQ(edges.front()->cost, 1.2)
      << "Wrong cost of directed edges added to vertex in pointer-type graph";

  ASSERT_EQ(graph.FindVertex(nodes[1])->edges_to.size(), 1)
      << "Failed to add edge to vertex";
  ASSERT_EQ(graph.FindVertex(nodes[2])->vertices_from.size(), 1)
      << "Failed to maintain list of vertices_from";

  graph.RemoveEdge(nodes[1], nodes[2]);

  ASSERT_EQ(graph.FindVertex(nodes[1])->edges_to.size(), 0)
      << "Failed to remove edge frome vertex";
  ASSERT_EQ(graph.FindVertex(nodes[2])->vertices_from.size(), 0)
      << "Failed to maintain list of vertices_from";

  ASSERT_EQ(graph.GetTotalEdgeNumber(), 1)
      << "Failed to remove a directed edge from pointer-type graph";

  // remove non-existing edge
  graph.RemoveEdge(nodes[1], nodes[3]);
  ASSERT_EQ(graph.GetTotalEdgeNumber(), 1);

  graph.RemoveUndirectedEdge(nodes[1], nodes[3]);
  ASSERT_EQ(graph.GetTotalEdgeNumber(), 1);

  edges.clear();
  for (auto it = graph.FindVertex(0)->edge_begin();
       it != graph.FindVertex(0)->edge_end(); ++it)
    edges.push_back(it);
  bool edge_intact =
      (edges.size() == 1) && (edges.front()->src->vertex_id == 0) &&
      (edges.front()->dst->vertex_id == 1) && (edges.front()->cost == 1.2);
  ASSERT_TRUE(edge_intact) << "A wrong edge is removed from pointer-type graph";

  graph.AddUndirectedEdge(nodes[3], nodes[4], 1.8);
  graph.AddUndirectedEdge(nodes[4], nodes[5], 2.0);
  ASSERT_EQ(graph.GetTotalEdgeNumber(), 5)
      << "Failed to add a undirected edge from pointer-type graph";

  graph.RemoveUndirectedEdge(nodes[4], nodes[5]);
  ASSERT_EQ(graph.GetTotalEdgeNumber(), 3)
      << "Failed to remove a undirected edge from pointer-type graph";
}

TEST_F(GraphModificationTest, RemoveVertexWithEdge) {
  Graph<TestState *> graph;

  ASSERT_EQ(graph.GetTotalEdgeNumber(), 0) << "Graph should have no edge now";

  graph.AddEdge(nodes[0], nodes[1], 1.2);
  graph.AddEdge(nodes[1], nodes[2], 1.5);

  graph.RemoveVertex(2);
  ASSERT_EQ(graph.GetTotalVertexNumber(), 2);
  ASSERT_EQ(graph.GetTotalEdgeNumber(), 1);

  graph.AddEdge(nodes[1], nodes[2], 1.5);
  graph.AddEdge(nodes[1], nodes[3], 1.5);
  graph.RemoveVertex(1);
  ASSERT_EQ(graph.GetTotalVertexNumber(), 3);
  ASSERT_EQ(graph.GetTotalEdgeNumber(), 0);

  graph.AddEdge(nodes[0], nodes[1], 1.2);
  graph.AddEdge(nodes[1], nodes[2], 1.5);
  graph.AddEdge(nodes[1], nodes[3], 1.5);
  graph.RemoveVertex(2);
  //   ASSERT_EQ(graph.GetTotalVertexNumber(), 3);
  //   ASSERT_EQ(graph.GetTotalEdgeNumber(), 2);
}

TEST_F(GraphModificationTest, ClearVertexEdge) {
  Graph<TestState *> graph;

  ASSERT_EQ(graph.GetTotalVertexNumber(), 0)
      << "Graph should have no vertex at beginning";
  ASSERT_EQ(graph.GetTotalEdgeNumber(), 0)
      << "Graph should have no edge at beginning";

  graph.AddEdge(nodes[0], nodes[1], 1.2);
  graph.AddEdge(nodes[1], nodes[2], 1.5);

  ASSERT_TRUE(graph.GetTotalVertexNumber() == 3 &&
              graph.GetTotalEdgeNumber() == 2)
      << "Graph should have some vertices and edges now";

  graph.ClearAll();

  ASSERT_TRUE(graph.GetTotalVertexNumber() == 0 &&
              graph.GetTotalEdgeNumber() == 0)
      << "Graph should be empty now";
}

TEST_F(GraphModificationTest, ConvenienceMethodsVertexQueries) {
  Graph<TestState *> graph;
  
  // Test HasVertex
  ASSERT_FALSE(graph.HasVertex(0)) << "Empty graph should have no vertices";
  
  graph.AddVertex(nodes[0]);
  graph.AddVertex(nodes[1]);
  graph.AddVertex(nodes[2]);
  
  ASSERT_TRUE(graph.HasVertex(0)) << "Graph should have vertex 0";
  ASSERT_TRUE(graph.HasVertex(1)) << "Graph should have vertex 1";
  ASSERT_FALSE(graph.HasVertex(5)) << "Graph should not have vertex 5";
  ASSERT_TRUE(graph.HasVertex(nodes[0])) << "Graph should have vertex by state";
  
  // Test GetVertex
  auto* vertex = graph.GetVertex(0);
  ASSERT_NE(vertex, nullptr) << "GetVertex should return valid pointer";
  ASSERT_EQ(vertex->vertex_id, 0) << "Vertex should have correct ID";
  
  auto* null_vertex = graph.GetVertex(10);
  ASSERT_EQ(null_vertex, nullptr) << "GetVertex should return nullptr for non-existent vertex";
  
  // Test STL-like interface
  ASSERT_FALSE(graph.empty()) << "Graph with vertices should not be empty";
  ASSERT_EQ(graph.size(), 3) << "Graph should have 3 vertices";
}

TEST_F(GraphModificationTest, ConvenienceMethodsDegrees) {
  Graph<TestState *> graph;
  
  // Create a simple directed graph
  // 0 -> 1 -> 2
  // |    |
  // v    v
  // 3    4
  graph.AddEdge(nodes[0], nodes[1], 1.0);
  graph.AddEdge(nodes[0], nodes[3], 2.0);
  graph.AddEdge(nodes[1], nodes[2], 1.5);
  graph.AddEdge(nodes[1], nodes[4], 2.5);
  
  // Test out-degree
  ASSERT_EQ(graph.GetOutDegree(0), 2) << "Vertex 0 should have out-degree 2";
  ASSERT_EQ(graph.GetOutDegree(1), 2) << "Vertex 1 should have out-degree 2";
  ASSERT_EQ(graph.GetOutDegree(2), 0) << "Vertex 2 should have out-degree 0";
  ASSERT_EQ(graph.GetOutDegree(3), 0) << "Vertex 3 should have out-degree 0";
  
  // Test in-degree
  ASSERT_EQ(graph.GetInDegree(0), 0) << "Vertex 0 should have in-degree 0";
  ASSERT_EQ(graph.GetInDegree(1), 1) << "Vertex 1 should have in-degree 1";
  ASSERT_EQ(graph.GetInDegree(2), 1) << "Vertex 2 should have in-degree 1";
  ASSERT_EQ(graph.GetInDegree(3), 1) << "Vertex 3 should have in-degree 1";
  
  // Test total degree
  ASSERT_EQ(graph.GetVertexDegree(0), 2) << "Vertex 0 should have total degree 2";
  ASSERT_EQ(graph.GetVertexDegree(1), 3) << "Vertex 1 should have total degree 3";
  ASSERT_EQ(graph.GetVertexDegree(2), 1) << "Vertex 2 should have total degree 1";
  
  // Test non-existent vertex
  ASSERT_EQ(graph.GetVertexDegree(10), 0) << "Non-existent vertex should have degree 0";
}

TEST_F(GraphModificationTest, ConvenienceMethodsNeighbors) {
  Graph<TestState *> graph;
  
  graph.AddEdge(nodes[0], nodes[1], 1.0);
  graph.AddEdge(nodes[0], nodes[2], 2.0);
  graph.AddEdge(nodes[0], nodes[3], 3.0);
  
  auto neighbors = graph.GetNeighbors(nodes[0]);
  ASSERT_EQ(neighbors.size(), 3) << "Vertex 0 should have 3 neighbors";
  
  // Check that all expected neighbors are present
  std::set<int64_t> neighbor_ids;
  for (auto* neighbor : neighbors) {
    neighbor_ids.insert(neighbor->id_);
  }
  ASSERT_TRUE(neighbor_ids.count(1)) << "Should have neighbor 1";
  ASSERT_TRUE(neighbor_ids.count(2)) << "Should have neighbor 2";
  ASSERT_TRUE(neighbor_ids.count(3)) << "Should have neighbor 3";
  
  auto no_neighbors = graph.GetNeighbors(nodes[1]);
  ASSERT_EQ(no_neighbors.size(), 0) << "Vertex 1 should have no neighbors";
}

TEST_F(GraphModificationTest, ConvenienceMethodsEdgeQueries) {
  Graph<TestState *> graph;
  
  graph.AddEdge(nodes[0], nodes[1], 1.5);
  graph.AddEdge(nodes[1], nodes[2], 2.5);
  
  // Test HasEdge
  ASSERT_TRUE(graph.HasEdge(nodes[0], nodes[1])) << "Should have edge 0->1";
  ASSERT_FALSE(graph.HasEdge(nodes[1], nodes[0])) << "Should not have edge 1->0";
  ASSERT_TRUE(graph.HasEdge(nodes[1], nodes[2])) << "Should have edge 1->2";
  ASSERT_FALSE(graph.HasEdge(nodes[0], nodes[2])) << "Should not have edge 0->2";
  
  // Test GetEdgeWeight
  ASSERT_DOUBLE_EQ(graph.GetEdgeWeight(nodes[0], nodes[1]), 1.5);
  ASSERT_DOUBLE_EQ(graph.GetEdgeWeight(nodes[1], nodes[2]), 2.5);
  ASSERT_DOUBLE_EQ(graph.GetEdgeWeight(nodes[1], nodes[0]), 0.0); // Default for non-existent
  
  // Test GetEdgeCount
  ASSERT_EQ(graph.GetEdgeCount(), 2) << "Should have 2 edges";
  graph.AddEdge(nodes[2], nodes[3], 3.5);
  ASSERT_EQ(graph.GetEdgeCount(), 3) << "Should have 3 edges after addition";
}

TEST_F(GraphModificationTest, BatchOperations) {
  Graph<TestState *> graph;
  
  // Test batch vertex addition
  std::vector<TestState*> batch_nodes = {nodes[0], nodes[1], nodes[2], nodes[3]};
  graph.AddVertices(batch_nodes);
  ASSERT_EQ(graph.size(), 4) << "Should have 4 vertices after batch add";
  
  // Test batch edge addition
  std::vector<std::tuple<TestState*, TestState*, double>> edges = {
    std::make_tuple(nodes[0], nodes[1], 1.0),
    std::make_tuple(nodes[1], nodes[2], 2.0),
    std::make_tuple(nodes[2], nodes[3], 3.0)
  };
  graph.AddEdges(edges);
  ASSERT_EQ(graph.GetEdgeCount(), 3) << "Should have 3 edges after batch add";
  
  // Test batch vertex removal
  std::vector<TestState*> to_remove = {nodes[1], nodes[3]};
  graph.RemoveVertices(to_remove);
  ASSERT_EQ(graph.size(), 2) << "Should have 2 vertices after batch remove";
  ASSERT_FALSE(graph.HasVertex(1)) << "Vertex 1 should be removed";
  ASSERT_FALSE(graph.HasVertex(3)) << "Vertex 3 should be removed";
}

TEST_F(GraphModificationTest, VertexAccessEdge) {
  Graph<TestState *> graph;

  graph.AddEdge(nodes[0], nodes[1], 1.2);
  graph.AddEdge(nodes[0], nodes[2], 1.5);
  graph.AddEdge(nodes[0], nodes[3], 1.8);
  graph.AddEdge(nodes[0], nodes[3], 2.0);

  std::vector<int64_t> nc = {1, 2, 3};

  auto neighbours = graph.FindVertex(0)->GetNeighbours();
  std::vector<int64_t> nids2;
  for (auto &n : neighbours) nids2.push_back(n->vertex_id);
  ASSERT_TRUE(nids2 == nc)
      << "Graph should have 3 neighbors (checked from vertex pointer)";

  auto nbs = graph.FindVertex(0)->GetNeighbours();
  std::vector<int64_t> nids;
  for (auto &nb : nbs) nids.push_back(nb->vertex_id);
  ASSERT_TRUE(nids.size() == 3) << "Graph should have 3 neighbors";
  ASSERT_TRUE(nids == nc) << "Graph should have 3 neighbors";

  auto edge_cost1 = graph.FindVertex(0)->FindEdge(1)->cost;
  auto edge_cost2 = graph.FindVertex(0)->FindEdge(2)->cost;
  auto edge_cost3 = graph.FindVertex(0)->FindEdge(3)->cost;

  ASSERT_TRUE(edge_cost1 == 1.2) << "Edge cost to vertex 1 should be 1.2";
  ASSERT_TRUE(edge_cost2 == 1.5) << "Edge cost to vertex 2 should be 1.5";
  ASSERT_TRUE(edge_cost3 == 2.0) << "Edge cost to vertex 3 should be 2.0";

  // try to find non-existing edge
  ASSERT_EQ(graph.FindVertex(0)->FindEdge(4), graph.FindVertex(0)->edge_end());
  ASSERT_EQ(graph.FindVertex(0)->FindEdge(nodes[4]),
            graph.FindVertex(0)->edge_end());

  bool check_neighbour = graph.FindVertex(0)->CheckNeighbour(1) &&
                         graph.FindVertex(0)->CheckNeighbour(2) &&
                         graph.FindVertex(0)->CheckNeighbour(3);
  ASSERT_TRUE(check_neighbour) << "Vertex 0 and 1,2,3 should be neighbours";
  ASSERT_FALSE(graph.FindVertex(0)->CheckNeighbour(4));
}
