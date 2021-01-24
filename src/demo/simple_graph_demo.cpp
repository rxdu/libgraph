/*
 * basic_example.cpp
 *
 * Created on: Nov 22, 2017 12:03
 * Description:
 *
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

// standard libaray
#include <iostream>
#include <vector>
#include <ctime>
#include <cmath>

// user
#include "graph/graph.hpp"
#include "graph/search/astar.hpp"
#include "graph/search/dijkstra.hpp"

using namespace rdu;

#define ROW_SIZE 4
#define COL_SIZE 4

struct SimpleState {
  SimpleState() = default;
  SimpleState(int32_t row, int32_t col) : row(row), col(col) {}

  int32_t row;
  int32_t col;
};

struct SimpleStateIndexer {
  int64_t operator()(SimpleState *state) {
    return state->row * COL_SIZE + state->col;
  }

  int64_t operator()(const SimpleState &state) {
    return state.row * COL_SIZE + state.col;
  }
};

double CalcHeuristic(SimpleState *node1, SimpleState *node2) {
  int32_t dist_row = node1->row - node2->row;
  int32_t dist_col = node1->col - node2->col;

  return std::sqrt(dist_row * dist_row + dist_col * dist_col);
}

int main(int argc, char **argv) {
  SimpleStateIndexer indexer;

  std::vector<SimpleState *> nodes;

  // create nodes
  for (int i = 0; i < ROW_SIZE; i++)
    for (int j = 0; j < COL_SIZE; j++) nodes.push_back(new SimpleState(i, j));

  // create a graph
  Graph<SimpleState *, double, SimpleStateIndexer> graph;

  graph.AddEdge(nodes[0], nodes[1], 1.0);
  graph.AddEdge(nodes[1], nodes[0], 1.0);
  graph.AddEdge(nodes[1], nodes[2], 1.0);
  graph.AddEdge(nodes[1], nodes[6], 1.414);
  graph.AddEdge(nodes[2], nodes[1], 1.0);
  graph.AddEdge(nodes[2], nodes[3], 1.0);
  graph.AddEdge(nodes[2], nodes[6], 1.0);
  graph.AddEdge(nodes[3], nodes[2], 1.0);
  graph.AddEdge(nodes[3], nodes[6], 1.414);
  graph.AddEdge(nodes[6], nodes[1], 1.414);
  graph.AddEdge(nodes[6], nodes[2], 1.0);
  graph.AddEdge(nodes[6], nodes[3], 1.414);
  graph.AddEdge(nodes[6], nodes[9], 1.414);
  graph.AddEdge(nodes[6], nodes[10], 1.0);
  graph.AddEdge(nodes[9], nodes[6], 1.414);
  graph.AddEdge(nodes[9], nodes[10], 1.0);
  graph.AddEdge(nodes[9], nodes[13], 1.0);
  graph.AddEdge(nodes[10], nodes[6], 1.0);
  graph.AddEdge(nodes[10], nodes[9], 1.0);
  graph.AddEdge(nodes[10], nodes[13], 1.414);
  graph.AddEdge(nodes[13], nodes[9], 1.0);
  graph.AddEdge(nodes[13], nodes[10], 1.414);

  auto all_edges = graph.GetAllEdges();

  for (auto &e : all_edges) e->PrintEdge();

  // In order to use A* search, you need to specify how to calculate heuristic
  std::cout << "\nA* search: " << std::endl;
  auto path_a = AStar::Search(
      &graph, 0, 13, CalcHeuristicFunc_t<SimpleState *>(CalcHeuristic));
  for (auto &e : path_a) std::cout << "id: " << indexer(e) << std::endl;

  // Dijkstra search
  std::cout << "\nDijkstra search: " << std::endl;
  auto path_d = Dijkstra::Search(&graph, 0, 13);
  for (auto &e : path_d) std::cout << "id: " << indexer(e) << std::endl;

  // need to delete all nodes, the graph only maintains pointers to these nodes
  for (auto &e : nodes) delete e;

  return 0;
}
