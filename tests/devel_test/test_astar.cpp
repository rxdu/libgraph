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

using namespace rdu;

#define ROW_SIZE 4
#define COL_SIZE 4

struct SimpleState {
  SimpleState() = default;
  SimpleState(int32_t row, int32_t col) : row_(row), col_(col) {}

  int32_t row_;
  int32_t col_;
};

struct SimpleStateIndexer {
  int64_t operator()(SimpleState *state) {
    return state->row_ * COL_SIZE + state->col_;
  }

  int64_t operator()(const SimpleState &state) {
    return state.row_ * COL_SIZE + state.col_;
  }
};

double CalcHeuristicSimpleState(SimpleState *node1, SimpleState *node2) {
  int32_t dist_row = node1->row_ - node2->row_;
  int32_t dist_col = node1->col_ - node2->col_;

  return std::sqrt(dist_row * dist_row + dist_col * dist_col);
}

////////////////////////////////////////////////////////////////////

struct Index {
  int64_t x;
  int64_t y;
};

struct SquareCell {
  SquareCell(int64_t id) : id_(id){};
  ~SquareCell() = default;

  Index idx;
  int64_t id_;

  int64_t GetUniqueID() const { return id_; }
};

double CalcHeuristic(SquareCell node1, SquareCell node2) {
  return std::abs(node1.idx.x - node2.idx.x) +
         std::abs(node1.idx.y - node2.idx.y);
}

// index of square grid: positive x points towards right, positive y points
// upwards
class GetSquareCellNeighbour {
 public:
  GetSquareCellNeighbour(int grid_size_row, int grid_size_col, double size,
                         const std::vector<uint64_t> obstacle_ids)
      : row_size_(grid_size_row),
        col_size_(grid_size_col),
        cell_size_(size),
        obstacle_ids_(obstacle_ids){};

  // define the functor operator
  std::vector<std::tuple<SquareCell, double>> operator()(SquareCell cell) {
    std::vector<std::tuple<SquareCell, double>> adjacent_cells;

    Index pos[4];

    pos[0].x = cell.idx.x;
    pos[0].y = cell.idx.y + 1;

    pos[1].x = cell.idx.x;
    pos[1].y = cell.idx.y - 1;

    pos[2].x = cell.idx.x + 1;
    pos[2].y = cell.idx.y;

    pos[3].x = cell.idx.x - 1;
    pos[3].y = cell.idx.y;

    for (int i = 0; i < 4; i++) {
      if (pos[i].x >= 0 && pos[i].x < col_size_ && pos[i].y >= 0 &&
          pos[i].y < row_size_) {
        uint64_t new_id = pos[i].y * col_size_ + pos[i].x;

        if (std::find(obstacle_ids_.begin(), obstacle_ids_.end(), new_id) !=
            obstacle_ids_.end())
          continue;

        SquareCell ncell(new_id);
        ncell.idx.x = pos[i].x;
        ncell.idx.y = pos[i].y;

        adjacent_cells.emplace_back(ncell, cell_size_);
      }
    }

    return adjacent_cells;
  }

 private:
  int row_size_;
  int col_size_;
  double cell_size_;
  std::vector<uint64_t> obstacle_ids_;
};

////////////////////////////////////////////////////////////////////

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

  // Dijkstra search
  std::cout << "\nA* search: " << std::endl;
  //   auto path = Dijkstra::PerformSearch(&graph, graph.FindVertex(0),
  //                                       graph.FindVertex(13));
  // for (auto &e : path) std::cout << "id: " << e->vertex_id << std::endl;

  auto path = AStar::Search(
      &graph, 0, 13,
      CalcHeuristicFunc_t<SimpleState *>(CalcHeuristicSimpleState));
  for (auto &e : path)
    std::cout << "id: " << SimpleStateIndexer()(e) << std::endl;

  //   auto path2 = AStar::Search(&graph, 0, 13);
  //   for (auto &e : path2)
  //     std::cout << "id2: " << SimpleStateIndexer()(e) << std::endl;

  // need to delete all nodes, the graph only maintains pointers to these nodes
  for (auto &e : nodes) delete e;

  //---------------------------------------------------------------------

  SquareCell cell_s(0);
  cell_s.idx.x = 0;
  cell_s.idx.y = 0;

  SquareCell cell_g(24);
  cell_g.idx.x = 4;
  cell_g.idx.y = 4;

  std::vector<uint64_t> obstacle_ids;
  obstacle_ids.push_back(5);
  obstacle_ids.push_back(6);
  obstacle_ids.push_back(7);
  obstacle_ids.push_back(17);
  obstacle_ids.push_back(18);
  obstacle_ids.push_back(19);

  Graph<SquareCell, double> search_graph;
  auto start_vtx = search_graph.AddVertex(cell_s);
  auto goal_vtx = search_graph.AddVertex(cell_g);
  auto find_neighbours = GetSquareCellNeighbour(5, 5, 1.0, obstacle_ids);
  //   auto path_i =
  //       Dijkstra::PerformSearch(&search_graph, start_vtx, goal_vtx,
  // GetNeighbourFunc_t<SquareCell>(find_neighbours));

  //   std::cout << "Inc dijkstra search: " << std::endl;
  //   for (auto &e : path_i) std::cout << "id: " << e->vertex_id <<
  //   std::endl;

  //---------------------------------------------------------------------

  Graph<SquareCell, double> sgraph;
  auto path_i2 = AStar::IncSearch(&sgraph, cell_s, cell_g,
                               CalcHeuristicFunc_t<SquareCell>(CalcHeuristic),
                               GetNeighbourFunc_t<SquareCell>(find_neighbours));

  std::cout << "Inc A* search2: " << std::endl;
  for (auto &e : path_i2) std::cout << "id: " << e.GetUniqueID() << std::endl;

  return 0;
}
