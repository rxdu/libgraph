/*
 * graph_search_inc_test.cpp
 *
 * Created on: Mar 14, 2018 14:05
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include <stdio.h>
#include <vector>

#include "gtest/gtest.h"

#include "graph/graph.hpp"
#include "graph/search/astar.hpp"
#include "graph/search/dijkstra.hpp"

using namespace rdu;

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
                         const std::vector<int64_t> obstacle_ids)
      : row_size_(grid_size_row),
        col_size_(grid_size_col),
        cell_size_(size),
        obstacle_ids_(obstacle_ids){};

 private:
  int row_size_;
  int col_size_;
  double cell_size_;
  std::vector<int64_t> obstacle_ids_;

 public:
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
        int64_t new_id = pos[i].y * col_size_ + pos[i].x;

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
};

struct GraphIncSearchTest : testing::Test {
  GraphIncSearchTest() : cell_s(SquareCell(0)), cell_g(SquareCell(24)) {
    cell_s.idx.x = 0;
    cell_s.idx.y = 0;

    cell_g.idx.x = 4;
    cell_g.idx.y = 4;

    obstacle_ids.push_back(5);
    obstacle_ids.push_back(6);
    obstacle_ids.push_back(7);
    obstacle_ids.push_back(17);
    obstacle_ids.push_back(18);
    obstacle_ids.push_back(19);

    spath.push_back(0);
    spath.push_back(1);
    spath.push_back(2);
    spath.push_back(3);
    spath.push_back(8);
    spath.push_back(13);
    spath.push_back(12);
    spath.push_back(11);
    spath.push_back(16);
    spath.push_back(21);
    spath.push_back(22);
    spath.push_back(23);
    spath.push_back(24);
  }

  virtual ~GraphIncSearchTest() = default;

  SquareCell cell_s;
  SquareCell cell_g;
  std::vector<int64_t> obstacle_ids;
  std::vector<int64_t> spath;
};

TEST_F(GraphIncSearchTest, IncDijkstra) {
  Graph<SquareCell, double> sgraph;
  auto find_neighbours = GetSquareCellNeighbour(5, 5, 1.0, obstacle_ids);
  auto path = Dijkstra::IncSearch(
      &sgraph, cell_s, cell_g, GetNeighbourFunc_t<SquareCell>(find_neighbours));

  std::vector<int64_t> path_ids;
  for (auto &e : path) path_ids.push_back(e.GetUniqueID());

  std::cout << "dijkstra ids: " << std::endl;
  for (auto &e : path) std::cout << e.GetUniqueID() << " ";
  std::cout << std::endl;

  std::cout << "path dijkstra: " << std::endl;
  for (auto &e : path) std::cout << e.id_ << " ";
  std::cout << std::endl;

  std::cout << "path expected: " << std::endl;
  for (auto &e : spath) std::cout << e << " ";
  std::cout << std::endl;

  ASSERT_TRUE(path_ids == spath) << "Path found by incremental Dijkstra in "
                                    "value-type graph is not correct";
}

TEST_F(GraphIncSearchTest, IncAStar) {
  Graph<SquareCell, double> sgraph;
  auto path = AStar::IncSearch(
      &sgraph, cell_s, cell_g, CalcHeuristicFunc_t<SquareCell>(CalcHeuristic),
      GetNeighbourFunc_t<SquareCell>(
          GetSquareCellNeighbour(5, 5, 1.0, obstacle_ids)));
  std::vector<int64_t> path_ids;
  for (auto &e : path) path_ids.push_back(e.GetUniqueID());

  ASSERT_TRUE(path_ids == spath)
      << "Path found by incremental A* in value-type graph is not correct";
}
