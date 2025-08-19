/*
 * inc_search_demo.cpp
 *
 *  Created on: Nov 14, 2016
 *      Author: rdu
 *
 *  Description: demo on how to incrementally build and search a graph
 *
 */

// standard libaray
#include <iostream>
#include <vector>
#include <tuple>
#include <cmath>
#include <cstdint>
#include <algorithm>

// user
#include "graph/graph.hpp"
#include "graph/search/astar.hpp"
#include "graph/search/dijkstra.hpp"

using namespace xmotion;

struct Index {
  int64_t x;
  int64_t y;
};

struct SquareCell {
  SquareCell(int64_t id) : id(id){};
  ~SquareCell() = default;

  Index idx;
  int64_t id;

  int64_t GetUniqueID() const { return id; }
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

 private:
  int row_size_;
  int col_size_;
  double cell_size_;
  std::vector<uint64_t> obstacle_ids_;

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
};

int main(int argc, char **argv) {
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

  auto find_neighbours = GetSquareCellNeighbour(5, 5, 1.0, obstacle_ids);

  Graph<SquareCell, double> sgraph1;
  // Build graph manually (incremental search can be simulated by building graph step by step)
  sgraph1.AddVertex(cell_s);
  sgraph1.AddVertex(cell_g);
  // Add edges based on neighbors (simplified for demo)
  auto neighbors = find_neighbours(cell_s);
  for (const auto& neighbor : neighbors) {
    sgraph1.AddVertex(std::get<0>(neighbor));
    sgraph1.AddEdge(cell_s, std::get<0>(neighbor), std::get<1>(neighbor));
  }
  auto path = AStar::Search(&sgraph1, cell_s, cell_g, CalcHeuristic);

  Graph<SquareCell, double> sgraph2;
  // Build second graph for Dijkstra comparison
  sgraph2.AddVertex(cell_s);
  sgraph2.AddVertex(cell_g);
  // Add edges based on neighbors (simplified for demo)
  auto neighbors2 = find_neighbours(cell_s);
  for (const auto& neighbor : neighbors2) {
    sgraph2.AddVertex(std::get<0>(neighbor));
    sgraph2.AddEdge(cell_s, std::get<0>(neighbor), std::get<1>(neighbor));
  }
  auto path2 = Dijkstra::Search(&sgraph2, cell_s, cell_g);

  std::cout << "path a*: " << std::endl;
  for (auto &e : path) std::cout << "id: " << e.id << std::endl;
  std::cout << "path dijkstra: " << std::endl;
  for (auto &e : path2) std::cout << "id: " << e.id << std::endl;

  return 0;
}
