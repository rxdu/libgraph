/*
 * performance_test.cpp
 *
 *  Created on: Nov 15, 2016
 *      Author: rdu
 */

// standard libaray
#include <iostream>
#include <vector>
#include <tuple>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <ctime>

// user
#include "graph/astar.h"

using namespace rdu;

#define GRID_SIZE 100

typedef struct
{
	int64_t x;
	int64_t y;
} Index;

struct SquareCell: public BDSBase<SquareCell>
{
	SquareCell(uint64_t id):
		BDSBase<SquareCell>(id){};
	~SquareCell(){};

	Index idx;

	double GetHeuristic(const SquareCell& other_struct) const {
		return std::abs(idx.x - other_struct.idx.x) + std::abs(idx.y - other_struct.idx.y);
	}
};

// index of square grid: positive x points towards right, positive y points upwards
class GetSquareCellNeighbour
{
public:
	GetSquareCellNeighbour(int grid_size_row, int grid_size_col, double size, const std::vector<uint64_t> obstacle_ids):
		row_size_(grid_size_row),
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
	std::vector<std::tuple<SquareCell, double>> operator()(SquareCell cell)
	{
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

		for(int i = 0; i < 4; i++)
		{
			if(pos[i].x >= 0 && pos[i].x < col_size_ &&
					pos[i].y >= 0 && pos[i].y < row_size_)
			{
				uint64_t new_id = pos[i].y * col_size_ + pos[i].x;

				if(std::find(obstacle_ids_.begin(), obstacle_ids_.end(), new_id) != obstacle_ids_.end())
					continue;

				SquareCell cell(new_id);
				cell.idx.x = pos[i].x;
				cell.idx.y = pos[i].y;

				adjacent_cells.emplace_back(cell, cell_size_);
			}
		}

		return adjacent_cells;
	}
};

std::vector<std::tuple<SquareCell, double>> GetNeighbours(SquareCell cell, std::vector<uint64_t>& obstacle_ids)
{
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

	for(int i = 0; i < 4; i++)
	{
		if(pos[i].x >= 0 && pos[i].x < GRID_SIZE &&
				pos[i].y >= 0 && pos[i].y < GRID_SIZE)
		{
			uint64_t new_id = pos[i].y * GRID_SIZE + pos[i].x;

			if(std::find(obstacle_ids.begin(), obstacle_ids.end(), new_id) != obstacle_ids.end())
				continue;

			SquareCell cell(new_id);
			cell.idx.x = pos[i].x;
			cell.idx.y = pos[i].y;

			adjacent_cells.emplace_back(cell, 1.0);
		}
	}

	return adjacent_cells;
}

int main(int argc, char** argv )
{
	std::vector<uint64_t> obstacle_ids;
	obstacle_ids.push_back(5);
	obstacle_ids.push_back(6);
	obstacle_ids.push_back(7);
	obstacle_ids.push_back(17);
	obstacle_ids.push_back(18);
	obstacle_ids.push_back(19);
	for(int i = 0; i < 50; i++)
		obstacle_ids.push_back(GRID_SIZE * 10 + i);
	for(int i = 30; i < 80; i++)
		obstacle_ids.push_back(GRID_SIZE * 30 + i);
	for(int i = 20; i < 90; i++)
		obstacle_ids.push_back(GRID_SIZE * 60 + i);
//	for(int i = 50; i < GRID_SIZE; i++)
//		obstacle_ids.push_back(GRID_SIZE * 80 + i);

	clock_t		exec_time;
	exec_time = clock();
	SquareCell cell_s(0);
	cell_s.idx.x = 0;
	cell_s.idx.y = 0;

	SquareCell cell_g(9595);
	cell_g.idx.x = 95;
	cell_g.idx.y = 95;

	auto path = AStar::IncSearch(cell_s, cell_g, GetNeighbourBDSFunc_t<SquareCell>(GetSquareCellNeighbour(GRID_SIZE, GRID_SIZE, 1.0, obstacle_ids)));
	exec_time = clock() - exec_time;
	std::cout << "Incremental search finished in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

//	for(auto& e : path)
//		std::cout << "id: " << e->vertex_id_ << std::endl;

//	std::cout << "\n----------------------------------------------------------\n" << std::endl;

	exec_time = clock();
	Graph_t<SquareCell> graph2;
	std::vector<SquareCell> cells;

	for(int i = 0; i < GRID_SIZE; i++)
		for(int j = 0; j < GRID_SIZE; j++)
		{
			uint64_t new_id = j * GRID_SIZE + i;
			SquareCell new_cell(new_id);
			new_cell.idx.x = i;
			new_cell.idx.y = j;
			cells.push_back(new_cell);
		}

	for(auto& c : cells)
	{
		std::vector<std::tuple<SquareCell, double>> nbs = GetNeighbours(c, obstacle_ids);

		for(auto& nc : nbs)
		{
			graph2.AddEdge(c, std::get<0>(nc), std::get<1>(nc));
		}
	}

	auto path2 = AStar::Search(graph2, cells[0].GetUniqueID(), cells[9595].GetUniqueID());
	if(path2.empty())
		std::cout << "search failed" << std::endl;
	exec_time = clock() - exec_time;
	std::cout << "Normal graph search finished in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

//	for(auto& e : path2)
//		std::cout << "id: " << e->vertex_id_ << std::endl;

	return 0;
}



