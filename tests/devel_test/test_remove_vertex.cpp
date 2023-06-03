// standard libaray
#include <iostream>
#include <vector>
#include <ctime>

// user
#include "graph/graph.hpp"

using namespace xmotion;

struct TestState {
  TestState(uint64_t id) : id_(id){};

  int64_t id_;
};

int main(int argc, char* argv[]) {
  std::vector<TestState*> nodes;
  for (int i = 0; i < 9; i++) nodes.push_back(new TestState(i));

  Graph<TestState*> graph;

  graph.AddEdge(nodes[0], nodes[1], 1.2);
  graph.AddEdge(nodes[1], nodes[2], 1.5);
  graph.AddEdge(nodes[1], nodes[3], 1.5);
  graph.RemoveVertex(1);

  std::cout << "-- vertices: " << graph.GetTotalVertexNumber()
            << ", edges: " << graph.GetTotalEdgeNumber() << std::endl;

  graph.AddEdge(nodes[1], nodes[2], 1.5);
  graph.AddEdge(nodes[1], nodes[3], 1.5);

  std::cout << "-- vertices: " << graph.GetTotalVertexNumber()
            << ", edges: " << graph.GetTotalEdgeNumber() << std::endl;

  graph.RemoveVertex(2);

  std::cout << "-- vertices: " << graph.GetTotalVertexNumber()
            << ", edges: " << graph.GetTotalEdgeNumber() << std::endl;

  std::cout << "end" << std::endl;

  for (auto& nd : nodes) delete nd;
}
