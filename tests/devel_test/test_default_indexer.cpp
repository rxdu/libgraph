#include <iostream>

#include "graph/impl/default_indexer.hpp"

using namespace xmotion;

struct StateExample {
  StateExample(uint64_t id) : id(id){};

  int64_t id;
};

int main(int argc, char* argv[]) {
  {
    DefaultIndexer<StateExample> indexer;
    std::cout << "index: " << indexer(StateExample(1)) << std::endl;
  }

  {
    DefaultIndexer<StateExample*> indexer;
    StateExample* state_ptr = new StateExample(1);
    std::cout << "index: " << indexer(state_ptr) << std::endl;
    delete state_ptr;
  }

  {
    DefaultIndexer<std::shared_ptr<StateExample>> indexer;
    std::shared_ptr<StateExample> state_ptr = std::make_shared<StateExample>(1);
    std::cout << "index: " << indexer(state_ptr) << std::endl;
  }

  return 0;
}