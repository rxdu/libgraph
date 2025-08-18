/*
 * example_state.hpp
 *
 * Created on: Apr 15, 2016
 * Description:
 *
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef EXAMPLE_STATE_HPP
#define EXAMPLE_STATE_HPP

#include <cstdint>

namespace xmotion {

struct ExampleState {
  ExampleState(uint64_t id) : id(id){};

  int64_t id;
  
  // For compatibility with DefaultIndexer
  int64_t GetId() const { return id; }
};

}  // namespace xmotion

#endif /* EXAMPLE_STATE_HPP */
