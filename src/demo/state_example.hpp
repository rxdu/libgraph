/*
 * state_example.hpp
 *
 * Created on: Apr 15, 2016
 * Description:
 *
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef STATE_EXAMPLE_HPP
#define STATE_EXAMPLE_HPP

#include <cstdint>

namespace rdu {

struct StateExample {
  StateExample(uint64_t id) : id(id){};

  int64_t id;
};

}  // namespace rdu

#endif /* STATE_EXAMPLE_HPP */
