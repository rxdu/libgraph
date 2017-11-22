/* 
 * state_example.h
 * 
 * Created on: Apr 15, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef STATE_EXAMPLE_H
#define STATE_EXAMPLE_H

#include <cstdint>

namespace librav {

struct StateExample
{
	StateExample(uint64_t id):any_unique_id_(id){};

	uint64_t any_unique_id_;

	uint64_t GetUniqueID() const
	{
		return any_unique_id_;
	}
};

}

#endif /* STATE_EXAMPLE_H */
