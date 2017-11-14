/* 
 * bds_base.h
 * 
 * Created on: Apr 14, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef BDS_BASE_H
#define BDS_BASE_H

#include <cstdint>

namespace librav {

/****************************************************************************/
/*				   Bundled Data Structure (BDS) Base						*/
/****************************************************************************/
/// The base class of bundled data structure
template<typename BundledDataStructType>
class BDSBase {
protected:
	/// Only derived classes can call the constructor and destructor.
	// Deleting default constructor enforces the derived class to
	//	initialize the id somehow, just to make sure "data_id_" is
	//	used as the ID, not with any other names
	BDSBase() = delete;
	BDSBase(uint64_t struct_id):data_id_(struct_id){};
	~BDSBase(){};

public:
	uint64_t data_id_;
	uint64_t GetID() const {return data_id_;}

public:
	// You have to override this function if you want to use heuristics in A*
	double GetHeuristic(const BundledDataStructType& other_struct) const
	{
		return static_cast<BundledDataStructType*>(this)->GetHeuristic(other_struct);
	};
};

}

#endif /* BDS_BASE_H */
