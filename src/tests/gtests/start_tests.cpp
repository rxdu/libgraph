/* 
 * start_tests.cpp
 * 
 * Created on: Nov 30, 2017 16:22
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "gtest/gtest.h"

int main(int ac, char *av[])
{
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}
