// Copyright 2019 Project March.

#include "gtest/gtest.h"
#include <gmock/gmock.h>


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  auto res = RUN_ALL_TESTS();
  return res;
}
