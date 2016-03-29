// Copyright 2015 Caoyang Jiang

#ifndef UNIT_TEST_EIGENTEST_INC_EIGENTEST_H_
#define UNIT_TEST_EIGENTEST_INC_EIGENTEST_H_

#include "gtest/gtest.h"

class EigenTest : public ::testing::Test
{
 protected:
  EigenTest();
  ~EigenTest() override;

  void SetUp() override;
  void TearDown() override;
};

#endif  // UNIT_TEST_EIGENTEST_INC_EIGENTEST_H_
