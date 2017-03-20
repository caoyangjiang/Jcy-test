// Copyright 2015 Jason Juang

#ifndef UNIT_TEST_BITSTREAMTEST_INCLUDE_BITSTREAMTEST_H_
#define UNIT_TEST_BITSTREAMTEST_INCLUDE_BITSTREAMTEST_H_

#include "gtest/gtest.h"

class BitStreamTest : public ::testing::Test
{
 protected:
  BitStreamTest();
  ~BitStreamTest() override;

  void SetUp() override;
  void TearDown() override;
};

#endif  // UNIT_TEST_BITSTREAMTEST_INCLUDE_BITSTREAMTEST_H_
