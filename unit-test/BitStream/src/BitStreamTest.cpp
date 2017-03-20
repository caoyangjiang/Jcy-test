// Copyright 2015 Jason Juang

#include "BitStreamTest.h"

#include <string>

#include "Jcy/MiscTools/BitStream.h"

BitStreamTest::BitStreamTest()
{
}

BitStreamTest::~BitStreamTest()
{
}

void BitStreamTest::SetUp()
{
}

void BitStreamTest::TearDown()
{
}

TEST_F(BitStreamTest, SimpleTest)
{
  jcy::BitStream bs0(jcy::BitStream::MODE::WR, jcy::BitStream::ENDIAN::LITTLE);
  jcy::BitStream bs1(jcy::BitStream::MODE::RD, jcy::BitStream::ENDIAN::LITTLE);

  const int zero = 0;
  const int one  = 1;

  for (size_t i = 0; i < 1000; i++)
  {
    bs0.Write(reinterpret_cast<const uint8_t*>(&zero), 1);
    bs0.Write(reinterpret_cast<const uint8_t*>(&one), 1);
  }

  EXPECT_EQ(bs0.GetWrittenSize(), 2000u);
}
