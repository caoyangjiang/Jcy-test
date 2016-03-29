// Copyright 2016 Caoyang Jiang

#include "EigenTest.h"
#include <vector>
#include "Jcy/Eigen/Dct2d.h"

EigenTest::EigenTest()
{
}

EigenTest::~EigenTest()
{
}

void EigenTest::SetUp()
{
}

void EigenTest::TearDown()
{
}

TEST_F(EigenTest, Dct2d)
{
  Jcy::Dct2d dct;

  std::vector<std::vector<double>> in;
  std::vector<std::vector<double>> coef;
  std::vector<std::vector<double>> recon;

  for (size_t i = 0; i < 4; i++)
  {
    std::vector<double> tmp;
    for (size_t j = 0; j < 4; j++)
    {
      tmp.push_back(1.0);
    }
    in.push_back(tmp);
  }

  dct.Forward(in, coef);
  dct.Inverse(coef, recon);

  for (size_t i = 0; i < 4; i++)
  {
    for (size_t j = 0; j < 4; j++)
    {
      // EXPECT_LE(std::abs(in[i][j] - recon[i][j]), 1e-10);
      EXPECT_EQ(std::nearbyint(in[i][j]), std::nearbyint(recon[i][j]));
    }
  }
}
