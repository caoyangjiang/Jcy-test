// Copyright @ 2016 Caoyang Jiang

#include "Jcy/Transform/Dct2d.h"

JCY_WINDOWS_DISABLE_ALL_WARNING
#include <cmath>
#include <iostream>
#include <vector>
JCY_WINDOWS_DISABLE_ALL_WARNING

namespace jcy
{
Dct2d::Dct2d()
{
}

Dct2d::~Dct2d()
{
}

bool Dct2d::Forward(const std::vector<std::vector<double>>& indata,
                    std::vector<std::vector<double>>& outdata)
{
  int dim;
  double c1;
  Eigen::MatrixXd kernel;
  Eigen::MatrixXd in;
  Eigen::MatrixXd out;

  // Check input matrix
  if (indata.size() == 0)
  {
    std::cout << "Matrix empty" << std::endl;
    return false;
  }

  for (size_t isize = 0; isize < indata.size() - 1; isize++)
  {
    if (indata[isize].size() != indata[isize + 1].size())
    {
      std::cout << "Matrix is not rectangular" << std::endl;
      return false;
    }
  }

  if (indata.size() != indata[0].size())
  {
    std::cout << "Matrix is not square" << std::endl;
    return false;
  }

  dim    = static_cast<int>(indata.size());
  kernel = Eigen::MatrixXd(dim, dim);
  in     = Eigen::MatrixXd(dim, dim);
  out    = Eigen::MatrixXd(dim, dim);

  // Read 2D vector into matrix
  for (size_t row = 0; row < static_cast<size_t>(dim); row++)
  {
    for (size_t col = 0; col < static_cast<size_t>(dim); col++)
    {
      in(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) =
          indata[row][col];
    }
  }

  // Build transform kernel
  // (c1 * cos((2 * m + 1) * k * PI) / (2 * N))
  for (int row = 0; row < dim; row++)
  {
    for (int col = 0; col < dim; col++)
    {
      c1 = (row == 0) ? 0.5 : sqrt(2) / 2.0;
      kernel(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) =
          c1 * cos((2 * col + 1) * row * PI / (2 * dim));
    }
  }

  // Transform
  out = kernel * in;               // row-wise transformation
  out = out * kernel.transpose();  // column-wise transformation

  outdata = indata;  // assign dimension

  for (size_t row = 0; row < static_cast<size_t>(dim); row++)
  {
    for (size_t col = 0; col < static_cast<size_t>(dim); col++)
    {
      outdata[row][col] =
          out(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col));
    }
  }

  // kernel = kernel.transpose();
  return true;
}

bool Dct2d::Inverse(const std::vector<std::vector<double>>& indata,
                    std::vector<std::vector<double>>& outdata)
{
  int dim;
  double c1;
  Eigen::MatrixXd kernel;
  Eigen::MatrixXd in;
  Eigen::MatrixXd out;

  // Check input matrix
  if (indata.size() == 0)
  {
    std::cout << "Matrix empty" << std::endl;
    return false;
  }

  for (size_t isize = 0; isize < indata.size() - 1; isize++)
  {
    if (indata[isize].size() != indata[isize + 1].size())
    {
      std::cout << "Matrix is not rectangular" << std::endl;
      return false;
    }
  }

  if (indata.size() != indata[0].size())
  {
    std::cout << "Matrix is not square" << std::endl;
    return false;
  }

  dim    = static_cast<int>(indata.size());
  kernel = Eigen::MatrixXd(dim, dim);
  in     = Eigen::MatrixXd(dim, dim);
  out    = Eigen::MatrixXd(dim, dim);

  // Read 2D vector into matrix
  for (size_t row = 0; row < static_cast<size_t>(dim); row++)
  {
    for (size_t col = 0; col < static_cast<size_t>(dim); col++)
    {
      in(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) =
          indata[row][col];
    }
  }

  // Build forword transform kernel
  // (c1 * cos((2 * m + 1) * k * PI) / (2 * N))
  for (int row = 0; row < dim; row++)
  {
    for (int col = 0; col < dim; col++)
    {
      c1 = (row == 0) ? 0.5 : sqrt(2) / 2.0;
      kernel(row, col) = c1 * cos((2 * col + 1) * row * PI / (2 * dim));
    }
  }

  // Build inverse transform kernel

  out = kernel.transpose() * in;
  out = out * kernel;

  outdata = indata;  // assign dimension

  for (size_t row = 0; row < static_cast<size_t>(dim); row++)
  {
    for (size_t col = 0; col < static_cast<size_t>(dim); col++)
    {
      outdata[row][col] =
          out(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col));
    }
  }
  return true;
}

}  // namespace jcy
