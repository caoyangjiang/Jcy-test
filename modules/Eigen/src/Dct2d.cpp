// Copyright @ 2016 Caoyang Jiang

#include "Jcy/Eigen/Dct2d.h"
#include <cmath>
#include <iostream>
#include <vector>

namespace Jcy
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

  dim    = indata.size();
  kernel = Eigen::MatrixXd(dim, dim);
  in     = Eigen::MatrixXd(dim, dim);
  out    = Eigen::MatrixXd(dim, dim);

  // Read 2D vector into matrix
  for (int row = 0; row < dim; row++)
  {
    for (int col = 0; col < dim; col++)
    {
      in(row, col) = indata[row][col];
    }
  }

  // Build transform kernel
  // (c1 * cos((2 * m + 1) * k * PI) / (2 * N))
  for (int row = 0; row < dim; row++)
  {
    for (int col = 0; col < dim; col++)
    {
      c1 = (row == 0) ? 0.5 : sqrt(2) / 2.0;
      kernel(row, col) = c1 * cos((2 * col + 1) * row * M_PI / (2 * dim));
    }
  }

  // Transform
  out = kernel * in;               // row-wise transformation
  out = out * kernel.transpose();  // column-wise transformation

  outdata = indata;  // assign dimension

  for (int row = 0; row < dim; row++)
  {
    for (int col = 0; col < dim; col++)
    {
      outdata[row][col] = out(row, col);
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

  dim    = indata.size();
  kernel = Eigen::MatrixXd(dim, dim);
  in     = Eigen::MatrixXd(dim, dim);
  out    = Eigen::MatrixXd(dim, dim);

  // Read 2D vector into matrix
  for (int row = 0; row < dim; row++)
  {
    for (int col = 0; col < dim; col++)
    {
      in(row, col) = indata[row][col];
    }
  }

  // Build forword transform kernel
  // (c1 * cos((2 * m + 1) * k * PI) / (2 * N))
  for (int row = 0; row < dim; row++)
  {
    for (int col = 0; col < dim; col++)
    {
      c1 = (row == 0) ? 0.5 : sqrt(2) / 2.0;
      kernel(row, col) = c1 * cos((2 * col + 1) * row * M_PI / (2 * dim));
    }
  }

  // Build inverse transform kernel

  out = kernel.transpose() * in;
  out = out * kernel;

  outdata = indata;  // assign dimension

  for (int row = 0; row < dim; row++)
  {
    for (int col = 0; col < dim; col++)
    {
      outdata[row][col] = out(row, col);
    }
  }
  return true;
}

}  // namespace Jcy
