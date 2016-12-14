// Copyright @ 2016 Caoyang Jiang

#include "Jcy/Transform/SADct.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

namespace Jcy
{
template <class T>
std::ostream& operator<<(std::ostream& os,
                         const std::vector<std::vector<T>>& mat)
{
  os << std::setprecision(6) << std::fixed;
  for (size_t i = 0; i < mat.size(); i++)
  {
    for (size_t j = 0; j < mat[i].size(); j++)
    {
      os << mat[i][j] << " ";
    }
    os << std::endl;
  }

  return os;
}

SADct::SADct()
{
  /* Build transform kernels */

  for (int k = 0; k < MAXDIM; k++)  //  DCT-k basis
  {
    int n = k + 1;

    kernels_.push_back(Eigen::MatrixXd(n, n));

    for (int row = 0; row < n; row++)
    {
      for (int col = 0; col < n; col++)
      {
        // Eq. (1)
        double C0 = row == 0 ? sqrt(0.5) : 1;

        kernels_[k](row, col) = C0 * cos(row * (col + 0.5) * M_PI / n);
      }
    }
  }
}

SADct::~SADct()
{
}

bool SADct::SetContour(const std::vector<std::vector<bool>>& contour)
{
  contour_ = contour;

  return true;
}

bool SADct::Forward(const std::vector<std::vector<double>>& indata,
                    std::vector<std::vector<double>>& outdata)
{
  int dim[2];
  Eigen::MatrixXd mat;
  std::vector<std::vector<bool>> cont;

  cont   = contour_;
  dim[0] = contour_.size();
  dim[1] = contour_[0].size();
  mat    = Eigen::MatrixXd(dim[0], dim[1]);

  // Read 2D vector into matrix
  for (int row = 0; row < static_cast<int>(indata.size()); row++)
  {
    for (int col = 0; col < static_cast<int>(indata[row].size()); col++)
    {
      mat(row, col) = indata[row][col];
    }
  }

  // Push input matrix  up
  for (int col = 0; col < dim[1]; col++)
  {
    int occupiedrow = 0;
    for (int row = 0; row < dim[0]; row++)
    {
      if (cont[row][col])
      {
        bool tmp;
        double mtmp;

        mtmp = mat(row, col);
        mat(row, col)         = 0.0;
        mat(occupiedrow, col) = mtmp;

        tmp                    = cont[row][col];
        cont[row][col]         = false;
        cont[occupiedrow][col] = tmp;
        occupiedrow++;
      }
    }
  }

  // Column-wise transformation
  for (int col = 0; col < dim[1]; col++)
  {
    std::vector<double> columnval;
    // Scan column
    for (int row = 0; row < dim[0]; row++)
    {
      if (cont[row][col])
      {
        columnval.push_back(mat(row, col));
      }
    }

    if (!columnval.empty())
    {
      // Build vector
      int n = static_cast<int>(columnval.size());
      Eigen::MatrixXd columnmat(n, 1);

      for (int row = 0; row < n; row++)
      {
        columnmat(row, 0) = columnval[row];
      }

      // Transform
      columnmat = 2 * (kernels_[n - 1] * columnmat) / n;  // Eq. (2)

      // Write back
      for (int row = 0; row < dim[0]; row++)
      {
        if (cont[row][col])
        {
          mat(row, col) = columnmat(row, 0);
        }
      }
    }
  }

  // Push matrix left
  for (int row = 0; row < dim[0]; row++)
  {
    int occupiedcol = 0;
    for (int col = 0; col < dim[1]; col++)
    {
      if (cont[row][col])
      {
        bool tmp;
        double mtmp;

        mtmp = mat(row, col);
        mat(row, col)         = 0.0;
        mat(row, occupiedcol) = mtmp;

        tmp                    = cont[row][col];
        cont[row][col]         = false;
        cont[row][occupiedcol] = tmp;
        occupiedcol++;
      }
    }
  }

  // row-wise transformation
  for (int row = 0; row < dim[0]; row++)
  {
    std::vector<double> rowval;
    // Scan row
    for (int col = 0; col < dim[0]; col++)
    {
      if (cont[row][col])
      {
        rowval.push_back(mat(row, col));
      }
    }

    if (!rowval.empty())
    {
      // Build vector
      int n = static_cast<int>(rowval.size());
      Eigen::MatrixXd rowmat(n, 1);

      for (int col = 0; col < n; col++)
      {
        rowmat(col, 0) = rowval[col];
      }

      // Transform
      rowmat = 2 * (kernels_[n - 1] * rowmat) / n;  // Eq. (2)

      // Write back
      for (int col = 0; col < dim[1]; col++)
      {
        if (cont[row][col])
        {
          mat(row, col) = rowmat(col, 0);
        }
      }
    }
  }

  // Write only meaningful coefficients (according to contour map)
  outdata.clear();

  for (int row = 0; row < dim[0]; row++)
  {
    std::vector<double> tmp;
    for (int col = 0; col < dim[1]; col++)
    {
      if (cont[row][col])
      {
        tmp.push_back(mat(row, col));
      }
    }

    outdata.push_back(tmp);
  }

  return true;
}

bool SADct::Inverse(const std::vector<std::vector<double>>& indata,
                    std::vector<std::vector<double>>& outdata)
{
  std::vector<std::vector<bool>> contup;
  std::vector<std::vector<bool>> contupleft;
  std::vector<std::vector<bool>> cont;
  int dim[2];

  dim[0] = static_cast<int>(contour_.size());
  dim[1] = static_cast<int>(contour_[0].size());
  // Construct two intermediate contours

  cont = contour_;

  // Push input matrix  up
  for (int col = 0; col < dim[1]; col++)
  {
    int occupiedrow = 0;
    for (int row = 0; row < dim[0]; row++)
    {
      if (cont[row][col])
      {
        bool tmp;
        tmp                    = cont[row][col];
        cont[row][col]         = false;
        cont[occupiedrow][col] = tmp;
        occupiedrow++;
      }
    }
  }

  contup = cont;  // keep the up-pushed contour

  // Push matrix left
  for (int row = 0; row < dim[0]; row++)
  {
    int occupiedcol = 0;
    for (int col = 0; col < dim[1]; col++)
    {
      if (cont[row][col])
      {
        bool tmp;
        tmp                    = cont[row][col];
        cont[row][col]         = false;
        cont[row][occupiedcol] = tmp;
        occupiedcol++;
      }
    }
  }

  contupleft = cont;  // Keep the final contour

  /* Reconstruct matrix with coefficients and contour */
  Eigen::MatrixXd mat(dim[0], dim[1]);
  cont = contour_;
  // Load coefficients into matrix
  for (int row = 0; row < static_cast<int>(indata.size()); row++)
  {
    for (int col = 0; col < static_cast<int>(indata[row].size()); col++)
    {
      mat(row, col) = indata[row][col];
    }
  }

  // Row-wise inverse transformation
  for (int row = 0; row < dim[1]; row++)
  {
    std::vector<double> rowval;
    // Scan row
    for (int col = 0; col < dim[0]; col++)
    {
      if (contupleft[row][col])
      {
        rowval.push_back(mat(row, col));
      }
    }

    if (!rowval.empty())
    {
      // Build vector
      int n = static_cast<int>(rowval.size());
      Eigen::MatrixXd rowmat(n, 1);

      for (int col = 0; col < n; col++)
      {
        rowmat(col, 0) = rowval[col];
      }

      // Inverse Transform
      rowmat = kernels_[n - 1].transpose() * rowmat;  // Eq. (2)

      // Write back
      for (int col = 0; col < dim[1]; col++)
      {
        if (contupleft[row][col])
        {
          mat(row, col) = rowmat(col, 0);
        }
      }
    }
  }

  // Reconstruct pushed up matrix
  for (int row = 0; row < dim[1]; row++)
  {
    int dstcol = dim[0] - 1;
    while ((dstcol != 0) && !contup[row][dstcol]) dstcol--;

    for (int col = dim[0] - 1; col >= 0; col--)
    {
      if (contupleft[row][col])
      {
        if (dstcol < col)
        {
          std::cout << "Overrun happened in row-wise reconstruct!" << std::endl;
          return false;
        }

        mat(row, dstcol) = mat(row, col);
        dstcol--;
      }
    }
  }

  // Column-wise inverse transform
  for (int col = 0; col < dim[1]; col++)
  {
    std::vector<double> columnval;
    // Scan column
    for (int row = 0; row < dim[0]; row++)
    {
      if (contup[row][col])
      {
        columnval.push_back(mat(row, col));
      }
    }

    if (!columnval.empty())
    {
      // Build vector
      int n = static_cast<int>(columnval.size());
      Eigen::MatrixXd columnmat(n, 1);

      for (int row = 0; row < n; row++)
      {
        columnmat(row, 0) = columnval[row];
      }

      // Transform
      columnmat = kernels_[n - 1].transpose() * columnmat;  // Eq. (2)

      // Write back
      for (int row = 0; row < dim[0]; row++)
      {
        if (contup[row][col])
        {
          mat(row, col) = columnmat(row, 0);
        }
      }
    }
  }

  // Reconstruct original matrix by down push
  for (int col = 0; col < dim[0]; col++)
  {
    int dstrow = dim[1] - 1;
    while ((dstrow != 0) && !contour_[dstrow][col]) dstrow--;

    for (int row = dim[1] - 1; row >= 0; row--)
    {
      if (contup[row][col])
      {
        if (dstrow < row)
        {
          std::cout << "Overrun happened in col-wise reconstruct!" << std::endl;
          return false;
        }

        mat(dstrow, col) = mat(row, col);
        dstrow--;
      }
    }
  }

  // Write only meaningful samples (according to contour map)
  outdata.clear();

  for (int row = 0; row < dim[0]; row++)
  {
    std::vector<double> tmp;
    for (int col = 0; col < dim[1]; col++)
    {
      if (cont[row][col])
      {
        tmp.push_back(mat(row, col));
      }
    }

    outdata.push_back(tmp);
  }

  return true;
}

}  // namespace Jcy
