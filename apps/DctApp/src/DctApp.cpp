// Copyright @ 2016 Caoyang Jiang
#include <cmath>
#include <iomanip>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/Eigenvalues"
#include "Jcy/Transform/Dct2d.h"
#include "Jcy/Transform/SADct.h"
#include "boost/numeric/ublas/io.hpp"
#include "boost/numeric/ublas/matrix.hpp"

using boost::numeric::ublas::matrix;

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

template <class T>
void RoundVec(std::vector<std::vector<T>>& vec)
{
  for (size_t i = 0; i < vec.size(); i++)
  {
    for (size_t j = 0; j < vec[i].size(); j++)
    {
      vec[i][j] = std::nearbyint(vec[i][j]);
    }
  }
}

template <class T>
double MSE(const std::vector<std::vector<T>>& vec1,
           const std::vector<std::vector<T>>& vec2)
{
  int count       = 0;
  double sumerror = 0;

  for (size_t i = 0; i < vec1.size(); i++)
  {
    for (size_t j = 0; j < vec1[i].size(); j++)
    {
      sumerror += std::pow((vec1[i][j] - vec2[i][j]), 2);
      count++;
    }
  }

  return sumerror / static_cast<double>(count);
}

int main()
{
  std::vector<std::vector<double>> mat;
  std::vector<std::vector<double>> out;
  std::vector<std::vector<bool>> contour;
  std::vector<std::vector<double>> in;
  jcy::SADct dct;
  jcy::Dct2d dct2d;

  for (size_t i = 0; i < 4; i++)
  {
    std::vector<double> tmp;
    std::vector<bool> ctmp;

    for (size_t j = 0; j < 4; j++)
    {
      tmp.push_back(static_cast<double>(j) + static_cast<double>(i));
      ctmp.push_back(true);
    }
    contour.push_back(ctmp);
    mat.push_back(tmp);
  }

  contour[0][0] = false;
  contour[0][1] = false;
  contour[0][2] = false;
  contour[2][0] = false;
  contour[3][0] = false;
  contour[3][1] = false;

  dct2d.Forward(mat, out);
  std::cout << mat << std::endl;
  std::cout << out << std::endl;
  mat[0][0] = 0;
  mat[0][1] = 0;
  mat[0][2] = 0;
  mat[2][0] = 0;
  mat[3][0] = 0;
  mat[3][1] = 0;

  for (size_t i = 0; i < 4; i++)
  {
    std::vector<double> tmp;
    for (size_t j = 0; j < 4; j++)
    {
      if (contour[i][j])
      {
        tmp.push_back(mat[i][j]);
      }
    }

    if (!tmp.empty()) in.push_back(tmp);
  }

  std::cout << in << std::endl;

  dct.SetContour(contour);
  dct.Forward(mat, out);
  std::cout << out << std::endl;
  // out[3][0] = 0.0;
  // out[2][0] = 0.0;
  // out[2][1] = 0.0;
  // out[0][3] = 0.0;
  // out[1][2] = 0.0;
  // out[0][2] = 0.0;
  // out[1][1] = 0.0;
  std::cout << out << std::endl;
  dct.Inverse(out, out);

  // RoundVec(out);
  std::cout << out << std::endl;
  std::cout << "MSE: " << MSE(in, out) << std::endl;
}
