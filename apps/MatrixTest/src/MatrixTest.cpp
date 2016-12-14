// Copyright @ 2016 Caoyang Jiang
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <iomanip>
#include <vector>

#include "Jcy/Transform/SADct.h"

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

int main()
{
  matrix<int> adjmat(4, 4);
  matrix<int> eignvec(4, 1);
  matrix<int> r;

  // First row
  adjmat(0, 0) = 0;
  adjmat(0, 1) = 1;
  adjmat(0, 2) = 1;
  adjmat(0, 3) = 1;

  // Second row
  adjmat(1, 0) = 1;
  adjmat(1, 1) = 0;
  adjmat(1, 2) = 0;
  adjmat(1, 3) = 0;

  // Third row
  adjmat(2, 0) = 1;
  adjmat(2, 1) = 0;
  adjmat(2, 2) = 0;
  adjmat(2, 3) = 0;

  // Fourth row
  adjmat(3, 0) = 1;
  adjmat(3, 1) = 0;
  adjmat(3, 2) = 0;
  adjmat(3, 3) = 0;

  // Eignvector
  eignvec(0, 0) = 1;
  eignvec(1, 0) = 1;
  eignvec(2, 0) = 1;
  eignvec(3, 0) = 1;

  // r = boost::numeric::ublas::prod(adjmat, eignvec);

  // Eigen::Matrix<double, 6, 6> A;
  // A << 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
  // 1,
  //     0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0;
  // std::cout << "Matrix A: " << std::endl << A << std::endl;

  // Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(A);
  // std::cout << "Eigenvalues: " << std::endl
  //           << solver.eigenvalues() << std::endl;
  // std::cout << "Eigenvectors: " << std::endl
  //           << solver.eigenvectors() << std::endl;

  // std::cout << "Orthogonality check " << std::endl;

  // for (int ev = 0; ev < solver.eigenvectors().cols() - 1; ev++)
  // {
  //   for (int ev2 = ev + 1; ev2 < solver.eigenvectors().cols(); ev2++)
  //   {
  //     double dotprod =
  //         solver.eigenvectors().col(ev).dot(solver.eigenvectors().col(ev2));
  //     std::cout << ev << "th vecotr x " << ev2 << "th vector is: " << dotprod
  //               << std::endl;

  //     if (dotprod == 0)
  //     {
  //       std::cout << "good" << std::endl;
  //     }
  //     else
  //       std::cout << "Bad" << std::endl;
  //   }
  // }

  // int N = 3;
  // Eigen::MatrixXd fwcosmat(N, N);
  // Eigen::MatrixXd bwcosmat(N, N);
  // Eigen::MatrixXd in(N, 1);
  // Eigen::MatrixXd coef(N, 1);
  // Eigen::MatrixXd out(N, 1);

  // in(0, 0) = 1;
  // in(1, 0) = 2;
  // in(2, 0) = 3;

  // for (int row = 0; row < N; row++)
  // {
  //   for (int col = 0; col < N; col++)
  //   {
  //     double C0 = row == 0 ? sqrt(0.5) : 1;

  //     fwcosmat(row, col) = C0 * cos(row * (col + 0.5) * M_PI / N);
  //   }
  // }

  // bwcosmat = fwcosmat.transpose();

  // std::cout << "forward transformation matrix " << std::endl;
  // std::cout << fwcosmat << std::endl;

  // std::cout << "Transformed coefficients" << std::endl;
  // coef = 2 * fwcosmat * in / N;
  // std::cout << coef << std::endl;

  // std::cout << "forward transformation matrix " << std::endl;
  // std::cout << bwcosmat << std::endl;

  // std::cout << "Inverse Transformed coefficients" << std::endl;
  // out = bwcosmat * coef;
  // std::cout << out << std::endl;
  // Eigen::Matrix<float, 6, 1> b;
  // Eigen::Matrix<float, 6, 1> x;         << std::endl;
  // b << 0, 0, 0, 0, 0, 0;
  // std::cout << "Here is the matrix A:\n" << A << std::endl;
  // std::cout << "Here is the vector b:\n" << b << std::endl;
  // x = A.colPivHouseholderQr().solve(b);
  // std::cout << "The solution is:\n" << x << std::endl;
  //
  std::vector<std::vector<double>> mat;
  std::vector<std::vector<double>> out;
  std::vector<std::vector<bool>> contour;
  std::vector<std::vector<double>> in;

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

  Jcy::SADct dct;
  dct.SetContour(contour);
  dct.Forward(mat, out);
  std::cout << out << std::endl;
  out[3][0] = 0.0;
  out[2][0] = 0.0;
  out[2][1] = 0.0;
  out[0][3] = 0.0;
  out[1][2] = 0.0;
  out[0][2] = 0.0;
  out[1][1] = 0.0;
  std::cout << out << std::endl;
  dct.Inverse(out, out);

  std::cout << out << std::endl;
}
