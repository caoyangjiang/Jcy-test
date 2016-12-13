// Copyright @ 2016 Caoyang Jiang

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

template <class T>
class SimpleMatrix
{
 public:
  SimpleMatrix()
  {
  }

  SimpleMatrix(size_t rows, size_t cols)
  {
    mat_ = std::vector<std::vector<T>>(rows);

    for (size_t irow = 0; irow < mat_.size(); irow++)
    {
      mat_[irow] = std::vector<T>(cols);
    }
  }

  SimpleMatrix(const SimpleMatrix<T>& smat)
  {
    mat_ = smat.Get();
  }

  ~SimpleMatrix()
  {
  }

  SimpleMatrix<T>& operator=(const SimpleMatrix& lhs)
  {
    mat_ = lhs.Get();
    return *this;
  }

  void Create(size_t rows, size_t cols)
  {
    mat_ = std::vector<std::vector<T>>(rows);

    for (size_t irow = 0; irow < mat_.size(); irow++)
    {
      mat_[irow] = std::vector<T>(cols);
    }
  }

  T& operator()(size_t row, size_t column) const
  {
    return const_cast<T&>(mat_[row][column]);
  }

  std::vector<T> row(size_t row) const
  {
    return mat_[row];
  }

  std::vector<T> col(size_t col) const
  {
    std::vector<T> columnvalues;

    for (size_t irow = 0; irow < mat_.size(); irow++)
    {
      columnvalues.push_back(mat_[irow][col]);
    }

    return columnvalues;
  }

  void Transpose()
  {
    std::vector<std::vector<T>> mat(GetNumOfCols());

    for (size_t irow = 0; irow < mat.size(); irow++)
    {
      mat[irow] = std::vector<T>(GetNumOfRows());
    }

    for (size_t irow = 0; irow < mat_.size(); irow++)
    {
      for (size_t icol = 0; icol < mat_[irow].size(); icol++)
      {
        mat[icol][irow] = mat_[irow][icol];
      }
    }

    mat_ = mat;
  }

  size_t GetNumOfRows() const
  {
    return mat_.size();
  }

  size_t GetNumOfCols() const
  {
    return mat_[0].size();
  }

  const std::vector<std::vector<T>>& Get() const
  {
    return mat_;
  }

 private:
  std::vector<std::vector<T>> mat_;
};

template <class T>
SimpleMatrix<T> operator*(const SimpleMatrix<T>& mat1, T factor)
{
  SimpleMatrix<T> result(mat1.GetNumOfRows(), mat1.GetNumOfCols());

  for (size_t irow = 0; irow < mat1.GetNumOfRows(); irow++)
  {
    for (size_t icol = 0; icol < mat1.GetNumOfCols(); icol++)
    {
      result(irow, icol) = factor * mat1(irow, icol);
    }
  }

  return result;
}

template <class T>
SimpleMatrix<T> operator*(T factor, const SimpleMatrix<T>& mat1)
{
  SimpleMatrix<T> result(mat1.GetNumOfRows(), mat1.GetNumOfCols());

  for (size_t irow = 0; irow < mat1.GetNumOfRows(); irow++)
  {
    for (size_t icol = 0; icol < mat1.GetNumOfCols(); icol++)
    {
      result(irow, icol) = factor * mat1(irow, icol);
    }
  }

  return result;
}

template <class T>
SimpleMatrix<T> operator/(const SimpleMatrix<T>& mat1, T factor)
{
  SimpleMatrix<T> result(mat1.GetNumOfRows(), mat1.GetNumOfCols());

  for (size_t irow = 0; irow < mat1.GetNumOfRows(); irow++)
  {
    for (size_t icol = 0; icol < mat1.GetNumOfCols(); icol++)
    {
      result(irow, icol) = mat1(irow, icol) / factor;
    }
  }

  return result;
}

template <class T>
SimpleMatrix<T> operator/(T factor, const SimpleMatrix<T>& mat1)
{
  SimpleMatrix<T> result(mat1.GetNumOfRows(), mat1.GetNumOfCols());

  for (size_t irow = 0; irow < mat1.GetNumOfRows(); irow++)
  {
    for (size_t icol = 0; icol < mat1.GetNumOfCols(); icol++)
    {
      result(irow, icol) = factor / mat1(irow, icol);
    }
  }

  return result;
}

template <class T>
SimpleMatrix<T> operator*(const SimpleMatrix<T>& mat1,
                          const SimpleMatrix<T>& mat2)
{
  SimpleMatrix<T> result;

  if (mat1.GetNumOfCols() != mat2.GetNumOfRows())
  {
    std::cout << mat1.GetNumOfCols() << " " << mat2.GetNumOfRows() << std::endl;
    std::cout << "Dimension miss match" << std::endl;
    return result;
  }
  else
  {
    result.Create(mat1.GetNumOfRows(), mat2.GetNumOfCols());
    size_t mul = mat1.GetNumOfCols();
    T sum;

    // Matrix multiplication
    for (size_t irow = 0; irow < mat1.GetNumOfRows(); irow++)
    {
      for (size_t icol = 0; icol < mat2.GetNumOfCols(); icol++)
      {
        sum = 0;
        for (size_t k = 0; k < mul; k++) sum += mat1(irow, k) * mat2(k, icol);
        result(irow, icol) = sum;
      }
    }
  }

  return result;
}

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

// bool ForwardTransform(const std::vector<int>& src, std::vector<int>& dst)
// {
//   return true;
// }

// bool ForwardTransformQuant(const std::vector<int>& src,
//                            const double quantizer,
//                            std::vector<int>& dst)
// {
//   return true;
// }

void BuildTransformKernel(SimpleMatrix<double>& kernel, int dim)
{
  kernel.Create(dim, dim);

  /* Build transform kernels */
  for (int row = 0; row < dim; row++)
  {
    for (int col = 0; col < dim; col++)
    {
      // Eq. (1)
      double C0 = row == 0 ? sqrt(0.5) : 1;

      kernel(row, col) = C0 * cos(row * (col + 0.5) * M_PI / dim);
    }
  }
}

int main()
{
  int n          = 200;
  double array[] = {
      389, 359, 325, 399, 389, 376, 409, 389, 399, 418, 424, 409, 412, 424, 418,
      389, 409, 412, 389, 390, 387, 405, 390, 389, 387, 384, 359, 384, 380, 359,
      380, 376, 359, 376, 373, 359, 373, 368, 359, 368, 363, 359, 363, 355, 359,
      359, 355, 341, 298, 359, 341, 298, 318, 302, 292, 298, 302, 298, 292, 279,
      264, 246, 298, 298, 246, 359, 246, 240, 359, 240, 210, 359, 210, 208, 359,
      208, 226, 359, 226, 234, 359, 234, 247, 359, 247, 325, 359, 252, 325, 247,
      271, 314, 325, 288, 313, 314, 282, 288, 314, 271, 275, 314, 252, 256, 325,
      325, 256, 271, 325, 314, 319, 325, 325, 319, 339, 325, 325, 314, 315, 319,
      314, 313, 315, 325, 339, 365, 325, 365, 371, 325, 371, 376, 325, 376, 389,
      359, 389, 387, 413, 449, 441, 443, 446, 449, 454, 454, 449, 454, 449, 446,
      454, 455, 449, 455, 441, 449, 441, 436, 436, 441, 424, 453, 424, 419, 453,
      411, 419, 417, 419, 424, 417, 466, 453, 419, 466, 419, 472, 421, 432, 407,
      416, 421, 376, 421, 421, 426, 421, 416, 421, 421, 416, 417, 422, 421, 417,
      422, 417, 426, 421, 426};
  SimpleMatrix<double> vec1(1, n);
  SimpleMatrix<double> mat2;
  SimpleMatrix<double> tran;
  SimpleMatrix<double> recon;
  SimpleMatrix<int> coef(n, 1);
  for (int i = 0; i < n; i++)
  {
    vec1(0, i) = array[i];
    // vec1(0, i) = 2.0;
  }

  std::cout << vec1.Get() << std::endl;
  BuildTransformKernel(mat2, n);
  vec1.Transpose();

  tran = 2.0 * mat2 * vec1 / static_cast<double>(n);

  for (int i = 0; i < n; i++)
  {
    coef(i, 0) = static_cast<int>(tran(i, 0) + (tran(i, 0) > 0 ? 0.5 : -0.5));
  }

  mat2.Transpose();
  recon = mat2 * tran;

  coef.Transpose();
  tran.Transpose();
  recon.Transpose();

  std::cout << tran.Get() << std::endl;
  std::cout << coef.Get() << std::endl;
  std::cout << recon.Get() << std::endl;

  return 0;
}
