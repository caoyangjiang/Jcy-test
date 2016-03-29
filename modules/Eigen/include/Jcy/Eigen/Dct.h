// Copyright 2016 Caoyang Jiang

#ifndef MODULES_EIGEN_INCLUDE_JCY_EIGEN_DCT_H_
#define MODULES_EIGEN_INCLUDE_JCY_EIGEN_DCT_H_

// #include <Eigen/Dense>
// #include <Eigen/Eigenvalues>
// #include <cmath>
// #include <memory>

#include <vector>

namespace Jcy
{
class Dct
{
 protected:
  Dct();
  virtual ~Dct();

 public:
  // Forbid copying
  Dct(Dct const&) = delete;
  Dct& operator=(Dct const&) = delete;

  virtual bool Forward(const std::vector<std::vector<double>>& datain,
                       std::vector<std::vector<double>>& dataout) = 0;
  virtual bool Inverse(const std::vector<std::vector<double>>& datain,
                       std::vector<std::vector<double>>& dataout) = 0;
};

}  // namespace Jcy
#endif  // MODULES_EIGEN_INCLUDE_JCY_EIGEN_DCT_H_
