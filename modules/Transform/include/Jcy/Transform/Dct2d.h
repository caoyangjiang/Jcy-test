// Copyright 2016 Caoyang Jiang

#ifndef MODULES_TRANSFORM_INCLUDE_JCY_TRANSFORM_DCT2D_H_
#define MODULES_TRANSFORM_INCLUDE_JCY_TRANSFORM_DCT2D_H_

JCY_WINDOWS_DISABLE_ALL_WARNING
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <memory>
#include <vector>
JCY_WINDOWS_DISABLE_ALL_WARNING
#include "Jcy/Transform/Dct.h"

namespace jcy
{
/**
 * @brief   Simple NxN DCT forward and inverse transformation.
 */
class Dct2d : public Dct
{
 public:
  Dct2d();
  ~Dct2d() override;

  JCY_TRANSFORM_DLL bool Forward(
      const std::vector<std::vector<double>>& datain,
      std::vector<std::vector<double>>& dataout) override;
  JCY_TRANSFORM_DLL bool Inverse(
      const std::vector<std::vector<double>>& datain,
      std::vector<std::vector<double>>& dataout) override;

 private:
  const double PI = 3.1415926;
};

}  // namespace jcy
#endif  // MODULES_TRANSFORM_INCLUDE_JCY_TRANSFORM_DCT2D_H_
