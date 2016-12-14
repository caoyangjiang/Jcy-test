// Copyright 2016 Caoyang Jiang

#ifndef MODULES_TRANSFORM_INCLUDE_JCY_TRANSFORM_DCT2D_H_
#define MODULES_TRANSFORM_INCLUDE_JCY_TRANSFORM_DCT2D_H_

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <memory>

#include <vector>
#include "Jcy/Transform/Dct.h"

namespace Jcy
{
/**
 * @brief   Simple NxN DCT forward and inverse transformation.
 */
class Dct2d : public Dct
{
 public:
  Dct2d();
  ~Dct2d() override;

  bool Forward(const std::vector<std::vector<double>>& datain,
               std::vector<std::vector<double>>& dataout) override;
  bool Inverse(const std::vector<std::vector<double>>& datain,
               std::vector<std::vector<double>>& dataout) override;
};

}  // namespace Jcy
#endif  // MODULES_TRANSFORM_INCLUDE_JCY_TRANSFORM_DCT2D_H_
