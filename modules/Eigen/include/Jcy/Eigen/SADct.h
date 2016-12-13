// Copyright 2016 Caoyang Jiang

#ifndef MODULES_EIGEN_INCLUDE_JCY_EIGEN_SADCT_H_
#define MODULES_EIGEN_INCLUDE_JCY_EIGEN_SADCT_H_

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <cmath>
#include <memory>
#include <vector>

#include "Jcy/Eigen/Dct.h"

namespace Jcy
{
/**
 * @brief  S.Thomas & M.Bela, "Shape-Adaptive DCT for Generic Coding of Video"
 */
class SADct : public Dct
{
 public:
  SADct();
  ~SADct() override;

  bool Forward(const std::vector<std::vector<double>>& datain,
               std::vector<std::vector<double>>& dataout) override;
  bool Inverse(const std::vector<std::vector<double>>& datain,
               std::vector<std::vector<double>>& dataout) override;

  bool SetContour(const std::vector<std::vector<bool>>& contour);

 private:
  std::vector<std::vector<bool>> contour_;
  std::vector<Eigen::MatrixXd> kernels_;  // Up to 64x64 transformation
  const int MAXDIM = 16;                  // Maximum dimension in any direction
};

}  // namespace Jcy
#endif  // MODULES_EIGEN_INCLUDE_JCY_EIGEN_SADCT_H_
