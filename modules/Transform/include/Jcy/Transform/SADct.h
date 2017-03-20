// Copyright 2016 Caoyang Jiang

#ifndef MODULES_TRANSFORM_INCLUDE_JCY_TRANSFORM_SADCT_H_
#define MODULES_TRANSFORM_INCLUDE_JCY_TRANSFORM_SADCT_H_

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
 * @brief  S.Thomas & M.Bela, "Shape-Adaptive DCT for Generic Coding of Video"
 */
class SADct : public Dct
{
 public:
  JCY_TRANSFORM_DLL SADct();
  JCY_TRANSFORM_DLL ~SADct() override;

  JCY_TRANSFORM_DLL bool Forward(
      const std::vector<std::vector<double>>& datain,
      std::vector<std::vector<double>>& dataout) override;
  JCY_TRANSFORM_DLL bool Inverse(
      const std::vector<std::vector<double>>& datain,
      std::vector<std::vector<double>>& dataout) override;

  JCY_TRANSFORM_DLL bool SetContour(
      const std::vector<std::vector<bool>>& contour);

 private:
  std::vector<std::vector<bool>> contour_;
  std::vector<Eigen::MatrixXd> kernels_;  // Up to 64x64 transformation
  const uint32_t MAXDIM = 16;             // Maximum dimension in any direction
  const double PI       = 3.14159265;
};

}  // namespace jcy
#endif  // MODULES_TRANSFORM_INCLUDE_JCY_TRANSFORM_SADCT_H_
