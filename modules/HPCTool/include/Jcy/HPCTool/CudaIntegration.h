// Copyright 2016 Caoyang Jiang

#ifndef MODULES_HPCTOOL_INCLUDE_JCY_HPCTOOL_CUDAINTEGRATION_H_
#define MODULES_HPCTOOL_INCLUDE_JCY_HPCTOOL_CUDAINTEGRATION_H_

#include <vector>

namespace jcy
{
class CudaIntegration
{
 public:
  CudaIntegration();
  ~CudaIntegration();

  /**
   * @brief      Sets the color conversion width.
   *
   * @param[in]  width  The width
   */

  bool StartIntegration(double startx,
                        double endx,
                        size_t tcount,
                        std::vector<double>& area);
};

}  // namespace jcy

#endif  // MODULES_HPCTOOL_INCLUDE_JCY_HPCTOOL_CUDAINTEGRATION_H_
