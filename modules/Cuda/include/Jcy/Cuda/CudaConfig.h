// Copyright 2016 Caoyang Jiang

#ifndef MODULES_CUDA_INCLUDE_JCY_CUDA_CUDACONFIG_H_
#define MODULES_CUDA_INCLUDE_JCY_CUDA_CUDACONFIG_H_

#include <cuda.h>
#include <cuda_runtime_api.h>
#include <iostream>

namespace jcy
{
/**
 * @brief      This will align 'n' so that it is a multiple of 'a'
 *
 * @param[in]  n     input
 * @param[in]  a     number to align to
 *
 * @return     aligned value
 */
inline int align(const int n, const int a)

{
  const int r = (n & (a - 1));
  if (r == 0)
    return n;
  else
    return n - r + a;
}

/**
 * @brief      CUDA configurations
 */
struct CudaConfig
{
  // ID to identify GPU
  int id;

  int h, w;
  int h_a, w_a;

  // Cuda kernel defines
  int blk_h   = 16;
  int blk_w   = 32;
  int shift_h = 4;
  int shift_w = 5;

  cudaStream_t s;
};

}  // namespace jcy

#endif  // MODULES_CUDA_INCLUDE_JCY_CUDA_CUDACONFIG_H_
