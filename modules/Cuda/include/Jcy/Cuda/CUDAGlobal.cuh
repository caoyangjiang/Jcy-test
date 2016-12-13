// Copyright 2015 Jason Juang

#ifndef MODULES_CUDATEST_INC_CUDAGLOBAL_CUH_
#define MODULES_CUDATEST_INC_CUDAGLOBAL_CUH_

#include <cuda.h>
#include <cuda_runtime_api.h>
#include <iostream>

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

/** @brief      CUDA configurations
  * @author     Jason Juang
  * @attention  This is for internal use only, DO NOT distribute the code
  */
struct CUDAConfig
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

#endif  // MODULES_CUDATEST_INC_CUDAGLOBAL_CUH_
