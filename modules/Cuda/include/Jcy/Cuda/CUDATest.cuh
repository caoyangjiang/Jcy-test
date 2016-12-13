// Copyright 2015 Jason Juang

#ifndef MODULES_CUDATEST_INC_CUDATEST_CUH_
#define MODULES_CUDATEST_INC_CUDATEST_CUH_

#include <iostream>
#include <vector>

#include "Jcy/Cuda/CUDAGlobal.cuh"

namespace hvr
{
class CUDATest
{
 public:
  CUDATest();
  ~CUDATest();

  void AddOneToArray(std::vector<int> &in);

 private:
  int *data_;

  CUDAConfig cudaconfig_;
};

}  // namespace hvr

#endif  // MODULES_CUDATEST_INC_CUDATEST_CUH_
