// Copyright 2015 Jason Juang

#include "Jcy/Cuda/CUDATest.cuh"

__global__ static void CUDAKernelAddOneToArray(int *data)
{
  const int x  = blockIdx.x * blockDim.x + threadIdx.x;
  const int y  = blockIdx.y * blockDim.y + threadIdx.y;
  const int mx = gridDim.x * blockDim.x;

  data[y * mx + x] = data[y * mx + x] + 1.0f;
}

namespace jcy
{
CUDATest::CUDATest()
{
}

CUDATest::~CUDATest()
{
}

void CUDATest::AddOneToArray(std::vector<int> &in)
{
  if (in.empty()) return;

  cudaconfig_.w_a = static_cast<int>(in.size()) / cudaconfig_.blk_h;
  cudaconfig_.h_a = cudaconfig_.blk_h;

  cudaMallocManaged(reinterpret_cast<void **>(&data_),
                    in.size() * sizeof(int),
                    cudaMemAttachGlobal);

  for (int i = 0; i < in.size(); i++)
  {
    data_[i] = in.at(i);
  }

  dim3 blks((cudaconfig_.w_a >> cudaconfig_.shift_w),
            (cudaconfig_.h_a >> cudaconfig_.shift_h));
  dim3 threads(cudaconfig_.blk_w, cudaconfig_.blk_h);

  CUDAKernelAddOneToArray<<<blks, threads>>>(data_);

  cudaDeviceSynchronize();

  for (int i = 0; i < in.size(); i++)
  {
    in.at(i) = data_[i];
  }

  cudaFree(data_);
}

}  // namespace jcy
