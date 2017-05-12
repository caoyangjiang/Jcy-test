// Copyright 2016 Caoyang Jiang

#include <cuda_runtime.h>
#include <cstdio>
#include "Jcy/HPCTool/CudaIntegration.h"

__device__ static double XSQUARE(double x)
{
  // return x * x;
  // return x * x + 2 * x;
  return sin(x);
  // return pow(x, 2);
}

__global__ static void IntegrationKernal(double startx,
                                         double increment,
                                         double* psum)
{
  const int x  = blockIdx.x * blockDim.x + threadIdx.x;
  const int y  = blockIdx.y * blockDim.y + threadIdx.y;
  const int mx = gridDim.x * blockDim.x;

  // for (size_t i      = 0; i < 100000; i++)
  psum[y * mx + x] = (XSQUARE(startx + (y * mx + x) * increment) +
                      XSQUARE(startx + (y * mx + x + 1) * increment)) *
                     increment / 2;
  // std::printf("%d %1.12f\n", y * mx + x, psum[y * mx + x]);
}

namespace jcy
{
CudaIntegration::CudaIntegration()
{
}

CudaIntegration::~CudaIntegration()
{
}

bool CudaIntegration::StartIntegration(double startx,
                                       double endx,
                                       size_t tcount,
                                       std::vector<double>& area)
{
  double increment = (endx - startx) / tcount;
  double* d_area;

  cudaMalloc(reinterpret_cast<void**>(&d_area), sizeof(double) * tcount);

  dim3 blks(tcount / 256, 1);
  dim3 threads(256, 1);
  IntegrationKernal<<<blks, threads>>>(startx, increment, d_area);
  cudaDeviceSynchronize();

  area.assign(tcount, 0);
  cudaMemcpy(
      area.data(), d_area, sizeof(double) * tcount, cudaMemcpyDeviceToHost);
  cudaFree(d_area);

  return true;
}

}  // namespace jcy
