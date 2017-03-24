// Copyright 2017 Caoyang Jiang

#include "GpuEstimateLibrary.cuh"

#include <cstdint>
#include <iostream>
#include <vector>

__global__ static void CUDAInterEstimateKernel(
    int16_t* in, int16_t* ref, size_t count, uint8_t* mv, int16_t* res)
{
  int id          = threadIdx.x + blockIdx.x * blockDim.x;
  uint16_t sad[4] = {65535, 65535, 65535, 65535};
  int16_t pin;
  uint8_t mvloc = 0;

  if (id < count)
  {
    pin = in[id];

    sad[0]                    = abs(pin - ref[id]);
    if ((id - 1) >= 0) sad[1] = abs(pin - ref[id - 1]);
    if ((id - 2) >= 0) sad[2] = abs(pin - ref[id - 2]);
    if ((id - 3) >= 0) sad[3] = abs(pin - ref[id - 3]);

    for (uint8_t i = 1; i < 4; i++)
    {
      if (sad[i] < sad[0])
      {
        sad[0] = sad[i];
        mvloc  = i;
      }
    }

    mv[id]  = mvloc;
    res[id] = pin - ref[id - mvloc];
  }
}

namespace jcy
{
void GpuPredictor::InterEstimate(const std::vector<int16_t>& in,
                                 const std::vector<int16_t>& ref,
                                 std::vector<uint8_t>& mv,
                                 std::vector<int16_t>& res)
{
  size_t count  = in.size();
  int thdinblk  = 512;
  int blkingrid = (count + thdinblk - 1) / thdinblk;
  int16_t* din;
  int16_t* dref;
  uint8_t* dmv;
  int16_t* dres;

  cudaMalloc(reinterpret_cast<void**>(&din), count * sizeof(int16_t));
  cudaMalloc(reinterpret_cast<void**>(&dref), count * sizeof(int16_t));
  cudaMalloc(reinterpret_cast<void**>(&dmv), count * sizeof(uint8_t));
  cudaMalloc(reinterpret_cast<void**>(&dres), count * sizeof(int16_t));

  dim3 blkdim(thdinblk, 1);
  dim3 griddim(blkingrid, 1);

  cudaMemcpy(reinterpret_cast<void*>(din),
             reinterpret_cast<const void*>(in.data()),
             count * sizeof(int16_t),
             cudaMemcpyHostToDevice);
  cudaMemcpy(reinterpret_cast<void*>(dref),
             reinterpret_cast<const void*>(ref.data()),
             count * sizeof(int16_t),
             cudaMemcpyHostToDevice);

  CUDAInterEstimateKernel<<<griddim, blkdim>>>(din, dref, count, dmv, dres);

  mv.resize(count);
  res.resize(count);
  cudaMemcpy(reinterpret_cast<void*>(mv.data()),
             reinterpret_cast<const void*>(dmv),
             count * sizeof(uint8_t),
             cudaMemcpyDeviceToHost);
  cudaMemcpy(reinterpret_cast<void*>(res.data()),
             reinterpret_cast<const void*>(dres),
             count * sizeof(int16_t),
             cudaMemcpyDeviceToHost);

  cudaFree(din);
  cudaFree(dmv);
  cudaFree(dres);
  cudaFree(dref);
}

void CpuPredictor::InterEstimate(const std::vector<int16_t>& in,
                                 const std::vector<int16_t>& ref,
                                 std::vector<uint8_t>& mv,
                                 std::vector<int16_t>& res)
{
  int count = static_cast<int>(in.size());
  mv.resize(count);
  res.resize(count);

  for (int c = 0; c < count; c++)
  {
    uint16_t sad[4] = {65535, 65535, 65535, 65535};
    int16_t pin;
    uint8_t mvloc = 0;

    pin = in[c];

    sad[0]                   = abs(pin - ref[c]);
    if ((c - 1) >= 0) sad[1] = abs(pin - ref[c - 1]);
    if ((c - 2) >= 0) sad[2] = abs(pin - ref[c - 2]);
    if ((c - 3) >= 0) sad[3] = abs(pin - ref[c - 3]);

    for (uint8_t i = 1; i < 4; i++)
    {
      if (sad[i] < sad[0])
      {
        sad[0] = sad[i];
        mvloc  = i;
      }
    }

    mv[c]  = mvloc;
    res[c] = pin - ref[c - mvloc];
  }
}
}  // namespace jcy
