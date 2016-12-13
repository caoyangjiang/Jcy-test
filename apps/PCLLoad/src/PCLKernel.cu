// Copyright 2016 Caoyang Jiang

#include <cstdint>
#include <iostream>
#include <vector>

#define NNEIGHBOR 10

__device__ static bool CUDAFuncIsValInArray(int* array, int len, int val)
{
  for (int i = 0; i < len; i++)
  {
    if (array[i] == val) return true;
  }

  return false;
}
__global__ static void CUDAKernelFindNeighbor(size_t pointcnt,
                                              size_t polygoncnt,
                                              uint32_t* v1,
                                              uint32_t* v2,
                                              uint32_t* v3,
                                              int (*connectivity)[NNEIGHBOR])
{
  int neighbor[NNEIGHBOR];
  int neighborcnt = 0;
  int it          = 0;
  const int x     = blockIdx.x * blockDim.x + threadIdx.x;
  const int mx    = gridDim.x * blockDim.x;

  for (int in = 0; in < NNEIGHBOR; in++)
  {
    neighbor[in] = -1;
  }

  while ((x + it * mx) < pointcnt)
  {
    uint32_t idx = x + it * mx;

    for (size_t polysz = 0; polysz < polygoncnt; polysz++)
    {
      uint32_t v[3];

      v[0] = v1[polysz];
      v[1] = v2[polysz];
      v[2] = v3[polysz];

      if (idx == v[0])
      {
        if (!CUDAFuncIsValInArray(neighbor, NNEIGHBOR, v[1]))
        {
          neighbor[neighborcnt] = v[1];
          neighborcnt++;
        }

        if (!CUDAFuncIsValInArray(neighbor, NNEIGHBOR, v[2]))
        {
          neighbor[neighborcnt] = v[2];
          neighborcnt++;
        }
      }

      if (idx == v[1])
      {
        if (!CUDAFuncIsValInArray(neighbor, NNEIGHBOR, v[0]))
        {
          neighbor[neighborcnt] = v[0];
          neighborcnt++;
        }

        if (!CUDAFuncIsValInArray(neighbor, NNEIGHBOR, v[2]))
        {
          neighbor[neighborcnt] = v[2];
          neighborcnt++;
        }
      }

      if (idx == v[2])
      {
        if (!CUDAFuncIsValInArray(neighbor, NNEIGHBOR, v[0]))
        {
          neighbor[neighborcnt] = v[0];
          neighborcnt++;
        }

        if (!CUDAFuncIsValInArray(neighbor, NNEIGHBOR, v[1]))
        {
          neighbor[neighborcnt] = v[1];
          neighborcnt++;
        }
      }
    }

    for (int in = 0; in < NNEIGHBOR; in++)
    {
      connectivity[idx][in] = neighbor[in];
      neighbor[in]          = -1;
    }
    // std::sort(connectivity[sz].begin(), connectivity[sz].end(),
    // CompS2LU32);
    it++;
    neighborcnt = 0;
  }
}

bool FindNeighbor(int pointcnt,
                  const std::vector<std::vector<uint32_t>>& polygons,
                  std::vector<std::vector<uint32_t>>& connectivity)
{
  uint32_t* data[3];

  int(*cudaconnectivity)[NNEIGHBOR];

  cudaMallocManaged(reinterpret_cast<void**>(&data[0]),
                    polygons.size() * sizeof(uint32_t),
                    cudaMemAttachGlobal);
  cudaMallocManaged(reinterpret_cast<void**>(&data[1]),
                    polygons.size() * sizeof(uint32_t),
                    cudaMemAttachGlobal);
  cudaMallocManaged(reinterpret_cast<void**>(&data[2]),
                    polygons.size() * sizeof(uint32_t),
                    cudaMemAttachGlobal);
  cudaMallocManaged(reinterpret_cast<void**>(&cudaconnectivity),
                    pointcnt * sizeof(int[NNEIGHBOR]),
                    cudaMemAttachGlobal);

  std::cout << polygons.size() << std::endl;
  for (size_t sz = 0; sz < polygons.size(); sz++)
  {
    data[0][sz] = polygons[sz][0];
    data[1][sz] = polygons[sz][1];
    data[2][sz] = polygons[sz][2];
  }

  dim3 blks(64, 1);
  dim3 threads(256, 1);

  CUDAKernelFindNeighbor<<<blks, threads>>>(
      pointcnt, polygons.size(), data[0], data[1], data[2], cudaconnectivity);

  cudaDeviceSynchronize();

  for (int ip = 0; ip < pointcnt; ip++)
  {
    std::vector<uint32_t> tmp;
    for (int in = 0; in < NNEIGHBOR; in++)
    {
      if (cudaconnectivity[ip][in] != -1)
      {
        tmp.push_back(cudaconnectivity[ip][in]);
      }
    }
    connectivity.push_back(tmp);
  }

  cudaFree(data[0]);
  cudaFree(data[1]);
  cudaFree(data[2]);
  cudaFree(cudaconnectivity);

  return true;
}
