// Copyright 2016 Caoyang Jiang

#include <cuda_runtime.h>
#include "Jcy/Cuda/CudaColorSpaceCVT.h"

__global__ static void CudaKernelYuv420ToRgb(int* data)
{
  const int x  = blockIdx.x * blockDim.x + threadIdx.x;
  const int y  = blockIdx.y * blockDim.y + threadIdx.y;
  const int mx = gridDim.x * blockDim.x;

  data[y * mx + x] = data[y * mx + x] + 1.0f;
}

namespace jcy
{
bool CudaColorSpaceCVT::CpuConvert(const uint8_t* in)
{
  return true;
}
bool CudaColorSpaceCVT::GpuConvert(const uint8_t* in)
{
  cudaError_t error;
  if (incs_ == ColorSpace::YUV420)
  {
    // Copy first component
    if ((error = cudaMemcpy2D(reinterpret_cast<void*>(incompptr_[0]),
                              static_cast<size_t>(kMAXSIZE),
                              reinterpret_cast<const void*>(in),
                              static_cast<size_t>(width_),
                              static_cast<size_t>(width_),
                              static_cast<size_t>(height_),
                              cudaMemcpyHostToDevice)) != cudaSuccess)
    {
      std::cout << "[ERROR]: Copy input first component onto GPU failed, "
                << cudaGetErrorString(error) << std::endl;
      return false;
    }

    // Copy Second component
    if ((error =
             cudaMemcpy2D(reinterpret_cast<void*>(incompptr_[1]),
                          static_cast<size_t>(kMAXSIZE),
                          reinterpret_cast<const void*>(in + width_ * height_),
                          static_cast<size_t>(width_ / 2),
                          static_cast<size_t>(width_ / 2),
                          static_cast<size_t>(height_ / 2),
                          cudaMemcpyHostToDevice)) != cudaSuccess)
    {
      std::cout << "[ERROR]: Copy input second component onto GPU failed, "
                << cudaGetErrorString(error) << std::endl;
      return false;
    }

    // Copy Third component
    if ((error = cudaMemcpy2D(
             reinterpret_cast<void*>(incompptr_[2]),
             static_cast<size_t>(kMAXSIZE),
             reinterpret_cast<const void*>(in + width_ * height_ * 5 / 4),
             static_cast<size_t>(width_ / 2),
             static_cast<size_t>(width_ / 2),
             static_cast<size_t>(height_ / 2),
             cudaMemcpyHostToDevice)) != cudaSuccess)
    {
      std::cout << "[ERROR]: Copy input third component onto GPU failed, "
                << cudaGetErrorString(error) << std::endl;
      return false;
    }
  }
  else if (incs_ == ColorSpace::RGB444)
  {
    for (int icomp = 0; icomp < 3; icomp++)
    {
      // Copy first component
      if ((error = cudaMemcpy2D(
               reinterpret_cast<void*>(incompptr_[icomp]),
               static_cast<size_t>(kMAXSIZE),
               reinterpret_cast<const void*>(in + icomp * width_ * height_),
               static_cast<size_t>(width_),
               static_cast<size_t>(width_),
               static_cast<size_t>(height_),
               cudaMemcpyHostToDevice)) != cudaSuccess)
      {
        std::cout << "[ERROR]: Copy input " << icomp
                  << " component onto GPU failed, " << cudaGetErrorString(error)
                  << std::endl;
        return false;
      }
    }
  }
  else
  {
    std::cout << "[ERROR]: Unsupported input color space." << std::endl;
    return false;
  }

  //
  return true;
}

CudaColorSpaceCVT::CudaColorSpaceCVT()
{
  cudaError_t error;

  if ((error = cudaMallocManaged(reinterpret_cast<void**>(&inbuffer),
                                 kMAXSIZE * kMAXSIZE * kMAXCOMP)) !=
      cudaSuccess)
  {
    std::cout << "[ERROR]: Malloc buffer failed, " << cudaGetErrorString(error)
              << std::endl;
    exit(1);
  }

  if ((error = cudaMallocManaged(reinterpret_cast<void**>(&cvtbuffer),
                                 kMAXSIZE * kMAXSIZE * kMAXCOMP)) !=
      cudaSuccess)
  {
    std::cout << "[ERROR]: Malloc buffer failed, " << cudaGetErrorString(error)
              << std::endl;
    exit(1);
  }

  // For conenience purpose.
  incompptr_[0] = inbuffer;
  incompptr_[1] = inbuffer + kMAXSIZE;
  incompptr_[2] = inbuffer + 2 * kMAXSIZE;

  cvtcompptr_[0] = cvtbuffer;
  cvtcompptr_[1] = cvtbuffer + kMAXSIZE;
  cvtcompptr_[2] = cvtbuffer + 2 * kMAXSIZE;
}

CudaColorSpaceCVT::~CudaColorSpaceCVT()
{
}

void CudaColorSpaceCVT::SetWidth(int width)
{
  width_ = width;
}

void CudaColorSpaceCVT::SetHeight(int height)
{
  height_ = height;
}

void CudaColorSpaceCVT::SetHardware(int hardware)
{
  hardware_ = hardware;
}

void CudaColorSpaceCVT::SetInOutCS(enum ColorSpace incs, enum ColorSpace outcs)
{
  incs_  = incs;
  outcs_ = outcs;
}

bool CudaColorSpaceCVT::Convert(const uint8_t* fb)
{
  if (hardware_ == 0)
  {
    return CpuConvert(fb);
  }
  else if (hardware_ == 1)
  {
    return GpuConvert(fb);
  }
  else
  {
    std::cout << "[ERROR]: Unsupported hardware type " << hardware_
              << std::endl;
    return false;
  }
}

const uint8_t* CudaColorSpaceCVT::GetConvertedFrm() const
{
  return cvtbuffer;
}

}  // namespace jcy
