// Copyright 2016 Caoyang Jiang

#ifndef MODULES_CUDA_INCLUDE_JCY_CUDA_CUDACOLORSPACECVT_H_
#define MODULES_CUDA_INCLUDE_JCY_CUDA_CUDACOLORSPACECVT_H_

#include <array>
#include <iostream>

#include "Jcy/Cuda/CudaConfig.h"

namespace jcy
{
enum struct ColorSpace
{
  YUV420 = 0,
  RGB444
};

class CudaColorSpaceCVT
{
 public:
  CudaColorSpaceCVT();
  ~CudaColorSpaceCVT();

  CudaColorSpaceCVT(const CudaColorSpaceCVT&) = delete;
  CudaColorSpaceCVT& operator=(const CudaColorSpaceCVT&) = delete;

  /**
   * @brief      Sets the color conversion width.
   *
   * @param[in]  width  The width
   */
  void SetWidth(int width);

  /**
   * @brief      Sets the color conversion height.
   *
   * @param[in]  height  The height
   */
  void SetHeight(int height);

  /**
   * @brief      Set hardware type. 0 for CPU and 1 for GPU.
   *
   * @param[in]  hardware  The hardware type
   */
  void SetHardware(int hardware);

  /**
   * @brief      Sets input and output color space type.
   *
   * @param[in]  in    Input color space type.
   * @param[in]  out   Output color space type.
   */
  void SetInOutCS(enum ColorSpace in, enum ColorSpace out);

  /**
   * @brief      Perform CPU based conversion.
   *
   * @param[in]  in    Input frame buffer.
   *
   * @return     True if conversion successful, false otherwise.
   */
  bool Convert(const uint8_t* in);

  /**
   * @brief      Gets the converted frame buffer.
   *
   * @return     The converted frame buffer.
   */
  const uint8_t* GetConvertedFrm() const;

  /*
   * @brief      Maximum supported conversion size for width and height.
   */
  static const int kMAXSIZE = 8192;

  /*
   * @brief      Maximum supported conversion components.
   */
  static const int kMAXCOMP = 3;

 private:
  bool CpuConvert(const uint8_t* in);
  bool GpuConvert(const uint8_t* in);

 private:
  uint8_t* inbuffer  = nullptr;
  uint8_t* cvtbuffer = nullptr;
  std::array<uint8_t*, kMAXCOMP> incompptr_;
  std::array<uint8_t*, kMAXCOMP> cvtcompptr_;

  int hardware_          = 0;
  int width_             = 0;
  int height_            = 0;
  enum ColorSpace incs_  = ColorSpace::YUV420;
  enum ColorSpace outcs_ = ColorSpace::YUV420;
};

}  // namespace jcy

#endif  // MODULES_CUDA_INCLUDE_JCY_CUDA_CUDACOLORSPACECVT_H_
