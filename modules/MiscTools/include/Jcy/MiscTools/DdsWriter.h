// Copyright 2016 Caoyang Jiang

#ifndef MODULES_MISCTOOLS_INCLUDE_JCY_MISCTOOLS_DDSWRITER_H_
#define MODULES_MISCTOOLS_INCLUDE_JCY_MISCTOOLS_DDSWRITER_H_

#include <string>

namespace Jcy
{
class DdsWriter
{
 public:
  bool Write(const std::string& dxtfile,
             int width,
             int height,
             const std::string& ofile);

 private:
  typedef unsigned int
      DWORD;  // On windows, replace this line by #include <windows.h>

  typedef struct DDS_PIXELFORMAT
  {
    enum FLAGS
    {
      DDPF_ALHAPIXELS = 0x1,
      DDPF            = 0x2,
      DDPF_FOURCC     = 0x4,
      DDPF_RGB        = 0x40,
      DDPF_YUV        = 0x200,
      DDPF_LUMINANCE  = 0x20000
    };

    enum FOURCC
    {
      DXT1 = 0x31545844,
      DXT2 = 0x32545844,
      DXT3 = 0x33545844,
      DXT4 = 0x34545844,
      DXT5 = 0x35545844,
      DX10 = 0x30315844
    };

    DWORD dwSize = 0;
    DWORD dwFlags;
    DWORD dwFourCC;
    DWORD dwRGBBitCount;
    DWORD dwRBitMask;
    DWORD dwGBitMask;
    DWORD dwBBitMask;
    DWORD dwABitMask;

    DDS_PIXELFORMAT()
        : dwSize(32)
        , dwFlags(DDPF_FOURCC)
        , dwFourCC(DXT1)
        , dwRGBBitCount(0)
        , dwRBitMask(0)
        , dwGBitMask(0)
        , dwBBitMask(0)
        , dwABitMask(0)
    {
    }
  } DDS_PIXELFORMAT;

  typedef struct DDS_HEADER
  {
    DWORD dwSize;
    DWORD dwFlags;
    DWORD dwHeight;
    DWORD dwWidth;
    DWORD dwPitchOrLinearSize;
    DWORD dwDepth;
    DWORD dwMipMapCount;
    DWORD dwReserved1[11];
    DDS_PIXELFORMAT ddspf;
    DWORD dwCaps;
    DWORD dwCaps2;
    DWORD dwCaps3;
    DWORD dwCaps4;
    DWORD dwReserved2;

    DDS_HEADER()
        : dwSize(124)
        , dwFlags(0x1007)
        , dwHeight(0)
        , dwWidth(0)
        , dwPitchOrLinearSize(0)
        , dwDepth(0)
        , dwMipMapCount(0)
        , dwReserved1{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
        , ddspf()
        , dwCaps(0x1000)
        , dwCaps2(0)
        , dwCaps3(0)
        , dwCaps4(0)
        , dwReserved2(0)
    {
    }
  } DDS_HEADER;

  typedef struct DDS
  {
    DWORD dwMagic;
    DDS_HEADER header;
    DDS()
        : dwMagic(0x20534444)  // Magic number
        , header()
    {
    }
  } DDS;
};
}  // namespace Jcy
#endif  // MODULES_MISCTOOLS_INCLUDE_JCY_MISCTOOLS_DDSWRITER_H_
