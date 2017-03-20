// Copyright @ 2016 Caoyang Jiang

#include "Jcy/ImageProcess/Convertor.h"

#include <fstream>
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"

namespace jcy
{
Convertor::Convertor()
{
}

Convertor::~Convertor()
{
}

bool Convertor::Yml2Raw(const std::string ymlfile,
                        const std::string rawfile,
                        int mode) const
{
  int16_t max    = 0;
  int16_t bufint = 0;
  float maxf     = 0.0;

  cv::Mat imgin;
  std::fstream outfile;
  std::ios_base::openmode omode = std::fstream::out | std::fstream::binary;

  if (mode == 0)
    omode |= std::fstream::trunc;
  else
    omode |= std::fstream::app;

  outfile.open(rawfile, omode);
  cv::FileStorage inputDepth(ymlfile, cv::FileStorage::READ);
  inputDepth.getFirstTopLevelNode() >> imgin;
  inputDepth.release();

  for (int rows = 0; rows < imgin.size().height; rows++)
  {
    for (int cols = 0; cols < imgin.size().width; cols++)
    {
      float buf = imgin.at<float>(rows, cols);
      buf       = buf * 100;
      bufint    = static_cast<int16_t>(buf);

      if (buf > 65535.0)
      {
        std::cout << "[ERROR]: Exceed 16 bit!" << std::endl;
        return false;
      }

      if (bufint > max) max = bufint;
      if (buf > maxf) maxf  = buf;

      outfile.write(reinterpret_cast<char*>(&bufint), sizeof(int16_t));
    }
  }

  outfile.close();
  return true;
}
}  // namespace jcy
