// Copyright 2016 Caoyang Jiang

#ifndef MODULES_OPENCV_INCLUDE_JCY_OPENCV_CONVERTOR_H_
#define MODULES_OPENCV_INCLUDE_JCY_OPENCV_CONVERTOR_H_

#include <opencv2/opencv.hpp>
#include <string>

namespace Jcy
{
class Convertor
{
 public:
  Convertor();
  ~Convertor();

  bool Yml2Raw(const std::string ymlfile,
               const std::string rawfile,
               int mode) const;
};
}  // namespace Jcy
#endif  // MODULES_OPENCV_INCLUDE_JCY_OPENCV_CONVERTOR_H_
