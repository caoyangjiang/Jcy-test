// Copyright 2016 Caoyang Jiang

#ifndef MODULES_IMAGEPROCESS_INCLUDE_JCY_IMAGEPROCESS_CONVERTOR_H_
#define MODULES_IMAGEPROCESS_INCLUDE_JCY_IMAGEPROCESS_CONVERTOR_H_

#include <string>
#include "opencv2/opencv.hpp"

namespace jcy
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
}  // namespace jcy
#endif  // MODULES_IMAGEPROCESS_INCLUDE_JCY_IMAGEPROCESS_CONVERTOR_H_
