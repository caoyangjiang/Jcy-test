// Copyright 2016 Caoyang Jiang

#ifndef MODULES_GEOMETRYCODER_INCLUDE_JCY_GEOMETRYCODER_CONVERTOR_H_
#define MODULES_GEOMETRYCODER_INCLUDE_JCY_GEOMETRYCODER_CONVERTOR_H_

#include <string>
#include "opencv2/opencv.hpp"

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
#endif  // MODULES_GEOMETRYCODER_INCLUDE_JCY_GEOMETRYCODER_CONVERTOR_H_
