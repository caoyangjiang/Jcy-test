// Copyright 2016 Caoyang Jiang

#ifndef MODULES_TRANSFORM_INCLUDE_JCY_TRANSFORM_DCT_H_
#define MODULES_TRANSFORM_INCLUDE_JCY_TRANSFORM_DCT_H_

JCY_WINDOWS_DISABLE_ALL_WARNING
#include <vector>
JCY_WINDOWS_DISABLE_ALL_WARNING

namespace jcy
{
class Dct
{
 protected:
  Dct();
  virtual ~Dct();

 public:
  // Forbid copying
  Dct(Dct const&) = delete;
  Dct& operator=(Dct const&) = delete;

  virtual bool Forward(const std::vector<std::vector<double>>& datain,
                       std::vector<std::vector<double>>& dataout) = 0;
  virtual bool Inverse(const std::vector<std::vector<double>>& datain,
                       std::vector<std::vector<double>>& dataout) = 0;
};

}  // namespace jcy
#endif  // MODULES_TRANSFORM_INCLUDE_JCY_TRANSFORM_DCT_H_
