// Copyright 2017 Caoyang Jiang

#include <iostream>
#include <vector>

namespace jcy
{
class GpuPredictor
{
 public:
  static void InterEstimate(const std::vector<int16_t>& in,
                            const std::vector<int16_t>& ref,
                            std::vector<uint8_t>& mv,
                            std::vector<int16_t>& res);
};

class CpuPredictor
{
 public:
  static void InterEstimate(const std::vector<int16_t>& in,
                            const std::vector<int16_t>& ref,
                            std::vector<uint8_t>& mv,
                            std::vector<int16_t>& res);
};
}  // namespace hvr
