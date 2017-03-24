// Copyright 2017 Caoyang Jiang

#include <chrono>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

#include "GpuEstimateLibrary.cuh"

int main(int, char**)
{
  std::default_random_engine generator;
  std::uniform_int_distribution<int16_t> dist(
      0, std::numeric_limits<int16_t>::max());
  std::chrono::high_resolution_clock::time_point time =
      std::chrono::high_resolution_clock::now();
  std::chrono::high_resolution_clock::duration dtn = time.time_since_epoch();
  generator.seed(dtn.count());

  std::chrono::high_resolution_clock::time_point beg;
  std::chrono::high_resolution_clock::time_point end;
  std::chrono::duration<double, std::milli> milliseconds;

  std::vector<int16_t> in;
  std::vector<int16_t> ref;
  std::vector<int16_t> res;
  std::vector<uint8_t> mv;

  for (size_t i = 0; i < 120000; i++)
  {
    int16_t val = dist(generator);
    in.push_back(val);
  }

  for (size_t i = 1; i < in.size(); i++)
  {
    ref.push_back(in[i] + 1);  // make res all 1s and mv all 1s
  }

  ref.push_back(in[in.size() - 1]);

  beg = std::chrono::high_resolution_clock::now();
  jcy::GpuPredictor::InterEstimate(in, ref, mv, res);
  end          = std::chrono::high_resolution_clock::now();
  milliseconds = end - beg;
  std::cout << "[Gpu time]: " << milliseconds.count() << " ms" << std::endl;

  beg = std::chrono::high_resolution_clock::now();
  jcy::CpuPredictor::InterEstimate(in, ref, mv, res);
  end          = std::chrono::high_resolution_clock::now();
  milliseconds = end - beg;
  std::cout << "[Cpu time]: " << milliseconds.count() << " ms" << std::endl;

  return 0;
}
