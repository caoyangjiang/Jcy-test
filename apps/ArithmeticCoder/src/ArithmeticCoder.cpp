// Copyright @ 2016 Caoyang Jiang

#include <chrono>
#include <iomanip>
#include <iostream>

#include "Jcy/EntropyCoder/ArithmeticEngine.h"

int main()
{
  Jcy::ArithmeticEngine<uint32_t> ae(
      Jcy::ArithmeticEngine<uint32_t>::MODE::ENCODE);
  Jcy::ArithmeticEngine<uint32_t> ad(
      Jcy::ArithmeticEngine<uint32_t>::MODE::DECODE);

  std::vector<uint32_t> symbols;
  std::vector<uint64_t> freq;
  const size_t totalsymbol = 100;

  for (size_t i = 0; i < totalsymbol; i++)
  {
    symbols.push_back(i);
    freq.push_back(1);
  }

  std::chrono::high_resolution_clock::time_point beg;
  std::chrono::high_resolution_clock::time_point end;
  std::chrono::duration<double, std::milli> milliseconds;

  beg = std::chrono::high_resolution_clock::now();
  ae.LoadProbabilityModel(freq);
  ae.Encode(reinterpret_cast<const uint32_t*>(symbols.data()), totalsymbol);
  end = std::chrono::high_resolution_clock::now();

  milliseconds = end - beg;
  std::cout << "Encode Speed: " << totalsymbol * 1000 / milliseconds.count()
            << " symb/sec." << std::endl;
  std::cout << "Bits: " << ae.GetCodedBitsCount() << std::endl;
  for (size_t i = 0; i < (ae.GetCodedBitsCount() + 7) / 8; i++)
  {
    std::cout << std::hex << static_cast<uint32_t>(*(ae.GetCodedBits() + i))
              << " ";
  }
  ad.LoadProbabilityModel(freq);
  ad.Decode(reinterpret_cast<const uint8_t*>(ae.GetCodedBits()),
            ae.GetCodedBitsCount(),
            totalsymbol);

  std::cout << std::endl;
  return 0;
}
