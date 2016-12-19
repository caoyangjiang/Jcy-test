// Copyright @ 2016 Caoyang Jiang

#include <chrono>
#include <iomanip>
#include <iostream>

#include "Jcy/EntropyCoder/ArithmeticEngine.h"

int main()
{
  Jcy::ArithmeticEngine<uint16_t> ae(
      Jcy::ArithmeticEngine<uint16_t>::MODE::ENCODE);
  Jcy::ArithmeticEngine<uint16_t> ad(
      Jcy::ArithmeticEngine<uint16_t>::MODE::DECODE);

  std::vector<uint16_t> symbols;
  std::vector<uint64_t> freq;
  const size_t totalsymbol = 5;

  for (size_t i = 0; i < totalsymbol; i++)
  {
    symbols.push_back(static_cast<uint8_t>(i));
    freq.push_back(1);
  }

  std::chrono::high_resolution_clock::time_point beg;
  std::chrono::high_resolution_clock::time_point end;
  std::chrono::duration<double, std::milli> milliseconds;

  beg = std::chrono::high_resolution_clock::now();
  ae.LoadProbabilityModel(freq);
  ae.Encode(reinterpret_cast<const uint16_t*>(symbols.data()), totalsymbol);
  end = std::chrono::high_resolution_clock::now();

  milliseconds = end - beg;
  std::cout << "Encode Speed: " << totalsymbol * 1000 / milliseconds.count()
            << " symb/sec." << std::endl;
  std::cout << "Bits: " << ae.GetCodedBitsCount() << std::endl;
  for (size_t i = 0; i < (ae.GetCodedBitsCount() + 7) / 8; i++)
  {
    std::printf("%x ",*(ae.GetCodedBits() + i));
  }
  ad.LoadProbabilityModel(freq);
  ad.Decode(reinterpret_cast<const uint8_t*>(ae.GetCodedBits()),
            ae.GetCodedBitsCount(),
            totalsymbol);

  std::cout << std::endl;
  return 0;
}
