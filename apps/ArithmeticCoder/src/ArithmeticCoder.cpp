// Copyright @ 2016 Caoyang Jiang

#include <chrono>
#include <iomanip>
#include <iostream>
#include <vector>

#include "Jcy/EntropyCoder/ArithmeticEngine.h"

int main()
{
  jcy::ArithmeticEngine<uint16_t> ae(
      jcy::ArithmeticEngine<uint16_t>::MODE::ENCODE);
  jcy::ArithmeticEngine<uint16_t> ad(
      jcy::ArithmeticEngine<uint16_t>::MODE::DECODE);

  std::vector<uint16_t> symbols;
  std::vector<uint64_t> freq;
  const size_t totalsymbol = 1000;
  for (size_t i = 0; i < totalsymbol; i++)
  {
    symbols.push_back(static_cast<uint16_t>(i));
    freq.push_back(1);
  }

  freq[3]    = 10000;
  freq[65]   = 9534;
  symbols[0] = 1;
  freq[0]    = 0;

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

  ad.LoadProbabilityModel(freq);
  ad.Decode(reinterpret_cast<const uint8_t*>(ae.GetCodedBits()),
            ae.GetCodedBitsCount(),
            totalsymbol);

  for (size_t i = 0; i < totalsymbol; i++)
  {
    if (symbols[i] != ad.GetDecodedSymbols()[i])
    {
      std::cout << "Verify failed at: " << i << std::endl;
      break;
    }
  }
  return 0;
}
