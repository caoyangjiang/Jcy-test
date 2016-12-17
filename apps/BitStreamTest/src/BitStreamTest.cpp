// Copyright @ 2016 Caoyang Jiang

#include <chrono>
#include <iostream>
#include <list>
#include <memory>
#include <vector>

#include "Jcy/MiscTools/BitStream.h"

using Jcy::BitStream;

int main()
{
  BitStream bs0(BitStream::MODE::WR, BitStream::ENDIAN::LITTLE);
  BitStream bs1(BitStream::MODE::RD, BitStream::ENDIAN::LITTLE);

  const int zero                  = 0;
  const int one                   = 1;
  const uint8_t zeroone           = 0x02;
  const uint8_t bitarray[1000000] = {0};

  for (size_t i = 0; i < 1000; i++)
  {
    bs0.Write(reinterpret_cast<const uint8_t*>(&zero), 1);
    bs0.Write(reinterpret_cast<const uint8_t*>(&one), 1);
  }

  if (bs0.GetWrittenSize() == 2000)
  {
    std::cout << "Single bit push length test passed " << std::endl;
  }

  std::cout << *reinterpret_cast<const uint16_t*>(bs0.GetBitBuffer())
            << std::endl;
  bs0.Reset();

  for (size_t i = 0; i < 1000; i++)
  {
    bs0.Write(reinterpret_cast<const uint8_t*>(&zeroone), 2);
  }

  if (bs0.GetWrittenSize() == 2000)
  {
    std::cout << "Double bit push length test passed " << std::endl;
  }
  std::cout << *reinterpret_cast<const uint16_t*>(bs0.GetBitBuffer())
            << std::endl;
  bs0.Reset();

  for (size_t i = 0; i < 1000; i++)
  {
    bs0.Write(reinterpret_cast<const uint8_t*>(&zeroone), 3);
  }

  if (bs0.GetWrittenSize() == 3000)
  {
    std::cout << "Triple bit push length test passed " << std::endl;
  }

  std::cout << *reinterpret_cast<const uint16_t*>(bs0.GetBitBuffer())
            << std::endl;
  bs0.Reset();

  std::chrono::high_resolution_clock::time_point beg;
  std::chrono::high_resolution_clock::time_point end;
  std::chrono::duration<double, std::milli> milliseconds;

  beg = std::chrono::high_resolution_clock::now();
  for (size_t i = 0; i < 1000000; i++)
  {
    bs0.Write(reinterpret_cast<const uint8_t*>(&one), 1);
  }

  end = std::chrono::high_resolution_clock::now();

  milliseconds = end - beg;
  std::cout << "Write 1M bits took " << milliseconds.count() << " ms."
            << std::endl;

  beg = std::chrono::high_resolution_clock::now();
  bs0.Write(reinterpret_cast<const uint8_t*>(bitarray), 1000000);
  end = std::chrono::high_resolution_clock::now();

  milliseconds = end - beg;
  std::cout << "Write 1M bits took " << milliseconds.count() << " ms."
            << std::endl;

  bs1.Load(reinterpret_cast<const uint8_t*>(bitarray), 960000);

  beg = std::chrono::high_resolution_clock::now();
  bs1.Read(960000);
  end          = std::chrono::high_resolution_clock::now();
  milliseconds = end - beg;
  std::cout << "Reading 1M bits once took " << milliseconds.count() << " ms."
            << std::endl;

  if (bs1.GetRemSize() == 0)
  {
    std::cout << "Reading 1M bits once correct" << std::endl;
  }

  bs1.Reset();

  beg = std::chrono::high_resolution_clock::now();
  for (size_t i = 0; i < 60000; i++)
  {
    bs1.Read(16);
  }
  end          = std::chrono::high_resolution_clock::now();
  milliseconds = end - beg;
  std::cout << bs1.GetRemSize() << std::endl;
  std::cout << "Reading 1M bits sequentially took " << milliseconds.count()
            << " ms." << std::endl;

  if (bs1.GetRemSize() == 0)
  {
    std::cout << "Reading 1M bits sequentially correct" << std::endl;
  }

  bs1.Reset();
  if (bs1.Read(1000001) == nullptr)
  {
    std::cout << "End correct" << std::endl;
  }

  return 0;
}
