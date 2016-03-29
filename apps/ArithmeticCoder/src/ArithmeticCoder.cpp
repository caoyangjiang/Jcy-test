// Copyright @ 2016 Caoyang Jiang

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>

#define c 32
#define Top (std::pow(2, c) - 1)
#define Qtr (Top / 4 + 1)
#define Half (2 * Qtr)
#define Qtr3 (3 * Qtr)

uint64_t low           = 0;
uint64_t high          = Top;
uint64_t opposite_bits = 0;
uint64_t range         = 0;
int index              = 0;
uint64_t value         = 0;
uint64_t bit           = 0;
uint64_t cum           = 0;
uint64_t model[]       = {100, 50, 0};

void encode(int index, uint64_t* cum_freq)
{
  range = high - low + 1;
  high  = low + (range * cum_freq[index - 1]) / cum_freq[0] - 1;
  low   = low + (range * cum_freq[index]) / cum_freq[0];
  while (true)
  {
    if (high < Half)
    {
      std::cout << "0" << std::endl;
      while (opposite_bits > 0)
      {
        std::cout << "1" << std::endl;
        opposite_bits--;
      }
    }
    else if (low >= Half)
    {
      std::cout << "1" << std::endl;
      while (opposite_bits > 0)
      {
        std::cout << "0" << std::endl;
        opposite_bits--;
      }
      low -= Half;
      high -= Half;
    }
    else if (low >= Qtr && high < Qtr3)
    {
      opposite_bits++;
      low -= Qtr;
      high -= Qtr;
    }
    else
    {
      break;
    }

    low <<= 1;
    high = 2 * high + 1;
  }
}

void encoder_flush()
{
  opposite_bits++;
  if (low < Qtr)
  {
    std::cout << "0" << std::endl;
    while (opposite_bits > 0)
    {
      std::cout << "1" << std::endl;
      opposite_bits--;
    }
  }
  else
  {
    std::cout << "1" << std::endl;
    while (opposite_bits > 0)
    {
      std::cout << "0" << std::endl;
      opposite_bits--;
    }
  }
  low  = 0;
  high = Top;
}

int decode(uint64_t* cum_freq)
{
  range = high - low + 1;
  cum   = ((value - low + 1) * cum_freq[0] - 1) / range;

  index = 0;
  for (int i = 1; i < 3; i++)
  {
    // std::cout << cum_freq[i] << " " << cum << " " << cum_freq[i - 1]
    //           << std::endl;
    if (cum_freq[i] <= cum && cum < cum_freq[i - 1]) index = i;
  }

  std::cout << index << std::endl;
  if (index == 0)
  {
    std::cout << "can't find" << std::endl;
    return 0;
  }

  high = low + (range * cum_freq[index - 1]) / cum_freq[0] - 1;
  low  = low + (range * cum_freq[index]) / cum_freq[0];

  while (true)
  {
    if (high < Half)
    {
      value -= Half;
      low -= Half;
      high -= Half;
    }
    else if (low >= Qtr && high < Qtr3)
    {
      value -= Qtr;
      low -= Qtr;
      high -= Qtr;
    }
    else
    {
      break;
    }
    low <<= 1;
    high  = 2 * high + 1;
    value = 2 * value + 1;
  }

  return index;
}

void decoder_reset()
{
  value = 0;
  low   = 0;
  high  = Top;
  for (int i = 1; i <= c; i++)
  {
  }
}
int main()
{
  std::cout << c << " " << std::hex << static_cast<uint32_t>(Top) << " "
            << static_cast<uint32_t>(Qtr) << " " << static_cast<uint32_t>(Half)
            << " " << static_cast<uint32_t>(Qtr3) << std::endl;
  int sym[] = {1, 1, 1, 2};

  std::cout << "Encode " << std::endl;
  low           = 0;
  high          = Top;
  opposite_bits = 0;
  range         = 0;
  for (size_t i = 0; i < 3; i++)
    encode(sym[i], reinterpret_cast<uint64_t*>(model));
  encoder_flush();

  low           = 0;
  high          = Top;
  opposite_bits = 0;
  range         = 0;
  value         = 0xE0000000;

  // std::cout << std::hex << value << std::endl;

  std::cout << "Decode " << std::endl;
  // decode(reinterpret_cast<uint64_t*>(model));

  while (decode(reinterpret_cast<uint64_t*>(model)) != 2)
  {
  }

  return 0;
}
