// Copyright @ 2016 Caoyang Jiang

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
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
// uint64_t bit           = 0;
uint64_t cum     = 0;
uint64_t model[] = {100, 75, 50, 25, 0};
#define SYMBOLCOUNT 4
std::list<bool> bits;

std::ostream& operator<<(std::ostream& os, const std::list<bool>& list)
{
  for (std::list<bool>::const_iterator it = list.begin(); it != list.end();
       it++)
  {
    os << *it;
  }
  return os;
}

void encode(int index, uint64_t* cum_freq)
{
  std::cout << "SymHi " << cum_freq[index - 1] << std::endl;
  std::cout << "SymLow " << cum_freq[index] << std::endl;
  range = high - low + 1;
  high  = low + (range * cum_freq[index - 1]) / cum_freq[0] - 1;
  low   = low + (range * cum_freq[index]) / cum_freq[0];
  while (true)
  {
    if (high < Half)
    {
      bits.push_back(0);
      while (opposite_bits > 0)
      {
        bits.push_back(1);
        opposite_bits--;
      }
    }
    else if (low >= Half)
    {
      bits.push_back(1);

      while (opposite_bits > 0)
      {
        bits.push_back(0);
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
    bits.push_back(0);
    while (opposite_bits > 0)
    {
      bits.push_back(1);
      opposite_bits--;
    }
  }
  else
  {
    bits.push_back(1);
    while (opposite_bits > 0)
    {
      bits.push_back(0);
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
  for (int i = 1; i < SYMBOLCOUNT + 1; i++)
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
    high = 2 * high + 1;

    if (bits.size() != 0)
    {
      bool bit = bits.front();
      bits.pop_front();
      value = 2 * value + uint64_t(bit);
    }
    else
    {
      value = 2 * value;
    }
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
    if (bits.size() != 0)
    {
      bool bit = bits.front();
      bits.pop_front();
      value = 2 * value + uint64_t(bit);
    }
    else
    {
      // Else append 0
      value = 2 * value;
    }
  }
  std::cout << value << std::endl;
}
int main()
{
  std::cout << c << " " << std::hex << static_cast<uint32_t>(Top) << " "
            << static_cast<uint32_t>(Qtr) << " " << static_cast<uint32_t>(Half)
            << " " << static_cast<uint32_t>(Qtr3) << std::endl;
  int sym[] = {1, 1, 2, 4, 1, 2, 3};

  std::cout << "Encode " << std::endl;
  low           = 0;
  high          = Top;
  opposite_bits = 0;
  range         = 0;

  for (size_t i = 0; i < 7; i++)
  {
    std::cout << "Encode symbol " << sym[i] << std::endl;
    encode(sym[i], reinterpret_cast<uint64_t*>(model));
  }
  encoder_flush();

  std::cout << "[Encoded bits]: " << bits << std::endl;

  std::cout << "Decode " << std::endl;
  low           = 0;
  high          = Top;
  opposite_bits = 0;
  range         = 0;
  value         = 0;

  decoder_reset();
  decode(reinterpret_cast<uint64_t*>(model));
  decode(reinterpret_cast<uint64_t*>(model));
  decode(reinterpret_cast<uint64_t*>(model));
  decode(reinterpret_cast<uint64_t*>(model));
  decode(reinterpret_cast<uint64_t*>(model));
  decode(reinterpret_cast<uint64_t*>(model));
  decode(reinterpret_cast<uint64_t*>(model));
  // while (decode(reinterpret_cast<uint64_t*>(model)) != 3)
  // {
  // }

  return 0;
}
