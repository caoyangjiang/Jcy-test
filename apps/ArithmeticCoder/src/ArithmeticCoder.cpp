// Copyright @ 2016 Caoyang Jiang

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <memory>
#include <vector>

#define c 32
#define Top (std::pow(2, c) - 1)
#define Qtr (Top / 4 + 1)
#define Half (2 * Qtr)
#define Qtr3 (3 * Qtr)

// uint64_t low           = 0;
// uint64_t high          = Top;
// uint64_t opposite_bits = 0;
// uint64_t range         = 0;
// int index              = 0;
// uint64_t value         = 0;
// uint64_t bit           = 0;
// uint64_t cum     = 0;
// uint64_t model[] = {100, 75, 50, 25, 0};
std::list<bool> bits;
//#define SYMBOLCOUNT 7

std::ostream& operator<<(std::ostream& os, const std::list<bool>& list)
{
  for (std::list<bool>::const_iterator it = list.begin(); it != list.end();
       it++)
  {
    os << *it << " ";
  }
  return os;
}

template <class T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& vec)
{
  for (size_t i = 0; i < vec.size(); i++)
  {
    os << vec[i] << " ";
  }
  return os;
}

class ArithmeticCodec
{
 public:
  ArithmeticCodec()
  {
  }

  ~ArithmeticCodec()
  {
  }

  bool Config(int16_t symmin,
              int16_t symmax,
              bool isadaptive,
              bool usedefaultmodel)
  {
    min_             = 0;
    max_             = symmax;
    isadaptive_      = isadaptive;
    usedefaultmodel_ = usedefaultmodel;
    probmodel_.assign(symmax - symmin + 2, 0);  // probmodel[0] has the total.
    pmf_.assign(symmax - symmin + 1, 0);

    for (int16_t symidx = symmin; symidx <= symmax; symidx++)
    {
      UpdateModel(symidx);
    }

    std::cout << CodeValueBits_ << " " << std::hex << TopValue_ << " "
              << QtrValue_ << " " << HalfValue_ << " " << Qtr3Value_
              << std::endl;

    std::cout << std::dec << probmodel_.size() << " / " << probmodel_
              << std::endl;
    std::cout << std::dec << pmf_.size() << " / " << pmf_ << std::endl;

    return true;
  }

  bool StartModel(const std::vector<int>& symprob)
  {
    if (static_cast<int16_t>(symprob.size() - 1) > max_)
    {
      std::cout << "[ERROR]: exceed max" << std::endl;
      return false;
    }

    probmodel_.assign(symprob.size() + 1, 0);
    pmf_ = symprob;

    // Find out total probability
    for (size_t i = 0; i < symprob.size(); i++)
    {
      probmodel_[0] += pmf_[i];
    }

    // Progressively substract to obtain model
    for (size_t i = 1; i < symprob.size(); i++)
    {
      probmodel_[i] = probmodel_[i - 1] - pmf_[i - 1];
    }

    std::cout << std::dec << probmodel_.size() << " / " << probmodel_
              << std::endl;
    std::cout << std::dec << pmf_.size() << " / " << pmf_ << std::endl;
    return true;
  }

  bool Encode(const std::vector<int16_t>& syms)
  {
    // Reset
    low_          = 0;
    high_         = TopValue_;
    oppositebits_ = 0;
    range_        = 0;

    for (size_t i = 0; i < syms.size(); i++)
    {
      EncodeSymbol(syms[i]);
    }

    EncodeFlush();

    std::cout << codedbits_ << std::endl;
    return true;
  }

  bool Decode(const std::vector<bool>& bitvec, size_t symcnt)
  {
    syms_.clear();
    codedbits_ = bitvec;
    nextbit_   = 0;

    DecodeInit();
    for (size_t isym = 0; isym < symcnt; isym++)
    {
      syms_.push_back(DecodeSymbol());
    }
    std::cout << syms_ << std::endl;
    return true;
  }

  const std::vector<bool>& GetCodedBits() const
  {
    return codedbits_;
  }

  const std::vector<int16_t>& GetDecodedSymbols() const
  {
    return syms_;
  }

 private:
  void EncodeSymbol(int16_t symidx)
  {
    range_ = static_cast<uint64_t>(high_ - low_) + 1;
    high_  = low_ +
            (range_ * static_cast<uint64_t>(probmodel_[symidx])) /
                static_cast<uint64_t>(probmodel_[0]) -
            1;
    low_ = low_ +
           (range_ * static_cast<uint64_t>(probmodel_[symidx + 1])) /
               static_cast<uint64_t>(probmodel_[0]);

    std::cout << "Encode " << symidx << " " << probmodel_[symidx + 1] << " "
              << probmodel_[symidx] << " " << low_ << " " << high_ << std::endl;
    while (true)
    {
      if (high_ < HalfValue_)
      {
        codedbits_.push_back(0);
        while (oppositebits_ > 0)
        {
          codedbits_.push_back(1);
          oppositebits_--;
        }
      }
      else if (low_ >= HalfValue_)
      {
        codedbits_.push_back(1);

        while (oppositebits_ > 0)
        {
          codedbits_.push_back(0);
          oppositebits_--;
        }
        low_ -= HalfValue_;
        high_ -= HalfValue_;
      }
      else if (low_ >= QtrValue_ && high_ < Qtr3Value_)
      {
        oppositebits_++;
        low_ -= QtrValue_;
        high_ -= QtrValue_;
      }
      else
      {
        break;
      }

      low_  = 2 * low_;
      high_ = 2 * high_ + 1;
    }
  }

  int16_t DecodeSymbol()
  {
    int16_t symidx = -1;
    int cumprob    = 0;

    range_  = static_cast<uint64_t>(high_ - low_) + 1;
    cumprob = static_cast<int>((static_cast<uint64_t>(value_ - low_ + 1) *
                                    static_cast<uint64_t>(probmodel_[0]) -
                                1) /
                               range_);

    // Search for the interval this probability falls
    for (size_t i = 0; i < probmodel_.size() - 1; i++)
    {
      if (probmodel_[i + 1] <= cumprob && cumprob < probmodel_[i])
        symidx = static_cast<int16_t>(i);
    }

    std::cout << "sym " << symidx << " " << probmodel_[symidx + 1] << " "
              << cumprob << " " << probmodel_[symidx] << std::endl;

    if (symidx == -1)
    {
      std::cout << "[Error]: Can't find corresponding symbol" << std::endl;
      return 0;
    }

    high_ = static_cast<uint32_t>(
        low_ +
        (range_ * static_cast<uint64_t>(probmodel_[symidx])) /
            static_cast<uint64_t>(probmodel_[0]) -
        1);
    low_ = static_cast<uint32_t>(
        low_ +
        (range_ * static_cast<uint64_t>(probmodel_[symidx + 1])) /
            static_cast<uint64_t>(probmodel_[0]));

    while (true)
    {
      if (high_ < HalfValue_)
      {
      }
      else if (low_ >= HalfValue_)
      {
        value_ -= HalfValue_;
        low_ -= HalfValue_;
        high_ -= HalfValue_;
      }
      else if (low_ >= QtrValue_ && high_ < Qtr3Value_)
      {
        value_ -= QtrValue_;
        low_ -= QtrValue_;
        high_ -= QtrValue_;
      }
      else
      {
        break;
      }

      low_  = 2 * low_;
      high_ = 2 * high_ + 1;

      if (nextbit_ < codedbits_.size())
      {
        bool bit = codedbits_[nextbit_++];
        value_   = 2 * value_ + uint64_t(bit);
      }
      else
      {
        value_ = 2 * value_;
      }
    }

    return symidx;
  }

  void DecodeInit()
  {
    value_ = 0;
    low_   = 0;
    high_  = TopValue_;

    for (uint32_t i = 1; i <= CodeValueBits_; i++)
    {
      if (nextbit_ < codedbits_.size())
      {
        bool bit = codedbits_[nextbit_++];
        value_   = 2 * value_ + uint64_t(bit);
      }
      else
      {
        // Else append 0
        value_ = 2 * value_ + 1;
      }
    }
  }

  void EncodeFlush()
  {
    oppositebits_++;
    if (low_ < QtrValue_)
    {
      codedbits_.push_back(0);
      while (oppositebits_ > 0)
      {
        codedbits_.push_back(1);
        oppositebits_--;
      }
    }
    else
    {
      codedbits_.push_back(1);
      while (oppositebits_ > 0)
      {
        codedbits_.push_back(0);
        oppositebits_--;
      }
    }
    low_  = 0;
    high_ = TopValue_;
  }

  bool UpdateModel(int16_t symidx)
  {
    if (symidx > max_)
    {
      std::cout << "[ERROR]: Symbol out of bound." << std::endl;
      return false;
    }

    pmf_[symidx]++;

    // Adjust high for this symidx and all later intervals.
    for (int16_t i = 0; i <= symidx; i++)
    {
      probmodel_[i]++;
    }

    return true;
  }

 private:
  std::vector<int> probmodel_;
  std::vector<int> pmf_;
  std::vector<bool> codedbits_;
  std::vector<int16_t> syms_;
  bool isadaptive_      = false;
  bool usedefaultmodel_ = true;
  int16_t min_          = 0;
  int16_t max_          = 65535;

  const uint32_t CodeValueBits_ = 32;
  const uint32_t TopValue_  = (static_cast<uint64_t>(1) << CodeValueBits_) - 1;
  const uint32_t QtrValue_  = TopValue_ / 4 + 1;
  const uint32_t HalfValue_ = 2 * QtrValue_;
  const uint32_t Qtr3Value_ = 3 * QtrValue_;

  uint32_t low_          = 0;
  uint32_t high_         = TopValue_;
  uint32_t oppositebits_ = 0;
  uint64_t range_        = 0;
  uint32_t value_        = 0;
  size_t nextbit_        = 0;
};

// void encode(int index, uint64_t* cum_freq)
// {
//   std::cout << "SymHi " << cum_freq[index - 1] << std::endl;
//   std::cout << "SymLow " << cum_freq[index] << std::endl;
//   range = high - low + 1;
//   high  = low + (range * cum_freq[index - 1]) / cum_freq[0] - 1;
//   low   = low + (range * cum_freq[index]) / cum_freq[0];
//   while (true)
//   {
//     if (high < Half)
//     {
//       bits.push_back(0);
//       while (opposite_bits > 0)
//       {
//         bits.push_back(1);
//         opposite_bits--;
//       }
//     }
//     else if (low >= Half)
//     {
//       bits.push_back(1);

//       while (opposite_bits > 0)
//       {
//         bits.push_back(0);
//         opposite_bits--;
//       }
//       low -= Half;
//       high -= Half;
//     }
//     else if (low >= Qtr && high < Qtr3)
//     {
//       opposite_bits++;
//       low -= Qtr;
//       high -= Qtr;
//     }
//     else
//     {
//       break;
//     }

//     low <<= 1;
//     high = 2 * high + 1;
//   }
// }

// void encoder_flush()
// {
//   opposite_bits++;
//   if (low < Qtr)
//   {
//     bits.push_back(0);
//     while (opposite_bits > 0)
//     {
//       bits.push_back(1);
//       opposite_bits--;
//     }
//   }
//   else
//   {
//     bits.push_back(1);
//     while (opposite_bits > 0)
//     {
//       bits.push_back(0);
//       opposite_bits--;
//     }
//   }
//   low  = 0;
//   high = Top;
// }

// int decode(uint64_t* cum_freq)
// {
//   range = high - low + 1;
//   cum   = ((value - low + 1) * cum_freq[0] - 1) / range;

//   index = 0;
//   for (int i = 1; i < SYMBOLCOUNT + 1; i++)
//   {
//     // std::cout << cum_freq[i] << " " << cum << " " << cum_freq[i - 1]
//     //           << std::endl;
//     if (cum_freq[i] <= cum && cum < cum_freq[i - 1]) index = i;
//   }

//   std::cout << index << std::endl;
//   if (index == 0)
//   {
//     std::cout << "can't find" << std::endl;
//     return 0;
//   }

//   high = low + (range * cum_freq[index - 1]) / cum_freq[0] - 1;
//   low  = low + (range * cum_freq[index]) / cum_freq[0];

//   while (true)
//   {
//     if (high < Half)
//     {
//       value -= Half;
//       low -= Half;
//       high -= Half;
//     }
//     else if (low >= Qtr && high < Qtr3)
//     {
//       value -= Qtr;
//       low -= Qtr;
//       high -= Qtr;
//     }
//     else
//     {
//       break;
//     }
//     low <<= 1;
//     high = 2 * high + 1;

//     if (bits.size() != 0)
//     {
//       bool bit = bits.front();
//       bits.pop_front();
//       value = 2 * value + uint64_t(bit);
//     }
//     else
//     {
//       value = 2 * value;
//     }
//   }

//   return index;
// }

// void decoder_reset()
// {
//   value = 0;
//   low   = 0;
//   high  = Top;
//   for (int i = 1; i <= c; i++)
//   {
//     if (bits.size() != 0)
//     {
//       bool bit = bits.front();
//       bits.pop_front();
//       value = 2 * value + uint64_t(bit);
//     }
//     else
//     {
//       // Else append 0
//       value = 2 * value;
//     }
//   }
//   std::cout << value << std::endl;
// }
int main()
{
  ArithmeticCodec coder;
  std::vector<int> symprob;
  std::vector<int16_t> syms;

  // for (size_t i = 0; i < 4; i++)
  // {
  //   symprob.push_back(1);
  // }

  // coder.Config(0, 3, true, true);
  // coder.StartModel(symprob);

  // syms.push_back(0);
  // syms.push_back(0);
  // syms.push_back(1);
  // syms.push_back(3);
  // syms.push_back(0);
  // syms.push_back(1);
  // syms.push_back(2);

  // coder.Encode(syms);

  symprob.push_back(381);
  symprob.push_back(430);
  symprob.push_back(456);
  symprob.push_back(184);

  coder.Config(0, 3, true, true);
  coder.StartModel(symprob);

  syms.push_back(0);
  syms.push_back(1);
  syms.push_back(2);
  syms.push_back(3);
  syms.push_back(2);
  syms.push_back(2);
  syms.push_back(0);
  syms.push_back(3);
  syms.push_back(2);
  syms.push_back(3);
  syms.push_back(2);
  syms.push_back(3);
  syms.push_back(0);
  syms.push_back(2);
  syms.push_back(1);
  syms.push_back(0);
  syms.push_back(2);

  // for (size_t i = 0; i < 1000; i++) syms.push_back(0);

  coder.Encode(syms);
  coder.Decode(coder.GetCodedBits(), syms.size());
  return 0;
}
