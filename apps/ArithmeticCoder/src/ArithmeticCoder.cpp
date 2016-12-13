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

std::list<bool> bits;

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
    int rfreq[3][4];
    int rlen[3][4];

    // 0: high
    rfreq[0][0] = 1200;
    rfreq[0][1] = 100;
    rfreq[0][2] = 50;
    rfreq[0][3] = 1;

    rlen[0][0] = 1;
    rlen[0][1] = 5;
    rlen[0][2] = 5;
    rlen[0][3] = 1989;

    BuildModel(0, 4, rfreq[0], rlen[0]);

    // 1: mid
    rfreq[1][0] = 1200;
    rfreq[1][1] = 100;
    rfreq[1][2] = 50;
    rfreq[1][3] = 1;

    rlen[1][0] = 1;
    rlen[1][1] = 10;
    rlen[1][2] = 10;
    rlen[1][3] = 1979;

    BuildModel(1, 4, rfreq[1], rlen[1]);

    // 2: low
    rfreq[2][0] = 1200;
    rfreq[2][1] = 100;
    rfreq[2][2] = 50;
    rfreq[2][3] = 1;

    rlen[2][0] = 1;
    rlen[2][1] = 20;
    rlen[2][2] = 20;
    rlen[2][3] = 1959;

    BuildModel(2, 4, rfreq[2], rlen[2]);
  }

  ~ArithmeticCodec()
  {
  }

  bool StartModel(int modelid)
  {
    probmodel_.assign(models_[modelid].size() + 1, 0);
    pmf_          = models_[modelid];
    probmodel_[0] = 0;

    // Find out total probability
    for (size_t i = 0; i < pmf_.size(); i++)
    {
      probmodel_[0] += pmf_[i];
    }

    // Progressively substract to obtain model
    for (size_t i = 1; i < pmf_.size(); i++)
    {
      probmodel_[i] = probmodel_[i - 1] - pmf_[i - 1];
    }
    // std::cout << "pmf " << pmf_ << std::endl;
    // std::cout << std::dec << probmodel_.size() << " / " << probmodel_
    //           << std::endl;
    return true;
  }

  bool StartModel(const std::vector<int>& symprob)
  {
    probmodel_.assign(symprob.size() + 1, 0);
    pmf_          = symprob;
    probmodel_[0] = 0;

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

    // std::cout << "pmf " << pmf_ << std::endl;
    // std::cout << std::dec << probmodel_.size() << " / " << probmodel_
    //           << std::endl;
    // std::cout << std::dec << pmf_.size() << " / " << pmf_ << std::endl;
    return true;
  }

  bool Encode(const std::vector<int16_t>& syms, bool adaptive)
  {
    // Reset
    low_          = 0;
    high_         = TopValue_;
    oppositebits_ = 0;
    range_        = 0;

    for (size_t i = 0; i < syms.size(); i++)
    {
      EncodeSymbol(syms[i]);

      if (adaptive) UpdateModel(syms[i]);
    }

    EncodeFlush();

    // std::cout << codedbits_ << std::endl;
    return true;
  }

  bool Decode(const std::vector<bool>& bitvec, size_t symcnt, bool adaptive)
  {
    syms_.clear();
    codedbits_ = bitvec;
    nextbit_   = 0;

    DecodeInit();
    for (size_t isym = 0; isym < symcnt; isym++)
    {
      int16_t sym = DecodeSymbol();
      syms_.push_back(sym);
      if (adaptive) UpdateModel(sym);
    }
    // std::cout << syms_ << std::endl;
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

    // std::cout << "Encode " << symidx << " " << probmodel_[symidx + 1] << " "
    //           << probmodel_[symidx] << " " << low_ << " " << high_ <<
    //           std::endl;
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
    cumprob = static_cast<int>(((static_cast<uint64_t>(value_ - low_) + 1) *
                                    static_cast<uint64_t>(probmodel_[0]) -
                                1) /
                               range_);

    // Search for the interval this probability falls
    for (size_t i = 0; i < probmodel_.size() - 1; i++)
    {
      if (probmodel_[i + 1] <= cumprob && cumprob < probmodel_[i])
        symidx = static_cast<int16_t>(i);
    }

    // std::cout << "sym " << symidx << " " << probmodel_[symidx + 1] << " "
    //           << cumprob << " " << probmodel_[symidx] << std::endl;

    if (symidx == -1)
    {
      std::cout << "[Error]: Can't find corresponding symbol for " << cumprob
                << std::endl;
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
    pmf_[symidx]++;

    // Adjust high for this symidx and all later intervals.
    for (int16_t i = 0; i <= symidx; i++)
    {
      probmodel_[i]++;
    }

    return true;
  }

  bool BuildModel(int modelid, int count, int rfreq[], int rlen[])
  {
    if (modelid > 2)
    {
      std::cout << "[ERROR]: Maximum allowed model id is 2" << std::endl;
      return false;
    }

    for (int r = 0; r < count; r++)
    {
      for (int i = 0; i < rlen[r]; i++)
      {
        models_[modelid].push_back(rfreq[r]);
      }
    }

    return true;
  }

  std::vector<int> probmodel_;
  std::vector<int> pmf_;
  std::vector<bool> codedbits_;
  std::vector<int16_t> syms_;
  bool isadaptive_      = false;
  bool usedefaultmodel_ = true;

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

  std::vector<int> models_[3];  // 0: high, 1: mid, 2: low
};

/**
 * @brief      { function_description }
 *
 * @return     { description_of_the_return_value }
 */
int main()
{
  ArithmeticCodec coder;
  std::vector<int> symprob;
  std::vector<int16_t> syms;
  std::vector<int16_t> decodedsyms;

  symprob.push_back(1444);
  symprob.push_back(2);

  // coder.StartModel(symprob);

  for (size_t ss = 0; ss < 1446; ss++)
  {
    syms.push_back(0);
  }

  syms[9]    = 1;
  syms[1431] = 1;

  coder.StartModel(symprob);
  coder.Encode(syms, false);

  // std::cout << coder.GetCodedBits().size() << " / " << std::endl;
  // for (size_t ss = 0; ss < 100; ss++)
  // {
  //   std::cout << coder.GetCodedBits()[ss] << " ";
  // }
  // std::cout << std::endl;

  // coder.StartModel(0);

  std::cout << coder.probmodel_ << std::endl;
  for (size_t ss = 0; ss < coder.GetCodedBits().size(); ss++)
  {
    std::cout << coder.GetCodedBits()[ss] << " ";
  }

  std::cout << std::endl;

  coder.StartModel(symprob);
  coder.Decode(coder.GetCodedBits(), syms.size(), false);
  decodedsyms = coder.GetDecodedSymbols();
  if (decodedsyms.size() != syms.size())
  {
    std::cout << "Size miss match" << std::endl;
    return -1;
  }

  for (size_t ss = 0; ss < decodedsyms.size(); ss++)
  {
    if (decodedsyms[ss] != syms[ss])
    {
      std::cout << "Miss match happened at " << ss << " " << decodedsyms[ss]
                << " " << syms[ss] << std::endl;
      return -1;
    }
  }
  return 0;
}
