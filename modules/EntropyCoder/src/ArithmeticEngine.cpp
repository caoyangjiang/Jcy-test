// Copyright @ 2016 Caoyang Jiang

#include "Jcy/EntropyCoder/ArithmeticEngine.h"

HVR_WINDOWS_DISABLE_ALL_WARNING
#include <cstring>
#include <iostream>
#include <memory>
HVR_WINDOWS_ENABLE_ALL_WARNING

namespace Jcy
{
template class ArithmeticEngine<uint8_t>;
template class ArithmeticEngine<uint16_t>;
template class ArithmeticEngine<uint32_t>;

template <typename T>
ArithmeticEngine<T>::ArithmeticEngine(enum MODE mode)
{
  mode_ = mode;

  if (mode_ == DECODE)
  {
    bs_ = std::make_unique<BitStream>(BitStream::MODE::RD,
                                      BitStream::ENDIAN::LITTLE);
  }
  else if (mode_ == ENCODE)
  {
    bs_ = std::make_unique<BitStream>(BitStream::MODE::WR,
                                      BitStream::ENDIAN::LITTLE);
  }

  backupmodel_.reserve(1000000);
  runtimemodel_.reserve(1000000);
}

template <typename T>
ArithmeticEngine<T>::~ArithmeticEngine()
{
}

template <typename T>
void ArithmeticEngine<T>::ResetBitBuffer()
{
  bs_->Reset();
}

template <typename T>
void ArithmeticEngine<T>::ResetSymBuffer()
{
  decodedsymbols_.clear();
}

template <typename T>
void ArithmeticEngine<T>::ResetEngineState()
{
  low_          = 0;
  high_         = kTopValue_;
  oppositebits_ = 0;
  range_        = 0;
  nextbit_      = 0;
  value_        = 0;
}

template <typename T>
void ArithmeticEngine<T>::ResetProbabilityModel()
{
  std::memcpy(reinterpret_cast<void*>(runtimemodel_.data()),
              reinterpret_cast<void*>(backupmodel_.data()),
              backupmodel_.size() * sizeof(uint32_t));
}

template <typename T>
void ArithmeticEngine<T>::LoadProbabilityModel(
    const std::vector<uint64_t>& frequency)
{
  backupmodel_.assign(frequency.size() + 1, 0);

  // Find out total probability
  for (size_t i = 0; i < frequency.size(); i++)
  {
    backupmodel_[0] += frequency[i];
  }

  // Progressively substract to obtain model
  for (size_t i = 1; i < frequency.size(); i++)
  {
    backupmodel_[i] = backupmodel_[i - 1] - frequency[i - 1];
  }

  runtimemodel_ = backupmodel_;
}

template <typename T>
void ArithmeticEngine<T>::LoadProbabilityModel(enum PROBABILITYMODEL, uint64_t)
{
}

template <typename T>
void ArithmeticEngine<T>::Encode(const T* symbols, size_t totalsymbol)
{
  size_t symcnt = 0;
  while (symcnt != totalsymbol)
  {
    EncodeASymbol(*(symbols + symcnt));
    symcnt++;
  }

  EncodeFlush();
}

template <typename T>
void ArithmeticEngine<T>::Decode(const uint8_t* bits,
                                 size_t totalbits,
                                 size_t totalsymbol)
{
  bs_->Load(bits, totalbits);

  // Get decoder going
  for (uint64_t i = 1; i <= kCodeValueBits_; i++)
  {
    if (bs_->GetRemSize() > 0)
    {
      uint8_t bit = bs_->ReadBit();
      value_      = (2 * value_) + static_cast<uint64_t>(bit);
    }
    else
    {
      // Else append 0
      value_ = 2 * value_;
    }
  }

  size_t symcnt = 0;
  while (symcnt != totalsymbol)
  {
	  std::cout << DecodeASymbol();
    symcnt++;
  }

  std::cout << std::endl;
}

template <typename T>
const char* ArithmeticEngine<T>::GetCodedBits() const
{
  return reinterpret_cast<const char*>(bs_->GetBitBuffer());
}

template <typename T>
size_t ArithmeticEngine<T>::GetCodedBitsCount() const 
{
  return bs_->GetWrittenSize();
}

template <typename T>
const T* ArithmeticEngine<T>::GetDecodedSymbols() const
{
  return decodedsymbols_.data();
}

template <typename T>
void ArithmeticEngine<T>::EncodeASymbol(T symbol)
{
  range_ = high_ - low_ + 1;
  high_  = low_ + (range_ * runtimemodel_[static_cast<size_t>(symbol)]) / runtimemodel_[0] - 1;
  low_   = low_ + (range_ * runtimemodel_[static_cast<size_t>(symbol) + 1]) / runtimemodel_[0];

  while (true)
  {
    if (high_ < kHalfValue_)
    {
      bs_->WriteZero();
      while (oppositebits_ > 0)
      {
        bs_->WriteOne();
        oppositebits_--;
      }
    }
    else if (low_ >= kHalfValue_)
    {
      bs_->WriteOne();
      while (oppositebits_ > 0)
      {
        bs_->WriteZero();
        oppositebits_--;
      }
      low_ -= kHalfValue_;
      high_ -= kHalfValue_;
    }
    else if (low_ >= kQtrValue_ && high_ < k3QtrValue_)
    {
      oppositebits_++;
      low_ -= kQtrValue_;
      high_ -= kQtrValue_;
    }
    else
    {
      break;
    }

    low_  = 2 * low_;
    high_ = 2 * high_ + 1;
  }
}

template <typename T>
void ArithmeticEngine<T>::EncodeFlush()
{
  oppositebits_++;

  if (low_ < kQtrValue_)
  {
    bs_->WriteZero();
    while (oppositebits_ > 0)
    {
      bs_->WriteOne();
      oppositebits_--;
    }
  }
  else
  {
    bs_->WriteOne();
    while (oppositebits_ > 0)
    {
      bs_->WriteZero();
      oppositebits_--;
    }
  }
}

template <typename T>
T ArithmeticEngine<T>::DecodeASymbol()
{
  uint64_t cummu = 0;
  //  uint64_t l     = 0;
  //  uint64_t r     = static_cast<uint64_t>(runtimemodel_.size()) - 1;
  //  uint64_t m     = 0;
  uint64_t idx = 0;

  range_ = high_ - low_ + 1;
  cummu  = ((value_ - low_ + 1) * runtimemodel_[0] - 1) / range_;

  // Linear search for the interval this probability falls
  for (size_t i = 0; i < runtimemodel_.size() - 1; i++)
  {
    if (runtimemodel_[i + 1] <= cummu && cummu < runtimemodel_[i])
      idx = static_cast<T>(i);
  }

  // Binary Search for the interval this probability falls (6x speedup)
  // while (runtimemodel_[l] > runtimemodel_[r])
  // {
  //   std::cout << "stuck 1" << std::endl;
  //   m = (l + r) / 2;
  //   if (cummu > runtimemodel_[m])
  //   {
  //     r = m - 1;
  //   }
  //   else if (cummu < runtimemodel_[m])
  //   {
  //     l = m + 1;
  //   }
  //   else
  //   {
  //     break;
  //   }
  // }

  // if (runtimemodel_[m] == cummu)
  // {
  //   while (runtimemodel_[m - 1] == runtimemodel_[m]) m--;
  //   idx = m - 1;
  // }
  // else if (cummu > runtimemodel_[l] && (l == r))
  // {
  //   idx = r - 1;
  // }
  // else if (cummu < runtimemodel_[l] && (l == r))
  // {
  //   idx = r;
  // }
  // else if (cummu == runtimemodel_[l] && (l == r))
  // {
  //   idx = r - 1;
  // }
  // else if (cummu > runtimemodel_[l] && (runtimemodel_[l] < runtimemodel_[r]))
  // {
  //   idx = r;
  // }
  // else
  // {
  //   std::cout << l << " " << r << " " << cummu << runtimemodel_[l] <<
  //   std::endl;
  //   std::cout << "some case not considered" << std::endl;
  // }

  high_ = low_ + (range_ * runtimemodel_[static_cast<size_t>(idx)]) / runtimemodel_[0] - 1;
  low_  = low_ + (range_ * runtimemodel_[static_cast<size_t>(idx) + 1]) / runtimemodel_[0];

  while (true)
  {
    if (high_ < kHalfValue_)
    {
    }
    else if (low_ >= kHalfValue_)
    {
      value_ -= kHalfValue_;
      low_ -= kHalfValue_;
      high_ -= kHalfValue_;
    }
    else if (low_ >= kQtrValue_ && high_ < k3QtrValue_)
    {
      value_ -= kQtrValue_;
      low_ -= kQtrValue_;
      high_ -= kQtrValue_;
    }
    else
    {
      break;
    }

    low_  = 2 * low_;
    high_ = 2 * high_ + 1;

    if (bs_->GetRemSize() > 0)
    {
      value_ = (2 * value_) + static_cast<uint64_t>(bs_->ReadBit());
    }
    else
    {
      value_ = 2 * value_;
    }
  }

  return static_cast<T>(idx);
}

}  // namespace Jcy
