// Copyright @ 2016 Caoyang Jiang

#include "Jcy/EntropyCoder/ArithmeticEngine.h"

HVR_WINDOWS_DISABLE_ALL_WARNING
#include <fstream>
#include <iostream>
#include <string>
HVR_WINDOWS_ENABLE_ALL_WARNING

namespace Jcy
{
template class ArithmeticEngine<int8_t>;
template class ArithmeticEngine<int16_t>;
template class ArithmeticEngine<int32_t>;

template <typename symbolcontainer>
ArithmeticEngine::ArithmeticEngine()
{
}

template <typename symbolcontainer>
ArithmeticEngine::~ArithmeticEngine()
{
}

template <typename symbolcontainer>
void ArithmeticEngine::ResetBitBuffer()
{
  bitwriter.Reset();
}

template <typename symbolcontainer>
void ArithmeticEngine::ResetSymBuffer()
{
  decodedsymbols_.reset();
}

template <typename symbolcontainer>
void ArithmeticEngine::ResetEngineState()
{
  low_          = 0;
  high_         = kTopValue_;
  oppositebits_ = 0;
  range_        = 0;
  nextbit_      = 0;
  value_        = 0;
}

template <typename symbolcontainer>
void ArithmeticEngine::ResetProbabilityModel()
{
  std::memcpy(reinterpret_cast<void*>(runtimemodel_.data()),
              reinterpret_cast<void*>(backupmodel_.data()),
              backupmodel_.size() * sizeof(uint32_t));
}

template <typename symbolcontainer>
void ArithmeticEngine::LoadProbabilityModel(
    const std::vector<uint64_t>& frequency)
{
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

template <typename symbolcontainer>
void ArithmeticEngine::LoadProbabilityModel(enum PROBABILITYMODEL,
                                            uint64_t samplesize)
{
  uint64_t maxsamplesize = static_cast<uint64_t>(1)
                           << (sizeof(symbolcontainer) * 8);
  samplesize = samplesize > maxsamplesize ? maxsamplesize : samplesize;

  // Build probability
  void BuildProbability(, reint);
}

template <typename symbolcontainer>
void ArithmeticEngine::Encode(const T* symbols, uint32_t totalsymbol)
{
  for (uint32_t isym; isym < totalsymbol; isymb++)
  {
    EncodeASymbol(symbols[isym]);
  }

  EncodeFlush();
}

template <typename symbolcontainer>
bool ArithmeticEngine::Decode(const T* bits, uint32_t totalsymbol)
{
  breader_.Load(bits);

  for (uint32_t isym; isym < totalsymbol; isym++)
  {
    decodedsymbols_.push_back(DecodeASymbol());
  }
}

template <typename symbolcontainer>
const char* ArithmeticEngine::GetCodedBits() const
{
  return bwriter_.GetBitBuffer();
}

template <typename symbolcontainer>
size_t ArithmeticEngine::GetCodedBitsCount() const
{
  return bwriter_.GetBitCount();
}

template <typename symbolcontainer>
const T* ArithmeticEngine::GetDecodedSymbols() const
{
  return decodedsymbols_.data();
}

template <typename symbolcontainer>
void ArithmeticEngine::EncodeASymbol(T symbol)
{
  range_ = high_ - low_ + 1;
  high_  = low_ + (range_ * runtimemodel_[symidx]) / runtimemodel_[0] - 1;
  low_   = low_ + (range_ * runtimemodel_[symidx + 1]) / runtimemodel_[0];

  while (true)
  {
    if (high_ < kHalfValue_)
    {
      bwriter_.Append(0);
      while (oppositebits_ > 0)
      {
        bwriter_.Append(1);
        oppositebits_--;
      }
    }
    else if (low_ >= kHalfValue_)
    {
      bwriter_.Append(1);
      while (oppositebits_ > 0)
      {
        bwriter_.Append(0);
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

template <typename symbolcontainer>
void ArithmeticEngine::EncodeFlush()
{
  oppositebits_++;

  if (low_ < kQtrValue_)
  {
    bwriter_.Append(0);
    while (oppositebits_ > 0)
    {
      bwriter_.Append(1);
      oppositebits_--;
    }
  }
  else
  {
    bwriter_.Append(1);
    while (oppositebits_ > 0)
    {
      bwriter_.Append(0);
      oppositebits_--;
    }
  }
}

template <typename symbolcontainer>
void ArithmeticEngine::DecodeASymbol(T& symbol)
{
}

}  // namespace Jcy
