// Copyright 2016 Caoyang Jiang

#include "Jcy/MiscTools/BitStream.h"

HVR_WINDOWS_DISABLE_ALL_WARNING
#include <iostream>
#include <string>
#include <vector>
HVR_WINDOWS_ENABLE_ALL_WARNING

namespace Jcy
{
BitStream::BitStream(enum MODE mode, enum ENDIAN endian)
{
  mode_   = mode;
  endian_ = endian;

  if (mode_ == MODE::WR)
  {
    wrbuf_.reserve(1000000);  // 8M bits
    wrpos_ = 0;
  }
  else if (mode_ == MODE::RD)
  {
    rdbuf_.reserve(1000000);  // 8M bits
    inbuf_  = nullptr;
    rdsize_ = 0;
    rdpos_  = 0;
  }
}

BitStream::~BitStream()
{
}

void BitStream::Write(const uint8_t* bits, size_t size)
{
  if (endian_ == ENDIAN::LITTLE)
  {
    if (wrpos_ == 0)  // internal buffer already byte-aligned
    {
      for (size_t byte = 0; byte < ((size + 7) >> 3); byte++)
      {
        wrbuf_.push_back(bits[byte]);
      }

      wrpos_ = size % 8;
      bitcounter_ += size;
    }
    else
    {
      size_t b          = 0;
      uint8_t& lastbyte = wrbuf_.back();

      // Fill up last byte
      while ((wrpos_ != 0) && (b != size))
      {
		  lastbyte = static_cast<uint8_t>(lastbyte | (((bits[b >> 3]) >> (b % 8)) & 0x01) << wrpos_);
        wrpos_   = (wrpos_ + 1) % 8;
        b++;
        bitcounter_++;
      }

      // Add new bytes
      while (b != size)
      {
        if (wrpos_ == 0) wrbuf_.push_back(uint8_t(0x00));
        uint8_t& newbyte = wrbuf_.back();
        newbyte = static_cast<uint8_t>(newbyte | (((bits[b >> 3]) >> (b % 8)) & 0x01) << wrpos_);
        wrpos_  = (wrpos_ + 1) % 8;
        b++;
        bitcounter_++;
      }
    }
  }
  else
  {
    std::cout << "Writing big endian not supported ." << std::endl;
  }
}

const uint8_t* BitStream::Read(size_t size)
{
  rdbuf_.clear();

  if (size > (rdsize_ - bitcounter_))
  {
    return nullptr;
  }

  if (size > rdbuf_.size())
  {
    rdbuf_ = std::move(std::vector<uint8_t>(size));
  }

  if (endian_ == ENDIAN::LITTLE)
  {
    if (rdpos_ == 0)
    {
      size_t bytes = size >> 3;
      size_t rem   = size % 8;

      for (size_t byte = 0; byte < bytes; byte++)
      {
        rdbuf_.push_back(inbuf_[bitcounter_ >> 3]);
        bitcounter_ = bitcounter_ + 8;
      }

      if (rem != 0)
      {
        rdbuf_.push_back(static_cast<uint8_t>(((static_cast<uint8_t>(1) << rem) - 1) &
                         (inbuf_[bitcounter_])));
        bitcounter_ += rem;
      }

      rdpos_ = rem;
    }

    else
    {
      size_t b = 0;
      // Add new bytes
      while (b != size)
      {
        if (rdpos_ == 0) rdbuf_.push_back(uint8_t(0x00));
        uint8_t& newbyte = rdbuf_.back();
        newbyte          = static_cast<uint8_t>(newbyte |
                  (((inbuf_[bitcounter_ >> 3]) >> (bitcounter_ % 8)) & 0x01)
                      << rdpos_);
        rdpos_ = (rdpos_ + 1) % 8;
        bitcounter_++;
        b++;
      }
    }
  }
  else
  {
    std::cout << "Reading big endian not supported. " << std::endl;
  }

  return rdbuf_.data();
}

void BitStream::Load(const uint8_t* bits, size_t size)
{
  inbuf_      = bits;
  rdsize_     = size;
  rdpos_      = 0;
  bitcounter_ = 0;
}

void BitStream::Reset()
{
  bitcounter_ = 0;

  if (mode_ == MODE::RD)
  {
    rdpos_ = 0;
    rdbuf_.clear();
  }
  else if (mode_ == MODE::WR)
  {
    wrpos_ = 0;
    wrbuf_.clear();
  }
}

const uint8_t* BitStream::GetBitBuffer() const
{
  if (mode_ == MODE::RD)
    return reinterpret_cast<const uint8_t*>(rdbuf_.data());
  else if (mode_ == MODE::WR)
    return reinterpret_cast<const uint8_t*>(wrbuf_.data());

  return nullptr;
}

size_t BitStream::GetSize() const
{
  if (mode_ == MODE::RD)
    return rdsize_ - bitcounter_;
  else if (mode_ == MODE::WR)
    return bitcounter_;

  return 0;
}
}  // namespace Jcy
