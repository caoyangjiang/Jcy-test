// Copyright 2016 Caoyang Jiang

#include "Jcy/MiscTools/BitStream.h"

JCY_WINDOWS_DISABLE_ALL_WARNING
#include <iostream>
#include <string>
#include <vector>
JCY_WINDOWS_ENABLE_ALL_WARNING

namespace jcy
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

void BitStream::WriteZero()
{
  const uint8_t kZERO = 0x00;
  Write(&kZERO, 1);
}

void BitStream::WriteOne()
{
  const uint8_t kONE = 0x01;
  Write(&kONE, 1);
}

void BitStream::Write(const uint8_t* bits, size_t size)
{
  if (endian_ == ENDIAN::LITTLE)
  {
    if (bitcounter_ == 0)  // internal buffer already byte-aligned
    {
      size_t bytes = size >> 3;
      size_t rem   = size % 8;

      for (size_t byte = 0; byte < bytes; byte++)
      {
        wrbuf_.push_back(bits[byte]);
      }

      if (rem != 0)
      {
        wrbuf_.push_back(static_cast<uint8_t>(
            ((static_cast<uint8_t>(1) << rem) - 1) & (bits[bytes])));
      }
      bitcounter_ += size;
    }
    else
    {
      size_t b = 0;
      // Add new bytes
      while (b != size)
      {
        if (bitcounter_ % 8 == 0)
        {
          wrbuf_.push_back(uint8_t(0));
        }

        uint8_t& newbyte = wrbuf_.back();
        newbyte          = static_cast<uint8_t>(
            newbyte |
            (((((bits[b >> 3]) >> (b % 8)) & 0x01)) << (bitcounter_ % 8)));
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

uint8_t BitStream::ReadBit()
{
  return *Read(1);
}

uint8_t BitStream::UnsafeReadBit()
{
  return *Read(1);
}

const uint8_t* BitStream::Read(size_t size)
{
  rdbuf_.clear();

  if (size > (rdsize_ - bitcounter_) || size > 1000000)
  {
    return nullptr;
  }

  if (endian_ == ENDIAN::LITTLE)
  {
    if (bitcounter_ % 8 == 0)
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
        rdbuf_.push_back(
            static_cast<uint8_t>(((static_cast<uint8_t>(1) << rem) - 1) &
                                 (inbuf_[(bitcounter_ >> 3)])));
        bitcounter_ += rem;
      }
    }
    else
    {
      size_t b     = 0;
      size_t rdpos = 0;
      // Add new bytes
      while (b != size)
      {
        if ((b % 8) == 0)
        {
          rdbuf_.push_back(uint8_t(0x00));
          rdpos = 0;
        }

        size_t bytes = bitcounter_ >> 3;
        size_t rem   = bitcounter_ % 8;

        uint8_t& newbyte = rdbuf_.back();
        newbyte = newbyte | (((inbuf_[bytes]) >> (rem)) & 0x01) << rdpos;
        bitcounter_++;
        b++;
        rdpos++;
      }
    }
  }
  else
  {
    std::cout << "Reading big endian not supported. " << std::endl;
  }

  return rdbuf_.data();
}

const uint8_t* BitStream::UnsafeRead(size_t size)
{
  rdbuf_.clear();

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
        rdbuf_.push_back(static_cast<uint8_t>(
            ((static_cast<uint8_t>(1) << rem) - 1) & (inbuf_[bitcounter_])));
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
        newbyte          = static_cast<uint8_t>(
            newbyte |
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

void BitStream::Load(const uint8_t* bits)
{
  inbuf_      = bits;
  rdsize_     = 0;
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

size_t BitStream::GetRemSize() const
{
  if (mode_ == MODE::RD) return rdsize_ - bitcounter_;
  return 0;
}

size_t BitStream::GetWrittenSize() const
{
  if (mode_ == MODE::WR) return bitcounter_;

  return 0;
}
}  // namespace jcy
