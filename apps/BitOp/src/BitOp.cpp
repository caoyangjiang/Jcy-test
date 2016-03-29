
// Copyright @ 2016 Caoyang Jiang

#include <boost/dynamic_bitset.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

class BitStream
{
 public:
  BitStream()
  {
  }

  BitStream& operator=(const BitStream& bs)
  {
    bitcontainer_ = bs.bitcontainer_;
    rdpos_        = bs.rdpos_;
    wtpos_        = bs.wtpos_;

    return *this;
  }

  BitStream(const BitStream& bs)
  {
    bitcontainer_ = bs.bitcontainer_;
    rdpos_        = bs.rdpos_;
    wtpos_        = bs.wtpos_;
  }

  ~BitStream()
  {
  }

  bool Load(const std::string& filename)
  {
    std::ifstream bsfs(filename, std::ifstream::in | std::ifstream::binary);
    std::unique_ptr<char[]> bsbuffer;
    size_t filesize = 0;

    if (!bsfs.is_open())
    {
      std::cout << "[ERROR]: Open input file failed." << std::endl;
      return true;
    }

    bsfs.seekg(0, bsfs.end);
    filesize = bsfs.tellg();
    bsfs.seekg(0, bsfs.beg);
    bsbuffer = std::unique_ptr<char[]>(new char[filesize]);
    bsfs.read(bsbuffer.get(), filesize);

    Clear();
    for (size_t ibyte = 0; ibyte < filesize; ibyte++)
    {
      char byte;
      byte = bsbuffer[ibyte];

      for (size_t ibit = 0; ibit < 8; ibit++)
      {
        bitcontainer_.push_back((0x01 & (byte >> ibit)) == 1);
      }
    }

    bsfs.close();
    return true;
  }

  bool Load(const std::string& filename, int nbits)
  {
    std::ifstream bsfs(filename, std::ifstream::in | std::ifstream::binary);
    std::unique_ptr<char[]> bsbuffer;
    size_t filesize = 0;
    int bitcount    = 0;

    if (!bsfs.is_open())
    {
      std::cout << "[ERROR]: Open input file failed." << std::endl;
      return true;
    }

    bsfs.seekg(0, bsfs.end);
    filesize = bsfs.tellg();
    bsfs.seekg(0, bsfs.beg);
    bsbuffer = std::unique_ptr<char[]>(new char[filesize]);
    bsfs.read(bsbuffer.get(), filesize);

    Clear();
    for (size_t ibyte = 0; ibyte < filesize; ibyte++)
    {
      char byte;
      byte = bsbuffer[ibyte];

      for (size_t ibit = 0; ibit < 8; ibit++)
      {
        if (bitcount == nbits)
          goto BITFILLED;
        else
          bitcount++;

        bitcontainer_.push_back((0x01 & (byte >> ibit)) == 1);
      }
    }
  BITFILLED:
    bsfs.close();
    return true;
  }

  bool Load(const BitStream& bs)
  {
    Clear();
    bitcontainer_ = bs.Get();
    return true;
  }

  bool Load(const BitStream& bs, int nbits)
  {
    size_t size = bs.Size() > static_cast<size_t>(nbits)
                      ? static_cast<size_t>(nbits)
                      : bs.Size();

    Clear();
    bitcontainer_ = boost::dynamic_bitset<>(size);

    for (size_t ibit = 0; ibit < size; ibit++)
    {
      bitcontainer_[ibit] = bs.Get()[ibit];
    }

    return true;
  }

  bool Load(const boost::dynamic_bitset<>& bits)
  {
    Clear();
    bitcontainer_ = bits;
    return true;
  }

  bool Load(const boost::dynamic_bitset<>& bits, int nbits)
  {
    size_t size = bits.size() > static_cast<size_t>(nbits)
                      ? static_cast<size_t>(nbits)
                      : bits.size();

    Clear();
    bitcontainer_ = boost::dynamic_bitset<>(size);
    for (size_t ibit = 0; ibit < size; ibit++)
    {
      bitcontainer_[ibit] = bits[ibit];
    }
    return true;
  }

  bool Save(const std::string& filename) const
  {
    std::ofstream bsfs(
        filename,
        std::ofstream::out | std::ofstream::binary | std::ofstream::trunc);

    bsfs.close();
    return true;
  }

  bool Save(const std::string& filename, int nbits) const
  {
    std::ofstream bsfs(
        filename,
        std::ofstream::out | std::ofstream::binary | std::ofstream::trunc);

    if (nbits == 0) return false;
    bsfs.close();
    return true;
  }

  BitStream Read(int nbits)
  {
    BitStream bs;
    size_t rembits = bitcontainer_.size() - rdpos_;
    size_t size    = static_cast<size_t>(nbits) > rembits
                      ? rembits
                      : static_cast<size_t>(nbits);
    boost::dynamic_bitset<> bits(size);

    for (size_t ibit = 0; ibit < size; ibit++)
    {
      bits[ibit] = bitcontainer_[rdpos_];
      rdpos_++;
    }

    bs.Load(bits);
    return bs;
  }

  BitStream Peek(int nbits)
  {
    BitStream bs;
    size_t rembits = bitcontainer_.size() - rdpos_;
    size_t size    = static_cast<size_t>(nbits) > rembits
                      ? rembits
                      : static_cast<size_t>(nbits);

    boost::dynamic_bitset<> bits(size);
    for (size_t ibit = 0; ibit < size; ibit++)
    {
      bits[ibit] = bitcontainer_[rdpos_ + ibit];
    }

    bs.Load(bits);
    return bs;
  }

  bool Append(const BitStream& bs)
  {
    for (size_t ibit = 0; ibit < bs.Size(); ibit++)
    {
      bitcontainer_.push_back(bs[ibit]);
    }

    return true;
  }

  bool Append(const BitStream& bs, int nbits)
  {
    for (size_t ibit = 0; ibit < static_cast<size_t>(nbits); ibit++)
    {
      bitcontainer_.push_back(bs[ibit]);
    }

    return true;
  }

  bool Append(bool bit)
  {
    bitcontainer_.push_back(bit);
    return true;
  }

  bool Mirror()
  {
    boost::dynamic_bitset<> bits(bitcontainer_.size());

    for (size_t ibit = 0; ibit < bitcontainer_.size(); ibit++)
    {
      bits[ibit] = bitcontainer_[bitcontainer_.size() - ibit - 1];
    }

    bitcontainer_ = bits;
    return true;
  }

  const boost::dynamic_bitset<>& Get() const
  {
    return bitcontainer_;
  }

  size_t Size() const
  {
    return bitcontainer_.size();
  }

  size_t ReadPos() const
  {
    return rdpos_;
  }

  size_t WritePos() const
  {
    return wtpos_;
  }

  bool IsEmpty() const
  {
    return bitcontainer_.size() == 0;
  }

  bool operator[](size_t pos) const
  {
    return bitcontainer_[pos];
  }

  void Clear()
  {
    bitcontainer_.clear();
    rdpos_ = 0;
    wtpos_ = 0;
  }

 private:
  boost::dynamic_bitset<> bitcontainer_;
  size_t rdpos_ = 0;
  size_t wtpos_ = 0;
};

class BitUtility
{
 public:
  static int BsToInt32(const BitStream& bs)
  {
    int num     = 0;
    size_t size = bs.Size() < 32 ? bs.Size() : 32;

    for (size_t i = 0; i < size; i++)
    {
      num |= (bs[i] << i);
    }

    return num;
  }

  static uint32_t BsToUInt32(const BitStream& bs)
  {
    uint32_t num = 0;
    size_t size  = bs.Size() < 32 ? bs.Size() : 32;

    for (size_t i = 0; i < size; i++)
    {
      num |= (bs[i] << i);
    }
    return num;
  }

  static BitStream IntToBs(int num)
  {
    BitStream bs;
    int nbits = 32;

    while (nbits != 0)
    {
      bs.Append((num & 0x00000001) == 1);
      num >>= 1;
      nbits--;
    }

    return bs;
  }

  static BitStream IntToBs(int num, int nbits)
  {
    BitStream bs;

    while (nbits != 0)
    {
      bs.Append((num & 0x00000001) == 1);
      num >>= 1;
      nbits--;
    }

    return bs;
  }

  static int SignToUnsigned(int num)
  {
    bool pos   = num > 0;
    int posnum = num < 0 ? -num : num;

    return posnum * 2 - pos;
  }

  static int UnsignedToSign(int num)
  {
    int val = (num + 1) / 2;

    return (num % 2) == 0 ? -val : val;
  }
};

class GolombDecoder
{
 public:
  GolombDecoder()
  {
  }
  virtual ~GolombDecoder()
  {
  }

  virtual bool Load(const BitStream& bs)                  = 0;
  virtual bool Decode()                                   = 0;
  virtual const std::vector<int>& GetDecodeBuffer() const = 0;
  virtual const BitStream& GetBitBuffer() const           = 0;
};

class GolombEncoder
{
 public:
  GolombEncoder()
  {
  }
  virtual ~GolombEncoder()
  {
  }

  virtual bool Load(const std::vector<int>& src)       = 0;
  virtual bool Encode()                                = 0;
  virtual const BitStream& GetEncodeBuffer() const     = 0;
  virtual const std::vector<int>& GetSrcBuffer() const = 0;
};

class ExpGolombDecoder : public GolombDecoder
{
 public:
  ExpGolombDecoder()
  {
  }
  ~ExpGolombDecoder() override
  {
  }

  bool Load(const BitStream& bs) override
  {
    db_.clear();
    bs_ = bs;
    return true;
  }

  bool Decode() override
  {
    BitStream suffix;

    while (bs_.ReadPos() < bs_.Size())
    {
      int leadingzerobits = -1;
      bool bit            = 0;

      for (; !bit; leadingzerobits++)
      {
        bit = bs_.Read(1)[0];
      }

      suffix = bs_.Read(leadingzerobits);
      suffix.Mirror();

      int codenum = (1 << leadingzerobits) - 1 + BitUtility::BsToInt32(suffix);
      db_.push_back(codenum);
    }

    return true;
  }

  const std::vector<int>& GetDecodeBuffer() const override
  {
    return db_;
  }

  const BitStream& GetBitBuffer() const override
  {
    return bs_;
  }

 private:
  BitStream bs_;
  std::vector<int> db_;
};

class ExpGolombEncoder : public GolombEncoder
{
 public:
  ExpGolombEncoder()
  {
  }
  ~ExpGolombEncoder()
  {
  }

  bool Load(const std::vector<int>& src)
  {
    bs_.Clear();
    src_ = src;
    return true;
  }

  bool Encode()
  {
    for (size_t isrc = 0; isrc < src_.size(); isrc++)
    {
      int leadingzeroplusone = 0;
      int num                = src_[isrc];
      BitStream rem;

      while (num >= ((1 << leadingzeroplusone) - 1))
      {
        leadingzeroplusone++;
      }

      for (size_t ileadzero = 0;
           ileadzero < static_cast<size_t>(leadingzeroplusone - 1);
           ileadzero++)
      {
        bs_.Append(0);
      }

      bs_.Append(1);

      rem = BitUtility::IntToBs(num - ((1 << (leadingzeroplusone - 1)) - 1),
                                leadingzeroplusone - 1);
      rem.Mirror();
      bs_.Append(rem);
    }

    return true;
  }

  const BitStream& GetEncodeBuffer() const
  {
    return bs_;
  }

  const std::vector<int>& GetSrcBuffer() const
  {
    return src_;
  }

 private:
  BitStream bs_;
  std::vector<int> src_;
};

// const int array[] = {
//     1838, -9, -25, -5, 13, -16, 19,  -19, 0,   3,   -12, 9,  14,  11, -3, -9,
//     -19,  -8, 7,   6,  2,  7,   11,  -1,  -3,  2,   2,   9,  2,   -2, -3, 2,
//     -8,   -8, -5,  -1, 3,  -2,  9,   -4,  -12, -3,  1,   2,  4,   0,  11, 7,
//     -1,   4,  -1,  4,  -2, -7,  -4,  -3,  0,   1,   -3,  -4, -4,  -4, 9,  4,
//     3,    -1, -1,  7,  2,  -5,  -3,  -4,  3,   -3,  3,   7,  -4,  -3, -6, 5,
//     2,    -9, -11, 7,  5,  -6,  -1,  10,  -2,  -20, 10,  1,  -6,  0,  8,  2,
//     -3,   5,  -2,  -5, 5,  -3,  -6,  6,   5,   -11, 1,   1,  -2,  3,  1,  -1,
//     -3,   3,  6,   -1, 6,  -3,  -12, -1,  7,   -6,  -6,  6,  1,   -3, 1,  4,
//     -5,   -1, 0,   2,  7,  -8,  -3,  0,   -9,  1,   -2,  -1, 2,   -3, -2, 3,
//     -1,   -3, 1,   0,  1,  4,   -2,  6,   7,   -10, 4,   -7, -1,  3,  -1, 4,
//     10,   -6, -2,  -1, 0,  -6,  10,  1,   10,  -4,  -5,  4,  -4,  2,  1,  1,
//     7,    -4, 1,   2,  -2, 4,   -1,  -3,  -2,  2,   -4,  -1, 2,   0,  2,  -3,
//     5,    -4, 2,   0,  -1, -2,  3,   0,   -1,  -1,  4,   2,  -10, 6,  -3, -3,
//     1,    -4, 8,   -1, -1, 4,   -3,  4,   -3,  -5,  7,   4,  -1,  3,  0,  0,
//     -5,   -3, 1,   -1, 6,  -5,  6,   0,   -1,  1,   5,   -8, 6,   -4, 0,  -1,
//     2,    7,  0,   -1, -3, -1,  -7,  -3,  3,   3,   3,   -1, 4,   0,  -1, -2,
//     2,    -1, -3,  6,  0,  2,   6,   -1,  -1,  -5,  -5,  2,  -4,  6,  2,  2,
//     0,    -3, -1,  2,  2,  -1,  7,   -1,  0,   2,   0,   2,  3,   -5, 0,  2,
//     -5,   -2, 1,   4,  -4, 2,   3,   3,   0,   1,   -4,  3,  -1,  5,  1,  0,
//     -1,   3,  -7,  1,  1,  -3,  2,   -6,  3,   2,   -3,  4,  5,   3,  -4, -2,
//     0,    -2, 0,   6,  -1, 1,   -1,  -3,  0,   2,   -1,  -3, 2,   1,  -1, 3,
//     2,    -1, -2,  2,  -1, -1,  2,   -5,  1,   2,   0,   2,  4,   -3, -4, -3,
//     4,    -1, -2,  5,  -1, 3,   -2,  -2,  7,   -3,  0,   1,  1,   -3, 2,  -1,
//     5,    1,  0,   -1, -3, -3,  3,   -3,  -1,  1,   4,   -1, 1,   3,  -1, 0,
//     1,    -1, 2,   3,  -5, 2,   -1,  -3,  0,   1,   0,   -1, -3,  -4, 3,  -2,
//     3,    4,  1,   1,  -2, 0,   2,   -3,  2,   1,   -3,  1,  1,   0,  -2, 1,
//     -1,   -2, -1,  1,  3,  2,   2,   2,   2,   -3,  -1,  1,  -1,  1,  -3, 3,
//     -2,   1,  1,   2,  -4, 3,   -4,  -1,  0,   2,   3,   0,  1,   -1, 0,  -2,
//     -3,   1,  0,   -3, 3,  -1,  1,   1,   -3,  2,   -2,  1,  1,   -4, 5,  0,
//     -1,   1,  1,   1,  -1, 0,   -1,  0,   -2,  1,   -2,  1,  0,   0,  -1, 0,
//     1,    -1, 0,   2,  1,  0,   2,   0,   -1,  0,   -3,  -2, -1,  -2, 2,  -2,
//     1,    1,  -4,  -1, -4, 2,   0,   0,   2,   3,   -4,  -1, -1,  -1, 0,  2,
//     0,    3,  -2,  -3, 0,  -3,  -1,  1,   -1,  2,   1,   -2, -1,  0,  0,  -1,
//     2,    0,  2,   0,  1,  4,   -3,  -1,  0,   0,   1,   -2, 0,   1,  0,  -1,
//     2,    2,  -4,  -1, 3,  1,   -1,  1,   -1,  -1,  1,   0,  0,   -1, -1, 1,
//     0,    2,  0,   -2, 2,  0,   -1,  1,   0,   -1,  0,   0,  -2,  -1, 1,  -1,
//     -2,   3,  0,   -2, 0,  0,   -1,  -1,  1,   1,   0,   -2, 1,   0,  0,  -2,
//     -1,   0,  -1,  0,  0,  1,   0,   0,   1,   -1,  -1,  -1, -1,  1,  1,  1,
//     0,    1,  1,   -1, 1,  0,   1,   1,   -1,  0,   1,   -1, 0,   0,  -1, -1,
//     0,    -1, 0,   0,  -1, 2,   0,   1,   1,   0,   0,   0,  1,   -1, -1, 0,
//     0,    1,  0,   -1, -2, -2,  0,   0,   1,   0,   1,   1,  1,   0,  0,  -1,
//     0,    -1, -1,  0,  0,  -1,  -1,  -1,  0,   0,   1,   0,  0,   1,  1,  1,
//     1,    0,  1,   0,  -1, 0,   0,   -1,  0,   0,   -1,  -1, 1,   1,  1,  1,
//     0,    0,  1,   0,  1,  -1,  0,   0,   -1,  -1,  -1,  -1, -1,  1,  0,  0,
//     1,    0,  0,   0,  0,  1,   0,   0,   1,   1,   -1,  0,  0,   -1, 0,  -1,
//     0,    0,  -1,  1,  0,  -1,  -1,  -1,  1,   1,   1,   0,  0,   1,  0,  0,
//     0,    -1, 0,   1,  1,  0,   0,   -1,  -1,  1,   -1,  0,  -2,  1,  1,  -1,
//     -1,   1,  0,   -3, 2,  1,   -1,  0,   -1,  -1,  -1,  0,  0,   0,  1,  -2,
//     -1,   1,  0,   -1, 0,  1,   1,   0,   0,   -1,  0,   2,  0,   -1, 1,  0,
//     -2,   1,  2,   -1, -1, 2,   0,   -1,  0,   1,   0,   0,  0,   1,  1,  -2,
//     -1,   0,  -1,  0,  0,  0,   1,   -1,  0,   1,   -1,  0,  0,   0,  1,  0,
//     0,    2,  1,   -2, 1,  -1,  -1,  1,   0,   -1,  2,   -1, 0,   0,  0,  -1,
//     2,    1,  3,   -1, -1, 1,   0,   1,   0,   0,   2,   -1, 0,   0,  -1, 1,
//     0,    0,  1,   1,  -1, 0,   0,   0,   1,   -1,  1,   0,  1,   0,  0,  -2,
//     1,    0,  -1,  0,  1,  1,   -3,  2,   -1,  -1,  0,   -2, 3,   0,  -1, 2,
//     -1,   2,  -1,  -1, 2,  0,   0,   0,   0,   0,   -2,  -1, -1,  0,  2,  -3,
//     2,    0,  0,   0,  3,  -2,  2,   -1,  0,   0,   0,   2,  0,   0,  -1, -1,
//     -2,   -2, 1,   2,  1,  0,   1,   1,   0,   0,   1,   -1, -2,  1,  0,  1,
//     3,    0,  0,   -1, -1, 0,   -3,  3,   1,   1,   1,   -1, 0,   1,  -1, -1,
//     2,    -2, -1,  2,  1,  1,   2,   -2,  -1,  1,   -2,  -1, 0,   2,  -2, 1,
//     1,    1,  -1,  -1, -2, 1,   0,   3,   0,   0,   0,   2,  -3,  0,  1,  -2,
//     2,    -2, 2,   0,  -2, 3,   3,   2,   -3,  0,   1,   -1, 0,   2,  -1, 1,
//     1,    0,  0,   0,  -1, -2,  0};

const int array[] = {
    371, 20, 20, 9,  7,  -8, 1,  11, -2, 8, 3,  5,  2,  2,  5,  4, 0, -2, -1,
    -1,  -1, 2,  1,  1,  2,  -1, 1,  1,  1, 0,  -1, -1, -1, -1, 0, 1, 1,  0,
    0,   -1, -2, 0,  0,  0,  -1, 0,  0,  1, 0,  0,  0,  1,  0,  0, 0, 1,  0,
    0,   0,  0,  0,  0,  1,  0,  0,  0,  0, 0,  0,  0,  1,  0,  0, 1, 1,  1,
    0,   0,  0,  1,  0,  1,  0,  0,  -1, 0, -1, 0,  0,  0,  0,  0, 0, 1,  0,
    -1,  0,  -1, -1, 0,  0,  0,  0,  0,  0, 0,  0,  0,  0,  -1, 0, 0, -1, -1,
    0,   0,  0,  -1, 0,  0,  0,  0,  0,  0, -1, 0,  0,  -1, 0,  0, 1, 0,  0,
    1,   0,  1,  0,  0,  0,  0,  0,  -1, 1, 1,  0,  1,  0,  0,  0, 0, 0,  -1,
    0,   0,  0,  0,  -1, 0,  -1, -1, 0,  0, 0,  0,  0,  0,  0,  0, 1, 1,  1,
    0,   0,  0,  1,  0,  0,  0,  1,  0,  1, 1,  0,  0,  0,  0,  0, 0, 1,  0,
    0,   0,  0,  0,  0,  0,  0,  0,  0,  0};

// const int array[] = {
//     -699, -157, -177, 228, -111, -37, 50, -41, 21, -16, -5,  49, -1,  -26,
//     20,
//     -8,   -17,  8,    13,  -7,   -10, 17, -11, 13, -6,  -16, 34, -19, -8, 15,
//     -10,  5,    2,    -4,  4,    -3,  -2, 11,  -7, 0,   0,   -1, 6,   -11, 3,
//     3,    -2,   -1,   3,   6,    -10, 3,  3,   -2, 2,   -3,  1,  1,   -1,  1,
//     1,    -1,   1,    -1,  4,    0,   -7, 10,  -2, -5,  4,   -3, 1,   3,   0,
//     -2,   4,    -3,   -1,  7,    -5,  0,  3,   -1, -1,  1,   -1, -1,  2, -2,
//     2,    -1,   -1,   0,   0,    3,   -4, 1,   2,  -2,  0,   1,  -3,  0,   3,
//     -5,   0,    0,    -1,  3,    0,   0,  -1,  1,  -1,  -1,  1,  0,   -3,  0,
//     2,    -5,   2,    2,   -4,   3,   1,  -2,  1,  2,   -3,  4,  1,   -7,  5,
//     1,    -4,   0,    0,   0,    -1,  0,  0,   0,  1,   1,   -1, 2,   -1, -2,
//     4,    -2,   -1,   0,   0,    -3,  -2, 3,   -1, 1,   0,   -1, 2,   -2,  1,
//     1,    -2,   0,    0,   -2,   0,   -3, 0,   1,  -6,  3,   -1, -2,  3, -3,
//     2,    2,    -3,   -3,  2,    1,   -2, 2,   -1, -4,  0,   0,  -3,  2, -1,
//     2,    4,    -4,   1,   1,    0,   -2, -2,  6,  -3,  -1,  2,  -4,  0,   0,
//     -1,   4,    1,    -1,  1,    -2,  -2, 1,   2,  1,   -2,  2,  0,   0,   0,
//     -2,   4,    -3,   -2,  0,    -2,  2,  1,   0,  1,   -2,  -2, 2,   -1,  1,
//     1,    -1,   0,    -3,  0,    0,   0,  0,   1,  2,   -3,  0,  0,   -1,  0,
//     -1,   1,    -1,   0,   1,    0,   -1, 1,   -2, 1,   0,   -3, 4,   0, -1,
//     2,    -1,   0,    1,   0,    -1,  1,  -1,  0,  2,   -2,  0,  2,   1, -1,
//     0,    1,    0,    0,   -1,   0,   0,  0,   0,  0,   3,   -2, 0,   2, 0};
int main(int argc, char**)
{
  BitStream bs;
  ExpGolombDecoder decoder;
  ExpGolombEncoder encoder;

  std::vector<int> numbers;

  if (argc < 2)
  {
    std::cout << "Not enough argument." << std::endl;
    return 0;
  }

  for (int i = 1; i < 10000; i++)
  {
    std::vector<int> src;

    src.push_back(i);
    encoder.Load(src);
    encoder.Encode();
    decoder.Load(encoder.GetEncodeBuffer());
    decoder.Decode();

    if (i != decoder.GetDecodeBuffer()[0])
    {
      std::cout << "Encoding " << i << std::endl;
      std::cout << "Decoded  " << decoder.GetDecodeBuffer()[0] << std::endl;
      std::cout << "Encoding / Decoding " << i << " Failed" << std::endl;
      break;
    }
  }

  for (int i = -1; i > -1000; i--)
  {
    std::vector<int> src;

    src.push_back(BitUtility::SignToUnsigned(i));

    encoder.Load(src);
    encoder.Encode();
    decoder.Load(encoder.GetEncodeBuffer());
    decoder.Decode();

    if (i != BitUtility::UnsignedToSign(decoder.GetDecodeBuffer()[0]))
    {
      std::cout << "Encoding " << i << std::endl;
      std::cout << "Decoded  "
                << BitUtility::UnsignedToSign(decoder.GetDecodeBuffer()[0])
                << std::endl;
      std::cout << "Encoding / Decoding " << i << " Failed" << std::endl;
      break;
    }
  }

  std::vector<int> testsrc;
  std::cout << "array size " << sizeof(array) * 4 << " bits" << std::endl;

  for (size_t ss = 0; ss < sizeof(array) / 4; ss++)
  {
    testsrc.push_back(array[ss]);
  }
  // testsrc.push_back(1);
  // testsrc.push_back(1);
  // testsrc.push_back(1);
  // testsrc.push_back(1);
  // testsrc.push_back(1);
  // testsrc.push_back(0);
  // testsrc.push_back(0);
  // testsrc.push_back(0);
  // testsrc.push_back(0);
  // testsrc.push_back(0);
  // testsrc.push_back(-1);
  // testsrc.push_back(-1);
  // testsrc.push_back(-1);
  // testsrc.push_back(-1);
  // testsrc.push_back(-1);
  // testsrc.push_back(0);
  // testsrc.push_back(0);
  // testsrc.push_back(0);
  // testsrc.push_back(0);
  // testsrc.push_back(0);

  std::vector<int> testunsigned;

  for (size_t i = 0; i < testsrc.size(); i++)
  {
    testunsigned.push_back(BitUtility::SignToUnsigned(testsrc[i]));
  }

  encoder.Load(testunsigned);
  std::cout << "Encoding ..." << std::endl;
  encoder.Encode();
  decoder.Load(encoder.GetEncodeBuffer());
  std::cout << "Decoding ..." << std::endl;
  decoder.Decode();

  std::cout << "Encoded size " << encoder.GetEncodeBuffer().Size() << " bits"
            << std::endl;

  std::vector<int> decodeunsigned = decoder.GetDecodeBuffer();

  if (testsrc.size() != decodeunsigned.size())
  {
    std::cout << "Size miss matched " << std::endl;
  }

  for (size_t i = 0; i < decodeunsigned.size(); i++)
  {
    if (testsrc[i] != BitUtility::UnsignedToSign(decodeunsigned[i]))
    {
      std::cout << "Encoded " << testsrc[i] << std::endl;
      std::cout << "Decoded " << BitUtility::UnsignedToSign(decodeunsigned[i]);
      std::cout << "Encoding / decoding failed " << std::endl;
    }
  }

  // boost::dynamic_bitset<> testvalue;
  // BitStream testbs;
  // for (int i = 0; i < 31; i++)
  // {
  //   testvalue.push_back(0);
  // }
  // testvalue.push_back(1);
  // testbs.Load(testvalue);

  // uint32_t num = BitUtility::BsToUInt32(testbs);
  // std::cout << "Value is " << num << std::endl;
}
