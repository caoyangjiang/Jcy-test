// Copyright @ 2016 Caoyang Jiang

#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>
// #include "Hvr/DashVideoDecoder/Mp4Boxes.h"

class NetworkBytes
{
 public:
  NetworkBytes()
  {
  }
  ~NetworkBytes()
  {
  }

  bool Load(std::unique_ptr<uint8_t[]>& bytes, int length)
  {
    if ((length <= 0) || (bytes == nullptr)) return false;

    bytes_   = std::move(bytes);
    length_  = length;
    readpos_ = 0;
    return true;
  }

  int NumOfRemainingBytes() const
  {
    return length_ - readpos_;
  }

  bool ReadUInt8(uint8_t& val)
  {
    if (length_ < (readpos_ + 1)) return false;

    val = bytes_[readpos_];
    readpos_++;

    return true;
  }

  bool ReadUInt16(uint16_t& val)
  {
    std::array<uint16_t, 2> pos = {0, 0};

    if (length_ < (readpos_ + 2)) return false;

    pos[1] = static_cast<uint16_t>(bytes_[readpos_]);
    readpos_++;
    pos[0] = static_cast<uint16_t>(bytes_[readpos_]);
    readpos_++;

    val = (pos[1] << 8) | pos[0];
    return true;
  }

  bool ReadUInt24(uint32_t& val)
  {
    std::array<uint32_t, 3> pos = {0, 0, 0};

    if (length_ < (readpos_ + 3)) return false;

    pos[2] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[1] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[0] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;

    val = (pos[2] << 16) | (pos[1] << 8) | pos[0];
    return true;
  }

  bool ReadUInt32(uint32_t& val)
  {
    std::array<uint32_t, 4> pos = {0, 0, 0, 0};

    if (length_ < (readpos_ + 4)) return false;

    pos[3] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[2] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[1] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[0] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;

    val = (pos[3] << 24) | (pos[2] << 16) | (pos[1] << 8) | pos[0];
    return true;
  }

  bool ReadUInt64(uint64_t& val)
  {
    std::array<uint64_t, 8> pos = {0, 0, 0, 0, 0, 0, 0, 0};

    if (length_ < (readpos_ + 8)) return false;

    pos[7] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[6] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[5] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[4] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[3] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[2] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[1] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;
    pos[0] = static_cast<uint32_t>(bytes_[readpos_]);
    readpos_++;

    val = (pos[7] << 56) | (pos[6] << 48) | (pos[5] << 40) | (pos[4] << 32) |
          (pos[3] << 24) | (pos[2] << 16) | (pos[1] << 8) | pos[0];

    return true;
  }

  bool SeekBack(int bytes)
  {
    if (readpos_ >= bytes)
    {
      readpos_ = readpos_ - bytes;
      return true;
    }

    readpos_ = 0;
    return false;
  }

 private:
  std::unique_ptr<uint8_t[]> bytes_;
  int length_  = 0;
  int readpos_ = 0;
};

class Box
{
 public:
  Box()
  {
  }

  ~Box()
  {
  }

  int Parse(NetworkBytes& ntbytes)
  {
    ntbytes.ReadUInt32(size_);
    ntbytes.ReadUInt32(boxtype_);

    if (size_ == 1)
    {
      std::cout << "Large size is not handled." << std::endl;
      return -1;
    }

    return 8;
  }

  uint32_t GetSize()
  {
    return size_;
  }

  uint32_t GetType()
  {
    return boxtype_;
  }

 protected:
  uint32_t size_    = 0;
  uint32_t boxtype_ = 0;
};

class FullBox : protected Box
{
 public:
  FullBox()
  {
  }
  ~FullBox()
  {
  }

  int Parse(NetworkBytes& ntbytes)
  {
    int sz = Box::Parse(ntbytes);
    ntbytes.ReadUInt8(version_);
    ntbytes.ReadUInt24(flags_);

    return sz + 4;
  }

  uint8_t GetVersion() const
  {
    return version_;
  }

  uint32_t GetFlags() const
  {
    return flags_;
  }

 protected:
  uint8_t version_ = 0;
  uint32_t flags_  = 0;
};

class TFDT : protected FullBox
{
 public:
  TFDT()
  {
  }
  ~TFDT()
  {
  }

  int Parse(NetworkBytes& ntbytes)
  {
    int usedbytes = FullBox::Parse(ntbytes);
    if (version_ == 0)
    {
      ntbytes.ReadUInt32(basedmediadecodetime_);
      usedbytes += 4;
    }
    else
    {
      std::cout << "Version > 0 not handled" << std::endl;
      return 0;
    }

    return usedbytes;
  }
  static const uint32_t TYPE_ = 0x74666474;

 private:
  uint32_t basedmediadecodetime_ = 0;
};

class TRUN : protected FullBox
{
 public:
  TRUN()
  {
  }
  ~TRUN()
  {
  }

  int Parse(NetworkBytes& ntbytes)
  {
    uint32_t uint32tmp;
    int32_t int32tmp;
    int byteused = FullBox::Parse(ntbytes);

    ntbytes.ReadUInt32(samplecount_);
    byteused += 4;

    if (flags_ & 0x00000001)
    {
      ntbytes.ReadUInt32(uint32tmp);
      int32tmp = static_cast<int32_t>(uint32tmp);
      dataoffset_.push_back(int32tmp);
      byteused += 4;
    }

    if (flags_ & 0x00000004)
    {
      ntbytes.ReadUInt32(uint32tmp);
      firstsampleflags_.push_back(uint32tmp);
      byteused += 4;
    }

    for (uint32_t count = 0; count < samplecount_; count++)
    {
      if (flags_ & 0x00000100)
      {
        ntbytes.ReadUInt32(uint32tmp);
        sampleduration_.push_back(uint32tmp);
        byteused += 4;
      }

      if (flags_ & 0x00000200)
      {
        ntbytes.ReadUInt32(uint32tmp);
        samplesize_.push_back(uint32tmp);
        byteused += 4;
      }

      if (flags_ & 0x00000400)
      {
        ntbytes.ReadUInt32(uint32tmp);
        sampleflags_.push_back(uint32tmp);
        byteused += 4;
      }

      if (version_ == 0)
      {
        if (flags_ & 0x00000800)
        {
          ntbytes.ReadUInt32(uint32tmp);
          samplecompositiontimeoffset_.push_back(uint32tmp);
          byteused += 4;
        }
      }
      else
      {
        std::cout << "Version > 0 not handled" << std::endl;
        return 0;
      }
    }

    return byteused;
  }

  static const uint32_t TYPE_ = 0x7472756e;

 private:
  uint32_t samplecount_ = 0;
  std::vector<int32_t> dataoffset_;
  std::vector<uint32_t> firstsampleflags_;
  std::vector<uint32_t> sampleduration_;
  std::vector<uint32_t> samplesize_;
  std::vector<uint32_t> sampleflags_;
  std::vector<uint32_t> samplecompositiontimeoffset_;
};

class TFHD : protected FullBox
{
 public:
  TFHD()
  {
  }
  ~TFHD()
  {
  }

  int Parse(NetworkBytes& ntbytes)
  {
    int byteused = FullBox::Parse(ntbytes);
    uint32_t uint32tmp;
    uint64_t uint64tmp;

    ntbytes.ReadUInt32(trackid_);
    byteused += 4;

    if (flags_ & 0x00000001)
    {
      ntbytes.ReadUInt64(uint64tmp);
      basedataoffset_.push_back(uint64tmp);
      byteused += 8;
    }

    if (flags_ & 0x00000002)
    {
      ntbytes.ReadUInt32(uint32tmp);
      sampledescriptionindex_.push_back(uint32tmp);
      byteused += 4;
    }

    if (flags_ & 0x00000008)
    {
      ntbytes.ReadUInt32(uint32tmp);
      defaultsampleduration_.push_back(uint32tmp);
      byteused += 4;
    }

    if (flags_ & 0x00000010)
    {
      ntbytes.ReadUInt32(uint32tmp);
      defaultsamplesize_.push_back(uint32tmp);
      byteused += 4;
    }

    if (flags_ & 0x00000020)
    {
      ntbytes.ReadUInt32(uint32tmp);
      defaultsampleflags_.push_back(uint32tmp);
      byteused += 4;
    }

    return byteused;
  }

  static const uint32_t TYPE_ = 0x74666864;

 private:
  uint32_t trackid_;
  std::vector<uint64_t> basedataoffset_;
  std::vector<uint32_t> sampledescriptionindex_;
  std::vector<uint32_t> defaultsampleduration_;
  std::vector<uint32_t> defaultsamplesize_;
  std::vector<uint32_t> defaultsampleflags_;
};

class TRAF : protected Box
{
 public:
  TRAF()
  {
  }
  ~TRAF()
  {
  }

  int Parse(NetworkBytes& ntbytes)
  {
    Box box;
    int byteused = Box::Parse(ntbytes);

    while (byteused < static_cast<int>(size_))
    {
      box.Parse(ntbytes);
      ntbytes.SeekBack(8);

      if (box.GetType() == TFHD::TYPE_)
      {
        byteused += tfhd_.Parse(ntbytes);
      }
      else if (box.GetType() == TFDT::TYPE_)
      {
        TFDT tfdt;
        byteused += tfdt.Parse(ntbytes);
        tfdt_.push_back(tfdt);
      }
      else if (box.GetType() == TRUN::TYPE_)
      {
        TRUN trun;
        byteused += trun.Parse(ntbytes);
        trun_.push_back(trun);
      }
      else
      {
        std::cout << "Unknow  box inside TRAF " << std::endl;
        return 0;
      }
    }

    if (byteused != static_cast<int>(size_))
    {
      std::cout << "Parse TRAF bytes failed" << std::endl;
      return 0;
    }

    return byteused;
  }

  static const uint32_t TYPE_ = 0x74726166;

 private:
  TFHD tfhd_;
  std::vector<TFDT> tfdt_;  // size=0: none, otherwise size=1;
  std::vector<TRUN> trun_;  // size=0: none, otherwise size>0;
};

/**
 * @brief      Movie Fragment Header Box (in Movie Fragment Box)
 */
class MFHD : protected FullBox
{
 public:
  MFHD()
  {
  }
  ~MFHD()
  {
  }

  int Parse(NetworkBytes& ntbytes)
  {
    int sz = FullBox::Parse(ntbytes);
    ntbytes.ReadUInt32(sequencenumber_);
    return sz + 4;
  }

  uint32_t GetSequenceNumber() const
  {
    return sequencenumber_;
  }

  static const uint32_t TYPE_ = 0x6d666864;

 private:
  uint32_t sequencenumber_;
};

/**
 * @brief      Moview Fragment Box
 */
class MOOF : protected Box
{
 public:
  MOOF()
  {
  }
  ~MOOF()
  {
  }

  int Parse(NetworkBytes& ntbytes)
  {
    int byteused = Box::Parse(ntbytes);

    while (byteused < static_cast<int>(size_))
    {
      Box box;
      box.Parse(ntbytes);
      ntbytes.SeekBack(8);

      if (box.GetType() == MFHD::TYPE_)
      {
        int sz = mfhd_.Parse(ntbytes);

        if (sz == 0) return 0;

        byteused += sz;
      }
      else if (box.GetType() == TRAF::TYPE_)
      {
        TRAF traf;
        int sz = traf.Parse(ntbytes);
        if (sz == 0) return 0;
        byteused += sz;
        traf_.push_back(traf);
      }
      else
      {
        std::cout << "Unknow box inside MOOF" << std::endl;
        return 0;
      }
    }

    if (byteused != static_cast<int>(size_))
    {
      std::cout << "Parse MOOF bytes failed" << std::endl;
      std::cout << byteused << " " << size_ << std::endl;
      return 0;
    }

    return size_;
  }

  static const uint32_t TYPE_ = 0x6d6f6f66;

 private:
  MFHD mfhd_;
  std::vector<TRAF> traf_;
};

class MoofParser
{
 public:
  MoofParser()
  {
  }

  ~MoofParser()
  {
  }

  bool Parse(std::unique_ptr<uint8_t[]>& buffer, int buffersize)
  {
    ntbytes_.Load(buffer, buffersize);

    if (moof_.Parse(ntbytes_) == 0) return false;

    return true;
  }

  // bool GetNextFrame(uint8_t*& packet, int& size)
  // {
  // }

  // int GetTotalSample() const
  // {
  //   return totalsample;
  // }

  // int GetCurrSampleId() const
  // {
  //   return currsample;
  // }

  // bool Reset()
  // {
  // }

 private:
  int totalsample = 0;
  int currsample  = 0;
  MOOF moof_;
  std::unique_ptr<uint8_t* []> samples_;
  std::unique_ptr<uint32_t[]> samplesizes_;
  NetworkBytes ntbytes_;
};

int main(int argc, char** argv)
{
  std::ifstream ifs;
  std::unique_ptr<uint8_t[]> buffer;
  MoofParser parser;
  int buffersize;

  if (argc < 2)
  {
    std::cout << "Not enough argument." << std::endl;
    return 1;
  }

  ifs.open(argv[1], std::ifstream::in | std::ifstream::binary);

  ifs.seekg(0, ifs.end);
  buffersize = ifs.tellg();
  ifs.seekg(0, ifs.beg);
  buffer = std::unique_ptr<uint8_t[]>(new uint8_t[buffersize]);
  ifs.read(reinterpret_cast<char*>(buffer.get()), buffersize);
  parser.Parse(buffer, buffersize);

  ifs.close();
  return 0;
}
