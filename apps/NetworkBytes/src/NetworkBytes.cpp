// Copyright @ 2016 Caoyang Jiang
#include <array>
#include <fstream>
#include <iostream>
#include <memory>

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

    ntbytes  = std::move(bytes);
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

    val = ntbytes[readpos_];
    readpos_++;

    return true;
  }

  bool ReadUInt16(uint16_t& val)
  {
    std::array<uint16_t, 2> pos = {0, 0};

    if (length_ < (readpos_ + 2)) return false;

    pos[1] = static_cast<uint16_t>(ntbytes[readpos_]);
    readpos_++;
    pos[0] = static_cast<uint16_t>(ntbytes[readpos_]);
    readpos_++;

    val = (pos[1] << 8) | pos[0];
    return true;
  }

  bool ReadUInt24(uint32_t& val)
  {
    std::array<uint32_t, 3> pos = {0, 0, 0};

    if (length_ < (readpos_ + 3)) return false;

    pos[2] = static_cast<uint32_t>(ntbytes[readpos_]);
    readpos_++;
    pos[1] = static_cast<uint32_t>(ntbytes[readpos_]);
    readpos_++;
    pos[0] = static_cast<uint32_t>(ntbytes[readpos_]);
    readpos_++;

    val = (pos[2] << 16) | (pos[1] << 8) | pos[0];
    return true;
  }

  bool ReadUInt32(uint32_t& val)
  {
    std::array<uint32_t, 4> pos = {0, 0, 0, 0};

    if (length_ < (readpos_ + 4)) return false;

    pos[3] = static_cast<uint32_t>(ntbytes[readpos_]);
    readpos_++;
    pos[2] = static_cast<uint32_t>(ntbytes[readpos_]);
    readpos_++;
    pos[1] = static_cast<uint32_t>(ntbytes[readpos_]);
    readpos_++;
    pos[0] = static_cast<uint32_t>(ntbytes[readpos_]);
    readpos_++;

    val = (pos[3] << 24) | (pos[2] << 16) | (pos[1] << 8) | pos[0];
    return true;
  }

  bool ReadUInt64(uint64_t& val)
  {
    std::array<uint64_t, 8> pos = {0, 0, 0, 0, 0, 0, 0, 0};

    if (length_ < (readpos_ + 8)) return false;

    pos[7] = static_cast<uint32_t>(ntbytes[readpos_]);
    readpos_++;
    pos[6] = static_cast<uint32_t>(ntbytes[readpos_]);
    readpos_++;
    pos[5] = static_cast<uint32_t>(ntbytes[readpos_]);
    readpos_++;
    pos[4] = static_cast<uint32_t>(ntbytes[readpos_]);
    readpos_++;
    pos[3] = static_cast<uint32_t>(ntbytes[readpos_]);
    readpos_++;
    pos[2] = static_cast<uint32_t>(ntbytes[readpos_]);
    readpos_++;
    pos[1] = static_cast<uint32_t>(ntbytes[readpos_]);
    readpos_++;
    pos[0] = static_cast<uint32_t>(ntbytes[readpos_]);
    readpos_++;

    val = (pos[7] << 56) | (pos[6] << 48) | (pos[5] << 40) | (pos[4] << 32) |
          (pos[3] << 24) | (pos[2] << 16) | (pos[1] << 8) | pos[0];

    return true;
  }

 private:
  std::unique_ptr<uint8_t[]> ntbytes;
  int length_  = 0;
  int readpos_ = 0;
};

/**
 * @brief   Box defined in (ISO/IEC 14496-12:2012 Section 4.2 Object Structure)
 */
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

/**
 * @brief    Box defined in (ISO/IEC 14496-12:2012 Section 4.2 Object Structure)
 */
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

/**
 * @brief    SegmentIndexBox (ISO/IEC 14496-12:2012 Section 8.16.3.2 Syntax)
 */
class SegmentIndexBox : protected FullBox
{
 public:
  class Reference
  {
   public:
    bool referencetype_;
    uint32_t referencedsize_;
    uint32_t subsegmentduration_;
    bool startswithSAP_;
    uint8_t SAPtype_;
    uint32_t SAPdeltatime_;
  };

 public:
  SegmentIndexBox()
  {
  }
  ~SegmentIndexBox()
  {
  }

  bool Parse(NetworkBytes& ntbytes)
  {
    // Parse Box, FullBox
    int usedbyte = FullBox::Parse(ntbytes);

    // Parse Segment Index Box
    ntbytes.ReadUInt32(referenceid_);
    usedbyte += 4;
    ntbytes.ReadUInt32(timescale);
    usedbyte += 4;

    if (version_ == 0)
    {
      std::array<uint32_t, 2> tmp = {0, 0};

      ntbytes.ReadUInt32(tmp[0]);
      ntbytes.ReadUInt32(tmp[1]);
      earliestpresentationtime_ = static_cast<uint64_t>(tmp[0]);
      usedbyte += 4;
      firstoffset_ = static_cast<uint64_t>(tmp[1]);
      usedbyte += 4;
    }
    else
    {
      ntbytes.ReadUInt64(earliestpresentationtime_);
      usedbyte += 8;
      ntbytes.ReadUInt64(firstoffset_);
      usedbyte += 8;
    }

    ntbytes.ReadUInt16(reserved_);
    usedbyte += 2;
    ntbytes.ReadUInt16(referencecount_);
    usedbyte += 2;

    reference_ = std::unique_ptr<Reference[]>(new Reference[referencecount_]);

    for (uint16_t i = 0; i < referencecount_; i++)
    {
      Reference& ref = reference_[i];
      std::array<uint32_t, 3> tmp = {0, 0, 0};

      ntbytes.ReadUInt32(tmp[0]);
      ntbytes.ReadUInt32(tmp[1]);
      ntbytes.ReadUInt32(tmp[2]);
      usedbyte += 12;

      ref.referencetype_      = tmp[0] >> 31;
      ref.referencedsize_     = tmp[0] & 0x7FFFFFFF;
      ref.subsegmentduration_ = tmp[1];
      ref.startswithSAP_      = (tmp[2]) >> 31;
      ref.SAPtype_      = static_cast<uint8_t>((tmp[2] >> 28) & 0x00000003);
      ref.SAPdeltatime_ = tmp[2] & 0x0FFFFFFF;
    }

    if (usedbyte != static_cast<int>(size_))
    {
      std::cout << "Parsing Segment Index failed" << usedbyte << size_
                << std::endl;
      return false;
    }

    // uint32_t totalsize = 0;
    // std::array<int, 2> count = {0, 0};
    // for (uint16_t i = 0; i < referencecount_; i++)
    // {
    //   if (reference_[i].referencetype_ == 0)
    //     count[0]++;
    //   else
    //     count[1]++;

    //   std::cout << reference_[i].startswithSAP_ << " "
    //             << static_cast<uint32_t>(reference_[i].SAPtype_) <<
    //             std::endl;
    //   std::cout << reference_[i].referencedsize_ << std::endl;
    //   totalsize += reference_[i].referencedsize_;
    // }

    // std::cout << count[0] << " " << count[1] << std::endl;
    // std::cout << totalsize << std::endl;
    return true;
  }

  static const uint32_t TYPE_ = 0x73696478;

 private:
  uint32_t referenceid_ = 0;
  uint32_t timescale    = 0;
  uint64_t earliestpresentationtime_;
  uint64_t firstoffset_;
  uint16_t reserved_;
  uint16_t referencecount_;
  std::unique_ptr<Reference[]> reference_;
};
int main(int, char** argv)
{
  std::ifstream ifs(argv[1], std::ifstream::in | std::ifstream::binary);
  std::unique_ptr<uint8_t[]> bytes;
  size_t filesize;
  SegmentIndexBox sidx;
  NetworkBytes ntbytes;

  ifs.seekg(0, ifs.end);
  filesize = ifs.tellg();
  ifs.seekg(0, ifs.beg);

  bytes = std::unique_ptr<uint8_t[]>(new uint8_t[filesize]);
  ifs.read(reinterpret_cast<char*>(bytes.get()), filesize);
  ntbytes.Load(bytes, filesize);

  if (!sidx.Parse(ntbytes))
  {
    std::cout << "Parse failed." << std::endl;
    return false;
  }

  ifs.close();
  return 0;
}
