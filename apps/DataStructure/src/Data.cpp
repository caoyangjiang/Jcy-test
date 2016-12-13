
// Copyright @ 2016 Caoyang Jiang

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

class BitBlockData
{
 public:
  void SetCodingMethod(uint8_t method)
  {
    codingmethod_ = method;
  }

  void SetSrcValues(const std::vector<int>& src)
  {
    src_ = src;
  }

  // void SetBitStream(const BitStream& bs)
  // {
  //   bs_ = bs;
  // }

  uint8_t GetCodingMethod() const
  {
    return codingmethod_;
  }

  const std::vector<int>& GetSrcValues() const
  {
    return src_;
  }

  // const BitStream& GetBitStream() const
  // {
  //   return bs_;
  // }

 private:
  uint8_t codingmethod_;
  std::vector<int> src_;
  // BitStream bs_;
};

class FrameData
{
 public:
  BitBlockData& operator()(size_t ibb)
  {
    return bitblocks_[ibb];
  }

  bool Pushback(const BitBlockData& bitblock)
  {
    bitblocks_.push_back(bitblock);
    return true;
  }

  size_t GetNumBitBlocks() const
  {
    return bitblocks_.size();
  }

  uint32_t GetFrameId() const
  {
    return frameid_;
  }

 private:
  uint32_t frameid_;
  std::vector<BitBlockData> bitblocks_;
};

class CodingData
{
 public:
  FrameData& operator()(size_t ifrm)
  {
    return frames_[ifrm];
  }

  bool Pushback(const FrameData& frame)
  {
    frames_.push_back(frame);
    return true;
  }

  size_t GetNumFrame() const
  {
    return frames_.size();
  }

 private:
  std::vector<FrameData> frames_;
};

int main(int, char**)
{
  CodingData topdata;

  topdata.Pushback(FrameData());

  topdata(0).Pushback(BitBlockData());
  topdata(0)(0).SetCodingMethod(1);

  std::cout << topdata(0)(0).GetCodingMethod() << std::endl;
  return 0;
}
