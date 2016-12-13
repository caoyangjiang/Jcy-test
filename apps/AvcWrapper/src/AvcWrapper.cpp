// Copyright @ 2016 Caoyang Jiang

#include <fstream>
#include <iostream>
#include <memory>

class AvcWrapper
{
 public:
  AvcWrapper()
  {
  }
  ~AvcWrapper()
  {
  }

  bool WriteAvcHeader(std::ofstream& in)
  {
    const uint8_t minsps[] = {
        0x00, 0x00, 0x00, 0x01, 0x67, 0x42, 0x00, 0x0a, 0xF8, 0x41, 0xA2};
    const uint8_t minpps[] = {
        0x00, 0x00, 0x01, 0x68, 0xEB, 0x80, 0x8C, 0xB2, 0x2C};

    in.write(reinterpret_cast<const char*>(minsps), sizeof(minsps));
    in.write(reinterpret_cast<const char*>(minpps), sizeof(minpps));

    return true;
  }

  bool WriteAvcFrameHeader(std::ofstream& in)
  {
    const uint8_t minframe[] = {
        0x00, 0x00, 0x01, 0x65, 0x88, 0x84, 0x3A, 0xFD, 0x5C, 0x8B};

    in.write(reinterpret_cast<const char*>(minframe), sizeof(minframe));
    return true;
  }

  bool WriteAvcDummyData(std::ofstream& in)
  {
    const uint8_t dummy[] = {0x00, 0x48, 0x56, 0x52};

    in.write(reinterpret_cast<const char*>(dummy), sizeof(dummy));

    return true;
  }
  // Insert 0x03 in a given bitstream
  // bool EmulationPreventionInsert(BitStream&)
  // Remove 0x03 in a given bistream
  // bool EmulationPreventionRemove(BitStream&)
};

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::cout << "Not enough argument" << std::endl;
    return -1;
  }

  std::ofstream outfs(argv[2], std::ofstream::trunc | std::ofstream::binary);
  std::ifstream indata(argv[1], std::ifstream::in | std::ifstream::binary);
  std::unique_ptr<char[]> buf(
      new char[600000]);  // Pretend 100,000 vertices of x,y
  AvcWrapper wrapper;

  indata.read(buf.get(), 600000);

  wrapper.WriteAvcHeader(outfs);

  wrapper.WriteAvcFrameHeader(outfs);
  wrapper.WriteAvcDummyData(outfs);

  wrapper.WriteAvcFrameHeader(outfs);
  outfs.write(buf.get(), 600000);

  wrapper.WriteAvcFrameHeader(outfs);
  outfs.write(buf.get(), 600000);

  outfs.close();
  indata.close();

  return 0;
}
