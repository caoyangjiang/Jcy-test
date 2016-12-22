// Copyright @ 2016 Caoyang Jiang

#include "Jcy/MiscTools/DdsWriter.h"
#include <fstream>
#include <memory>

namespace Jcy
{
bool DdsWriter::Write(const std::string& dxtfile,
                      int width,
                      int height,
                      const std::string& ofile)
{
  DDS dds;
//  printf("%ld", sizeof(DDS));
  std::ofstream ofs;
  std::ifstream ifs;
  std::unique_ptr<uint8_t[]> buf;

  ofs.open(dxtfile, std::ios::out | std::ios::trunc | std::ios::binary);
  ifs.open(ofile, std::ios::in | std::ios::binary);
  dds.header.dwHeight = static_cast<DWORD>(width);   // 2048
  dds.header.dwWidth  = static_cast<DWORD>(height);  // 8192
  buf                 = std::make_unique<uint8_t[]>(static_cast<size_t>(width * height / 2));
  ifs.read(reinterpret_cast<char*>(buf.get()), width * height / 2);
  ofs.write(reinterpret_cast<const char*>(&dds), sizeof(DDS));
  ofs.close();
  ifs.close();
  return 0;
}

}  // namespace Jcy
