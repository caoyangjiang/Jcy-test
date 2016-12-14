// Copyright @ 2016 Caoyang Jiang

#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#define PRINTLDINFO

class Stream
{
 public:
  Stream()
  {
  }

  ~Stream()
  {
  }

  bool Load(const uint8_t* buffer, size_t size)
  {
    if (buffer == nullptr || size == 0)
    {
      return false;
    }

    buffer_ = std::unique_ptr<uint8_t[]>(new uint8_t[size]);
    bssize_ = size;
    std::memcpy(buffer_.get(), buffer, size);

    AnalyzeBitStreamNals();

    return true;
  }

  bool Load(const char* filepath)
  {
    std::ifstream ifs(std::string(filepath),
                      std::ifstream::in | std::ifstream::binary);

    if (!ifs.is_open())
    {
      std::cout << "Open input file failed." << std::endl;
      return false;
    }

    ifs.seekg(0, ifs.end);
    bssize_ = static_cast<int>(ifs.tellg());
    ifs.seekg(0, ifs.beg);

    buffer_ = std::unique_ptr<uint8_t[]>(new uint8_t[bssize_]);
    ifs.read(reinterpret_cast<char*>(buffer_.get()), bssize_);
    ifs.close();

    AnalyzeBitStreamNals();

    return true;
  }

  bool Write(const char* output)
  {
    std::ofstream ofs(output, std::ofstream::binary | std::ofstream::out);
    if (!ofs.is_open())
    {
      std::cout << "Open output file failed." << std::endl;
    }

    ofs.write(reinterpret_cast<char*>(buffer_.get()), bssize_);
    ofs.close();

    return true;
  }

  int NalCount() const
  {
    int total = 0;

    for (int i = 0; i < 10; i++)
    {
      total += nalcount_[i];
    }

    return total;
  }

  int GopCount() const
  {
    return nalcount_[8];
  }

  int BitStremSize() const
  {
    return bssize_;
  }

  uint8_t* GetBuffer()
  {
    return buffer_.get();
  }

  bool GetGopById(int id, Stream& ss)
  {
    if (GopCount() <= id)
    {
      std::cout << "[ERROR]: Exceed maximum GOP ID. Query GopCount First."
                << std::endl;
      return false;
    }

    ss.Load(buffer_.get() + goplocs_[id], goplocs_[id + 1] - goplocs_[id]);

    return true;
  }

  void AnalyzeBitStreamNals()
  {
    int readpos = 0;

    while (readpos < bssize_)
    {
      if (buffer_[readpos] == 0x00 && buffer_[readpos + 1] == 0x00 &&
          buffer_[readpos + 2] == 0x01)
      {
        uint8_t naltype = buffer_[readpos + 3] & 0x1F;
        switch (naltype)
        {
          case 1:
            nalcount_[1]++;
            break;  // non-IDR
          case 5:
            nalcount_[5]++;
            break;  // IDR
          case 6:
            nalcount_[6]++;
            break;  // SEI
          case 7:
            nalcount_[7]++;
            goplocs_.push_back(readpos);
            gopcount_++;
            break;  // SPS
          case 8:
            nalcount_[8]++;
            break;  // PPS
          default:
            nalcount_[0]++;
            unidnal_.push_back(naltype);
            break;
        }
        readpos += 4;
      }
      else
      {
        readpos++;
      }
    }

    goplocs_.push_back(bssize_);  // For last GOP

#ifdef PRINTLDINFO
    int total = NalCount();

    std::cout << "Total NAL: " << total << std::endl;
    std::cout << "Non-IDR: " << nalcount_[1] << std::endl;
    std::cout << "IDR: " << nalcount_[5] << std::endl;
    std::cout << "SEI: " << nalcount_[6] << std::endl;
    std::cout << "SPS: " << nalcount_[7] << std::endl;
    std::cout << "PPS: " << nalcount_[8] << std::endl;
    std::cout << "GOPLOC: ";

    for (int i = 0; i < static_cast<int>(goplocs_.size()); i++)
    {
      std::cout << goplocs_[i] << " ";
    }

    std::cout << std::endl;

    std::cout << "Unidentified NALs: ";

    for (int i = 0; i < static_cast<int>(unidnal_.size()); i++)
    {
      std::cout << static_cast<int>(unidnal_[i]) << " ";
    }

    std::cout << std::endl;

#endif
  }

 private:
  int nalcount_[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int gopcount_     = 0;
  int bssize_       = 0;

  std::unique_ptr<uint8_t[]> buffer_;  // bitstream buffer
  std::vector<uint8_t> unidnal_;       // unidentified NAlS
  std::vector<int> goplocs_;           // GOP locations
};

int main(int argc, char** argv)
{
  Stream ss;
  Stream gop;

  if (argc < 3)
  {
    std::cout << "[ERROR]: Not enough argument " << std::endl;
    return -1;
  }
  if (!ss.Load(argv[1]))
  {
    return -1;
  }

  if (ss.GetGopById(std::atoi(argv[2]), gop))
  {
    std::string outputname =
        std::string("gop_") + std::string(argv[2]) + std::string(".h264");
    gop.Write(outputname.c_str());
  }

  std::cout << "BitStream Size: " << gop.BitStremSize() << std::endl;

  return 0;
}