// Copyright @ 2016 Caoyang Jiang

#include "Jcy/MiscTools/HvrMeshContainer.h"

namespace Jcy
{

HvrMeshFrameCreator::HvrMeshFrameCreator()
{
}

HvrMeshFrameCreator::~HvrMeshFrameCreator()
{
}

void HvrMeshFrameCreator::AddVertex(const std::vector<float>& vertex)
{
  vertices_.push_back(vertex);
}

void HvrMeshFrameCreator::Write(const std::string& filename)
{
  HvrMeshFrame frame;
  std::ofstream ofs(filename, std::ios::binary | std::ios::trunc);

  frame.datatype = 0;
  frame.chlayout = 0;
  frame.numch    = static_cast<uint8_t>(vertices_[0].size());
  frame.datasize = static_cast<uint32_t>(frame.numch * sizeof(float) * vertices_.size());

  ofs.write(reinterpret_cast<const char*>(&frame), sizeof(HvrMeshFrame));
  for (size_t v = 0; v < vertices_.size(); v++)
  {
    ofs.write(reinterpret_cast<const char*>(vertices_[v].data()),
              static_cast<std::streamsize>(frame.numch * sizeof(float)));
  }
  ofs.close();
}

HvrMeshSequenceCreator::HvrMeshSequenceCreator()
{
}

HvrMeshSequenceCreator::~HvrMeshSequenceCreator()
{
}

void HvrMeshSequenceCreator::Merge(const std::string& outputfilename,
                                   const std::string& inputfileprefix,
                                   int first,
                                   int last)
{
  std::string fname = std::to_string(first);

  auto Probe = [&]() -> int {
    for (int d = static_cast<int>(fname.size()); d < 11; d++)
    {
      std::ifstream in;
      fname = std::to_string(first);
      fname.insert(fname.begin(), static_cast<uint32_t>(d) - static_cast<uint32_t>(fname.size()), '0');
      fname = inputfileprefix + fname + ".hmf";
      in.open(fname);
      if (in.fail())
      {
        in.close();
        continue;
      }
      else
      {
        in.close();
        return d;
      }
    }
    return 0;
  };

  int numdigit      = Probe();
  int headersize    = sizeof(HvrMeshSequence);
  int framesegsize  = (last - first + 1) * static_cast<int>(sizeof(int)) * 2;
  int datasegoffset = headersize + framesegsize;
  std::vector<int> begs;
  std::vector<int> ends;
  HvrMeshSequence hms;
  std::ofstream ofs;

  hms.vmax = 0;
  // Get all frame sizes
  for (int ifrm = first; ifrm <= last; ifrm++)
  {
    HvrMeshFrame hmf;
    std::ifstream ifs;
    uint32_t vcnt;

    fname = std::to_string(ifrm);
    fname.insert(fname.begin(), static_cast<uint32_t>(numdigit) - static_cast<uint32_t>(fname.size()), '0');
    fname = inputfileprefix + fname + ".hmf";

    ifs.open(fname, std::ios::in | std::ios::binary);
    ifs.read(reinterpret_cast<char*>(&hmf), sizeof(HvrMeshFrame));

    if (ifrm == first)
    {
      hms.datatype   = hmf.datatype;
      hms.chlayout   = hmf.chlayout;
      hms.numch      = hmf.numch;
      hms.totalframe = static_cast<uint32_t>(last - first) + 1;
    }

    begs.push_back(datasegoffset);
    datasegoffset += hmf.datasize;
    ends.push_back(datasegoffset - 1);

    vcnt     = hmf.datasize / (hms.numch * (hms.datatype == 0 ? 4 : 2));
    hms.vmax = vcnt > hms.vmax ? vcnt : hms.vmax;
    ifs.close();
  }

  ofs.open(outputfilename, std::ios::binary | std::ios::trunc);

  // Write header
  ofs.write(reinterpret_cast<char*>(&hms), sizeof(HvrMeshSequence));

  // Write frame segment
  for (uint32_t i = 0; i < hms.totalframe; i++)
  {
    int beg = begs[i];
    int end = ends[i];

    ofs.write(reinterpret_cast<const char*>(&beg), sizeof(int));
    ofs.write(reinterpret_cast<const char*>(&end), sizeof(int));
  }

  // Write frame data
  for (int ifrm = first; ifrm <= last; ifrm++)
  {
    HvrMeshFrame hmf;
    std::ifstream ifs;
    std::unique_ptr<uint8_t[]> vbuf;

    fname = std::to_string(ifrm);
    fname.insert(fname.begin(), static_cast<uint32_t>(numdigit) - static_cast<uint32_t>(fname.size()), '0');
    fname = inputfileprefix + fname + ".hmf";

    ifs.open(fname, std::ios::in | std::ios::binary);
    ifs.read(reinterpret_cast<char*>(&hmf), sizeof(HvrMeshFrame));
    vbuf = std::unique_ptr<uint8_t[]>(new uint8_t[hmf.datasize]);
    ifs.read(reinterpret_cast<char*>(vbuf.get()), hmf.datasize);
    ofs.write(reinterpret_cast<const char*>(vbuf.get()), hmf.datasize);
  }

  ofs.close();
}

HvrMeshSequenceLoader::HvrMeshSequenceLoader()
{
}

HvrMeshSequenceLoader::~HvrMeshSequenceLoader()
{
  ifs_.close();
}

bool HvrMeshSequenceLoader::Parse(const std::string& filename)
{
  ifs_.open(filename, std::ios::binary | std::ios::in);

  if (!ifs_.is_open())
  {
    std::cout << "Open HvrMeshSequence file failed" << std::endl;
    return false;
  }

  ifs_.read(reinterpret_cast<char*>(&hms_), sizeof(hms_));

  for (uint32_t i = 0; i < hms_.totalframe; i++)
  {
    int beg, end;
    ifs_.read(reinterpret_cast<char*>(&beg), sizeof(int));
    ifs_.read(reinterpret_cast<char*>(&end), sizeof(int));
    begs_.push_back(beg);
    sizes_.push_back(end - beg + 1);
    std::cout << beg << " " << end << std::endl;
  }
  return true;
}

template <typename T>
std::vector<std::vector<T>> HvrMeshSequenceLoader::GetVertexByFrameID(
    int frameid)
{
  std::vector<std::vector<T>> vbuf;
  int numofvertex =
      sizes_[static_cast<size_t>(frameid)] / (hms_.numch * (hms_.datatype == 0 ? 4 : 2));

  ifs_.seekg(begs_[static_cast<size_t>(frameid)], ifs_.beg);

  for (int v = 0; v < numofvertex; v++)
  {
    std::vector<T> vt;
    vt.resize(hms_.numch);
    ifs_.read(reinterpret_cast<char*>(vt.data()), static_cast<std::streamsize>(sizeof(T) * hms_.numch));
    vbuf.push_back(vt);
  }
  return vbuf;
}

const HvrMeshSequence& HvrMeshSequenceLoader::GetHMS() const
{
  return hms_;
}

template std::vector<std::vector<float>>
HVR_WINDOWS_DLL_API HvrMeshSequenceLoader::GetVertexByFrameID<float>(int frameid);
template std::vector<std::vector<int16_t>>
HVR_WINDOWS_DLL_API HvrMeshSequenceLoader::GetVertexByFrameID<int16_t>(int frameid);
}  // namespace hvr
