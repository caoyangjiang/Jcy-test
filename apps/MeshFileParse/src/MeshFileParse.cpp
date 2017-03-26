// Copyright @ 2016 Caoyang Jiang

#include <string.h>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <list>
#include <memory>
#include <string>
#include <vector>
#include "Jcy/MiscTools/ObjFile.h"

#define MESH_PRIMITIVE_POINT 1
#define MESH_PRIMTIVE_LINE 2
#define MESH_PRIMITVE_TRIANGLE 3

// Buffer structure
// ---------------
// XYZ (vertices_size)
// ...
// ...
// ---------------
// UV (vertices_size)
// ...
// ...
// ---------------
// Index (indices_size)
// ...
// ...
// ---------------

typedef struct Vertice
{
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
  float u = 0.0;
  float v = 0.0;
} Vertice;

typedef struct Triangle
{
  Vertice v[3];
} Triangle;

class TriangleBuffer
{
 public:
  TriangleBuffer()
  {
  }
  ~TriangleBuffer()
  {
  }

  void Sort(int mode)
  {
    std::list<Triangle> vlist(buffer_.begin() + 1, buffer_.end());
    std::vector<Triangle> vvec;

    vvec.push_back(buffer_[0]);

    std::list<Triangle>::iterator it;
    std::list<Triangle>::iterator minit;

    while (!vlist.empty())
    {
      Triangle cur = vvec.back();
      it           = vlist.begin();
      float diff   = std::numeric_limits<float>::max();

      if (mode == 0)
      {
        while (it != vlist.end())
        {
          if (SAD(cur, *it) < diff)
          {
            minit = it;
            diff  = SAD(cur, *it);
          }
          it++;
        }
      }

      vvec.push_back(*minit);
      vlist.erase(minit);
    }

    buffer_ = vvec;
  }
  void Load(const std::vector<Triangle> vbuf)
  {
    // buffer_.assign(vbuf.begin(), vbuf.begin() + 1000);
    buffer_ = vbuf;
  }
  void SaveToFile(const std::string& filename)
  {
    std::ofstream ofs(
        filename,
        std::ofstream::out | std::ofstream::binary | std::ofstream::trunc);

    for (size_t ss = 0; ss < buffer_.size(); ss++)
    {
      for (size_t tri = 0; tri < 3; tri++)
      {
        ofs << static_cast<int16_t>(buffer_[ss].v[tri].x * 1000) << " "
            << static_cast<int16_t>(buffer_[ss].v[tri].y * 1000) << " "
            << static_cast<int16_t>(buffer_[ss].v[tri].z * 1000) << " "
            << static_cast<int16_t>(buffer_[ss].v[tri].u * 1000) << " "
            << static_cast<int16_t>(buffer_[ss].v[tri].v * 1000) << std::endl;
      }
    }

    ofs.close();
  }

  void SaveToFileAsBinary(const std::string& filename)
  {
    std::ofstream ofs(
        filename,
        std::ofstream::out | std::ofstream::binary | std::ofstream::trunc);

    for (size_t ss = 0; ss < buffer_.size(); ss++)
    {
      for (size_t tri = 0; tri < 3; tri++)
      {
        int16_t x, y, z, u, v;

        x = static_cast<int16_t>(buffer_[ss].v[tri].x * 1000);
        y = static_cast<int16_t>(buffer_[ss].v[tri].y * 1000);
        z = static_cast<int16_t>(buffer_[ss].v[tri].z * 1000);
        u = static_cast<int16_t>(buffer_[ss].v[tri].u * 1000);
        v = static_cast<int16_t>(buffer_[ss].v[tri].v * 1000);
        ofs.write(reinterpret_cast<const char*>(&(x)), sizeof(int16_t));
        ofs.write(reinterpret_cast<const char*>(&(y)), sizeof(int16_t));
        ofs.write(reinterpret_cast<const char*>(&(z)), sizeof(int16_t));
        ofs.write(reinterpret_cast<const char*>(&(u)), sizeof(int16_t));
        ofs.write(reinterpret_cast<const char*>(&(v)), sizeof(int16_t));
      }
    }

    ofs.close();
  }

 private:
  float SAD(const Triangle& t1, const Triangle& t2)
  {
    float sad       = 0.0;
    float xyzweight = 1.0;
    float uvweight  = 1.0;

    for (size_t ss = 0; ss < 3; ss++)
    {
      sad += xyzweight * std::abs(t1.v[ss].x - t2.v[ss].x);
      sad += xyzweight * std::abs(t1.v[ss].y - t2.v[ss].y);
      sad += xyzweight * std::abs(t1.v[ss].z - t2.v[ss].z);
      sad += uvweight * std::abs(t1.v[ss].u - t2.v[ss].u);
      sad += uvweight * std::abs(t1.v[ss].v - t2.v[ss].v);
    }
    return sad;
  }

 private:
  std::vector<Triangle> buffer_;
};

struct MeshFileHeader
{
  uint8_t texture[128];
  uint32_t vertices_size;
  uint32_t indices_size;
  uint8_t primitve;
  uint8_t index_type;
  uint8_t pos_component;
  uint8_t texcoord_component;
  uint32_t has_normal : 1;
  uint32_t has_texcoord : 1;

  MeshFileHeader()
      : vertices_size(0)
      , indices_size(0)
      , primitve(MESH_PRIMITVE_TRIANGLE)
      , index_type(32)
      , pos_component(1)
      , texcoord_component(1)
      , has_normal(0)
      , has_texcoord(0)
  {
    memset(texture, 0, sizeof(texture));
  }
};

struct MeshFileHeaderV2
{
  uint8_t texture[128];
  uint32_t vertices_size;
  uint32_t indices_size;
  float vertices_min_val;
  float vertices_max_val;
  uint8_t primitve;
  uint8_t index_type;
  uint8_t pos_component;
  uint8_t texcoord_component;
  uint32_t has_normal : 1;
  uint32_t has_texcoord : 1;

  explicit MeshFileHeaderV2(MeshFileHeader& v1)
  {
    for (size_t ss = 0; ss < 128; ss++) texture[ss] = v1.texture[ss];
    vertices_size                                   = v1.vertices_size;
    indices_size                                    = v1.indices_size;
    vertices_min_val                                = 0.0;
    vertices_max_val                                = 0.0;
    primitve                                        = v1.primitve;
    index_type                                      = v1.index_type;
    pos_component                                   = v1.pos_component;
    texcoord_component                              = v1.texcoord_component;
    has_normal                                      = v1.has_normal;
    has_texcoord                                    = v1.has_texcoord;
  }

  MeshFileHeaderV2()
      : vertices_size(0)
      , indices_size(0)
      , vertices_min_val(0.0)
      , vertices_max_val(0.0)
      , primitve(MESH_PRIMITVE_TRIANGLE)
      , index_type(32)
      , pos_component(1)
      , texcoord_component(1)
      , has_normal(0)
      , has_texcoord(0)
  {
    memset(texture, 0, sizeof(texture));
  }
};

void print(MeshFileHeader& fheader)
{
  std::cout << fheader.texture << std::endl;
  std::cout << fheader.vertices_size << std::endl;
  std::cout << fheader.indices_size << std::endl;
  std::cout << static_cast<uint32_t>(fheader.primitve) << std::endl;
  std::cout << static_cast<uint32_t>(fheader.index_type) << std::endl;
  std::cout << static_cast<uint32_t>(fheader.pos_component) << std::endl;
  std::cout << static_cast<uint32_t>(fheader.texcoord_component) << std::endl;
}

template <class T>
T FindMax(T* array, size_t size)
{
  T max = array[0];

  for (size_t ss = 1; ss < size; ss++)
  {
    if (array[ss] > max) max = array[ss];
  }

  return max;
}

template <class T>
T FindMin(T* array, size_t size)
{
  T min = array[0];

  for (size_t ss = 1; ss < size; ss++)
  {
    if (array[ss] < min) min = array[ss];
  }

  return min;
}

void Decimate(std::unique_ptr<float[]>& array, size_t size, size_t keepdigits)
{
  float scale = 1.0;

  for (size_t i = 0; i < keepdigits; i++)
  {
    scale = scale * 10.0;
  }

  for (size_t ss = 0; ss < size; ss++)
  {
    array[ss] = static_cast<float>(
        static_cast<double>(static_cast<int64_t>(array[ss] * scale)) / scale);
  }
}

void ArrayToVector(std::vector<jcy::Vertice>& buffer,
                   std::unique_ptr<float[]>& array,
                   size_t size,
                   size_t comps)
{
  for (size_t ss = 0; ss < size; ss += comps)
  {
    if (comps == 1)
    {
      jcy::Vertice v;
      v.x = array[ss];
      buffer.push_back(v);
    }
    else if (comps == 2)
    {
      jcy::Vertice v;
      v.x = array[ss];
      v.y = array[ss + 1];
      buffer.push_back(v);
    }
    else if (comps == 3)
    {
      jcy::Vertice v;
      v.x = array[ss];
      v.y = array[ss + 1];
      v.z = array[ss + 2];
      buffer.push_back(v);
    }
    else
    {
    }
  }

  if (buffer.size() != size / comps)
  {
    std::cout << "Array to vector wrong in V " << buffer.size() << " " << size
              << std::endl;
  }
}

template <class T>
void ArrayToVector(std::vector<T>& buffer,
                   std::unique_ptr<T[]>& array,
                   size_t size)
{
  for (size_t ss = 0; ss < size; ss++)
  {
    buffer.push_back(array[ss]);
  }

  if (buffer.size() != size)
  {
    std::cout << "Array to vector wrong in T" << std::endl;
  }
}

template <class T>
void IndexBufferToTriStrip(const std::vector<T>& idxbuffer,
                           const std::vector<jcy::Vertice>& vb,
                           const std::vector<jcy::Vertice>& uvb,
                           std::vector<Triangle>& tristrip)
{
  for (size_t ss = 0; ss < idxbuffer.size(); ss += 3)
  {
    Triangle tri;
    for (size_t t = 0; t < 3; t++)
    {
      tri.v[t].x = vb[idxbuffer[ss + t]].x;
      tri.v[t].y = vb[idxbuffer[ss + t]].y;
      tri.v[t].z = vb[idxbuffer[ss + t]].z;
      tri.v[t].u = uvb[idxbuffer[ss + t]].x;
      tri.v[t].v = uvb[idxbuffer[ss + t]].y;
    }
    tristrip.push_back(tri);
  }
}

int main(int, char** argv)
{
  std::ifstream ifs(argv[1], std::ifstream::in | std::ifstream::binary);
  std::ofstream ofs(argv[2], std::ofstream::out | std::ofstream::binary);
  MeshFileHeader fheader;
  jcy::ObjFile objwriter;

  ifs.read(reinterpret_cast<char*>(&fheader), sizeof(fheader));

  std::unique_ptr<float[]> vb(new float[3 * fheader.vertices_size]);
  std::unique_ptr<uint16_t[]> vb16(new uint16_t[3 * fheader.vertices_size]);
  std::unique_ptr<float[]> uvb(new float[2 * fheader.vertices_size]);
  std::unique_ptr<uint16_t[]> uvb16(new uint16_t[2 * fheader.vertices_size]);
  std::unique_ptr<uint16_t[]> ib(new uint16_t[fheader.indices_size]);

  ifs.read(reinterpret_cast<char*>(vb.get()),
           sizeof(float) * 3 * fheader.vertices_size);
  ifs.read(reinterpret_cast<char*>(uvb.get()),
           sizeof(float) * 2 * fheader.vertices_size);
  ifs.read(reinterpret_cast<char*>(ib.get()),
           sizeof(uint16_t) * fheader.indices_size);

  for (size_t ss = 0; ss < 2 * fheader.vertices_size; ss++)
  {
    uvb16[ss] = static_cast<uint16_t>(65535.0 * uvb[ss]);
  }

  MeshFileHeaderV2 fheader2(fheader);
  fheader2.vertices_min_val = FindMin(vb.get(), 3 * fheader.vertices_size);
  fheader2.vertices_max_val = FindMax(vb.get(), 3 * fheader.vertices_size);

  // ofs.write(reinterpret_cast<const char*>(&fheader2), sizeof(fheader2));
  // ofs.write(reinterpret_cast<const char*>(vb16.get()),
  //           sizeof(uint16_t) * 3 * fheader2.vertices_size);
  // ofs.write(reinterpret_cast<const char*>(uvb16.get()),
  //           sizeof(uint16_t) * 2 * fheader2.vertices_size);
  // ofs.write(reinterpret_cast<const char*>(ib.get()),
  //           sizeof(uint16_t) * fheader2.indices_size);

  std::cout << vb[0] << std::endl;
  Decimate(vb, 3 * fheader.vertices_size, 3);
  std::cout << vb[0] << std::endl;
  Decimate(uvb, 2 * fheader.vertices_size, 3);

  std::vector<jcy::Vertice> vbvec, uvbvec;
  std::vector<uint16_t> ibvec;
  std::vector<Triangle> tristrip;
  TriangleBuffer tribuffer;

  ArrayToVector(vbvec, vb, 3 * fheader.vertices_size, 3);
  ArrayToVector(uvbvec, uvb, 2 * fheader.vertices_size, 2);
  ArrayToVector(ibvec, ib, fheader2.indices_size);
  // ibvec.resize(5940);
  objwriter.SaveObjFileTrunc(
      std::string("trial.obj"), std::string("trial.mtl"), vbvec, uvbvec, ibvec);

  // for (size_t ss = 0; ss < 15; ss++)
  // {
  //   std::cout << vbvec[ibvec[ss]].x << " " << vbvec[ibvec[ss]].y << " "
  //             << vbvec[ibvec[ss]].z << " " << uvbvec[ibvec[ss]].x << " "
  //             << uvbvec[ibvec[ss]].y << " " << std::endl;
  // }

  IndexBufferToTriStrip(ibvec, vbvec, uvbvec, tristrip);

  std::fstream outfs(
      "Vertices.txt",
      std::ofstream::trunc | std::ofstream::binary | std::ofstream::out);
  std::cout << vbvec.size() << std::endl;
  for (size_t ss = 0; ss < vbvec.size(); ss++)
  {
    int16_t x, y, z;

    x = static_cast<int16_t>(vbvec[ss].x * 1000);
    y = static_cast<int16_t>(vbvec[ss].y * 1000);
    z = static_cast<int16_t>(vbvec[ss].z * 1000);

    outfs << x << " " << y << " " << z << std::endl;
    // outfs.write(reinterpret_cast<const char*>(&(x)), sizeof(int16_t));
    // outfs.write(reinterpret_cast<const char*>(&(y)), sizeof(int16_t));
    // outfs.write(reinterpret_cast<const char*>(&(z)), sizeof(int16_t));
  }

  outfs.close();
  tribuffer.Load(tristrip);
  // tribuffer.Sort(0);
  // tribuffer.SaveToFileAsBinary(std::string("tristrip.txt"));
  ofs.close();
  ifs.close();
  return 0;
}

// std::cout << std::setprecision(6) << vb[0] << vb[1] << vb[2] << std::endl;
// std::cout << std::setprecision(6) << vb[3] << vb[4] << vb[5] << std::endl;
// std::cout << "[Max in vb] " << FindMax(vb.get(), 3 * fheader.vertices_size)
//           << std::endl;
// std::cout << "[Min in vb] " << FindMin(vb.get(), 3 * fheader.vertices_size)
//           << std::endl;

// std::cout << std::setprecision(6) << uvb[0] << uvb[1] << std::endl;
// std::cout << std::setprecision(6) << uvb[2] << uvb[3] << std::endl;
// std::cout << std::setprecision(6) << uvb[4] << uvb[5] << std::endl;
// std::cout << "[Max in uvb] " << FindMax(uvb.get(), 2 * fheader.vertices_size)
//           << std::endl;
// std::cout << "[Min in uvb] " << FindMin(uvb.get(), 2 * fheader.vertices_size)
//           << std::endl;

// std::cout << ib[0] << " " << ib[1] << " " << ib[2] << std::endl;
// std::cout << ib[3] << " " << ib[4] << " " << ib[5] << std::endl;
// std::cout << ib[6] << " " << ib[7] << " " << ib[8] << std::endl;
// std::cout << "[Max in ib] " << FindMax(ib.get(), fheader.indices_size)
//           << std::endl;
// std::cout << "[Min in ib] " << FindMin(ib.get(), fheader.indices_size)
//           << std::endl;

// float min = FindMin(vb.get(), 3 * fheader.vertices_size);
// float r   = FindMax(vb.get(), 3 * fheader.vertices_size) -
//           FindMin(vb.get(), 3 * fheader.vertices_size);

// for (size_t ss = 0; ss < 3 * fheader.vertices_size; ss++)
// {
//   float mapped = ((vb[ss] - min) * 65535.0 / r);
//   vb16[ss]     = static_cast<uint16_t>(mapped);
// }
