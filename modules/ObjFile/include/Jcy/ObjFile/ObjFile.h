// Copyright 2016 Caoyang Jiang

#ifndef MODULES_OBJFILE_INCLUDE_JCY_OBJFILE_OBJFILE_H_
#define MODULES_OBJFILE_INCLUDE_JCY_OBJFILE_OBJFILE_H_

#include <fstream>
#include <string>
#include <vector>

namespace Jcy
{
typedef struct Vertice
{
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
} Vertice;

class ObjFile
{
 public:
  ObjFile();
  ~ObjFile();

  template <class T>
  void SaveObjFile(const std::string& filename,
                   const std::string& mtlname,
                   const std::vector<Vertice>& vb,
                   const std::vector<Vertice>& uvb,
                   const std::vector<T>& faces)
  {
    std::ofstream vstream(
        filename,
        std::ofstream::out | std::ofstream::binary | std::ofstream::trunc);
    vstream << "mtllib " << mtlname << std::endl;

    for (size_t i = 0; i < vb.size(); i++)
    {
      vstream << "v"
              << " " << vb[i].x << " " << vb[i].y << " " << vb[i].z
              << std::endl;
    }

    for (size_t i = 0; i < uvb.size(); i++)
    {
      vstream << "vt"
              << " " << uvb[i].x << " " << uvb[i].y << " " << std::endl;
    }

    for (size_t i = 0; i < faces.size(); i += 3)
    {
      vstream << "f"
              << " " << 1 + faces[i] << "/" << 1 + faces[i] << " "
              << 1 + faces[i + 1] << "/" << 1 + faces[i + 1] << " "
              << 1 + faces[i + 2] << "/" << 1 + faces[i + 2] << std::endl;
    }

    vstream.close();
  }

  template <class T>
  void SaveObjFileTrunc(const std::string& filename,
                        const std::string& mtlname,
                        const std::vector<Vertice>& vb,
                        const std::vector<Vertice>& uvb,
                        const std::vector<T>& faces)
  {
    std::ofstream vstream(
        filename,
        std::ofstream::out | std::ofstream::binary | std::ofstream::trunc);
    vstream << "mtllib " << mtlname << std::endl;

    for (size_t i = 0; i < vb.size(); i++)
    {
      float x = static_cast<float>(static_cast<int>(vb[i].x * 1000.0)) / 1000.0;
      float y = static_cast<float>(static_cast<int>(vb[i].y * 1000.0)) / 1000.0;
      float z = static_cast<float>(static_cast<int>(vb[i].z * 1000.0)) / 1000.0;

      vstream << "v"
              << " " << x << " " << y << " " << z << std::endl;
    }

    for (size_t i = 0; i < uvb.size(); i++)
    {
      vstream << "vt"
              << " " << uvb[i].x << " " << uvb[i].y << " " << std::endl;
    }

    for (size_t i = 0; i < faces.size(); i += 3)
    {
      vstream << "f"
              << " " << 1 + faces[i] << "/" << 1 + faces[i] << " "
              << 1 + faces[i + 1] << "/" << 1 + faces[i + 1] << " "
              << 1 + faces[i + 2] << "/" << 1 + faces[i + 2] << std::endl;
    }

    vstream.close();
  }
};
}  // namespace Jcy
#endif  // MODULES_OBJFILE_INCLUDE_JCY_OBJFILE_OBJFILE_H_
