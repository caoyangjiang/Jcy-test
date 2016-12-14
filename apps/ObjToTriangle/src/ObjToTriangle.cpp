// Copyright @ 2016 Caoyang Jiang

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "pcl/io/obj_io.h"
#include "pcl/io/ply_io.h"

int main(int argc, char** argv)
{
  pcl::PolygonMesh mesh;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<float> vbuffer;
  std::vector<int16_t> trunc;
  float min     = 99999.0;
  float max     = 0;
  int16_t min16 = 32000;
  int16_t max16 = -32000;
  std::string inputfilename, outputfilename;
  std::ofstream ofs;

  if (argc < 4)
  {
    std::cout << "not enough argument" << std::endl;
    return -1;
  }

  std::string num = std::to_string(std::atoi(argv[3]));
  num.insert(num.begin(), 3 - num.size(), '0');

  inputfilename = argv[1] + std::string("Mesh-F00") + num + std::string(".ply");
  outputfilename = argv[2] + std::string("Vertex") + num + std::string(".tri");

  std::cout << "[Reading " << inputfilename << "]" << std::endl;
  std::cout << "[Output  " << outputfilename << "]" << std::endl;
  pcl::io::loadPLYFile(inputfilename.c_str(), mesh);
  pcl::io::loadPLYFile(inputfilename.c_str(), *rgbcloud);

  std::cout << "# of polygons " << mesh.polygons.size() << std::endl;
  std::cout << "# of points " << rgbcloud->points.size() << std::endl;

  // for (size_t sz = 0; sz < 5; sz++)
  // {
  //   pcl::Vertices v;
  //   v = mesh.polygons[sz];
  //   pcl::PointXYZRGB p[3];

  //   p[0] = rgbcloud->points[v.vertices[0]];
  //   p[1] = rgbcloud->points[v.vertices[1]];
  //   p[2] = rgbcloud->points[v.vertices[2]];

  //   std::cout << "[Triangle #" << sz << "] " << std::endl;
  //   std::cout << "      [Vertex #0] " << v.vertices[0] << " " << p[0].x <<
  //   ","
  //             << p[0].y << "," << p[0].z << "," <<
  //             static_cast<uint32_t>(p[0].r)
  //             << "," << static_cast<uint32_t>(p[0].g) << ","
  //             << static_cast<uint32_t>(p[0].b) << std::endl;
  //   std::cout << "      [Vertex #1] " << v.vertices[1] << " " << p[1].x <<
  //   ","
  //             << p[1].y << "," << p[1].z << "," <<
  //             static_cast<uint32_t>(p[1].r)
  //             << "," << static_cast<uint32_t>(p[1].g) << ","
  //             << static_cast<uint32_t>(p[1].b) << std::endl;
  //   std::cout << "      [Vertex #2] " << v.vertices[2] << " " << p[2].x <<
  //   ","
  //             << p[2].y << "," << p[2].z << "," <<
  //             static_cast<uint32_t>(p[2].r)
  //             << "," << static_cast<uint32_t>(p[2].g) << ","
  //             << static_cast<uint32_t>(p[2].b) << std::endl;
  // }

  for (size_t sz = 0; sz < mesh.polygons.size(); sz++)
  {
    pcl::Vertices v;
    v = mesh.polygons[sz];
    // pcl::PointXYZRGB p[3];

    for (size_t iv = 0; iv < 3; iv++)
    {
      vbuffer.push_back(rgbcloud->points[v.vertices[iv]].x);
      vbuffer.push_back(rgbcloud->points[v.vertices[iv]].y);
      vbuffer.push_back(rgbcloud->points[v.vertices[iv]].z);
    }
  }

  for (size_t ss = 0; ss < vbuffer.size(); ss++)
  {
    if (vbuffer[ss] < min)
    {
      min = vbuffer[ss];
    }

    if (vbuffer[ss] > max)
    {
      max = vbuffer[ss];
    }
  }

  std::cout << min << " " << max << std::endl;

  // Save as triangles
  // for (size_t ss = 0; ss < vbuffer.size(); ss++)
  // {
  //   int16_t val = static_cast<int16_t>(vbuffer[ss]);
  //   trunc.push_back(val);
  // }
  // std::cout << "[Triangle count]: " << trunc.size() / 9 << std::endl;

  // Save as triangles+(fake UVs)
  for (size_t ss = 0; ss < vbuffer.size(); ss++)
  {
    int16_t val = static_cast<int16_t>(vbuffer[ss]);
    trunc.push_back(val);

    if (((ss + 1) % 3 == 0))
    {
      trunc.push_back(1);  // fake u
      trunc.push_back(2);  // fake v
    }
  }

  std::cout << "[Triangle count]: " << trunc.size() / 15 << std::endl;

  // Save index buffer
  // for (size_t sz = 0; sz < mesh.polygons.size(); sz++)
  // {
  //   pcl::Vertices v = mesh.polygons[sz];
  //   for (size_t iv = 0; iv < 3; iv++)
  //   {
  //     trunc.push_back(v.vertices[iv]);
  //   }
  // }

  // Save vertex buffer
  // for (size_t sz = 0; sz < rgbcloud->points.size(); sz++)
  // {
  //   trunc.push_back(static_cast<int16_t>(rgbcloud->points[sz].x * 10));
  //   trunc.push_back(static_cast<int16_t>(rgbcloud->points[sz].y * 10));
  //   trunc.push_back(static_cast<int16_t>(rgbcloud->points[sz].z * 10));
  // }
  // size_t rem = (rgbcloud->points.size() / 18 + 1) * 18;
  // std::cout << "Add " << rem - rgbcloud->points.size() << std::endl;
  // for (size_t sz = 0; sz < (rem - rgbcloud->points.size()); sz++)
  // {
  //   trunc.push_back(trunc.back());
  //   trunc.push_back(trunc.back());
  //   trunc.push_back(trunc.back());
  // }

  for (size_t ss = 0; ss < trunc.size(); ss++)
  {
    if (trunc[ss] < min16)
    {
      min16 = trunc[ss];
    }

    if (trunc[ss] > max16)
    {
      max16 = trunc[ss];
    }
  }

  std::cout << trunc.size() << " " << min16 << " " << max16 << std::endl;
  ofs.open(outputfilename.c_str(),
           std::ofstream::binary | std::ofstream::trunc);
  ofs.write(reinterpret_cast<const char*>(trunc.data()),
            trunc.size() * sizeof(int16_t));
  ofs.close();
}
